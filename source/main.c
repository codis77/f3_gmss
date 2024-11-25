/*********************************************************************
*
* File :         main.c
*
*        infraSensor
*
* An application to collect magnetic field data via the onboard
* LSM303DLHC MEMS sensor, and store them on a PC attached via RS232.
* The application is based upon the STM32F3 Discovery board.
* 
* The default sampling rate is 256Hz, and the default
* RS232 connection settings are 8N1@115200bps
*
**********************************************************************
*/

#include <math.h>
#include <stdio.h>
#include "arm_math.h"
#include "stm32f30x_conf.h"
#include "stm32f3_discovery.h"
#include "stm32f3_discovery_lsm303dlhc.h"
#include "main.h"
#include "adc.h"
#include "timer.h"


/* define ------------------------------------------------------------*/
#define SENS_INDEX_1_3                   0
#define SENS_INDEX_1_9                   1
#define SENS_INDEX_2_5                   2
#define SENS_INDEX_4_0                   3
#define SENS_INDEX_4_7                   4
#define SENS_INDEX_5_6                   5
#define SENS_INDEX_8_1                   6

#define SENSDATA_NO_ACTION               0
#define SENSDATA_DO_TX                   1

#define SAMPLE_RATE                      256 /* for host communication*/

/* typedefs ----------------------------------------------------------*/

/* global variables --------------------------------------------------*/
volatile uint32_t      TimingCounter   = 0;
volatile uint32_t      sampleFlag      = 0;
volatile uint32_t      apValue         = 0;    // actual ADC value
volatile uint32_t      transmitRunning = 0;
volatile int32_t       mCycle          = 0;
volatile char          txBuffer[TX_BUF_SIZE];
volatile uint32_t      txSendCount     = 0;
volatile uint32_t      txIndex         = 0;
volatile uint32_t      ubtnChange      = 0;  /* flag for user button */

float_t                MagBuffer[3]    = {0.0f};
float_t                AccBuffer[3]    = {0.0f};
float_t                Buffer[3]       = {0.0f};
float_t                gmssFFTbuf[GMSS_FFT_CHANNELS][GMSS_FFT_SIZE] = {{0},{0}};
float_t                S3Data[GMSS_FFT_CHANNELS][GMSS_FFT_SIZE]     = {{0},{0}};
float_t                STxData[GMSS_FFT_CHANNELS][GMSS_FFT_SIZE]    = {{0},{0}};
float_t                fftInBuf[GMSS_FFT_CHANNELS][GMSS_FFT_SIZE]   = {{0},{0}};

static arm_rfft_fast_instance_f32  S;
static arm_status                  status;


/* external variables -------------------------------------------------*/

/* private variables --------------------------------------------------*/

/* Private function prototypes ----------------------------------------*/
static void      initUSART    (void);
static void      sendHeader   (char *buffer, uint32_t len);
static void      initsendData (void);
static void      evb_LedsInit (void);
static void      evbLedsOn    (void);
static void      evbLedsOff   (void);
static void      pbInit       (void);

// --- magnetometer ---
static void      configMagnCompass (void);
static void      readCompassAcc    (float_t *pfData);
static uint32_t  readCompassMag    (float_t *pfData);
static void      initFFTBuffers    (void);

int32_t          processMagnData         (int32_t mCycle, float32_t *MagBuffer);
int32_t          updateSpectreStatistics (int32_t mCycle);
void             transmitSpectre         (void);

uint32_t         LSM303DLHC_TIMEOUT_UserCallback (void);
uint32_t         L3GD20_TIMEOUT_UserCallback (void);


//uint16_t  outValBuf[1000];

/** --- code start---
 **/
int  main (void)
{
    uint32_t           fs, i, sensitivity;
    uint16_t           tVal;
    RCC_ClocksTypeDef  RCC_Clocks;

    /* SysTick every millisecond */
    RCC_GetClocksFreq (&RCC_Clocks);
    fs = RCC_Clocks.HCLK_Frequency / 1000;
    SystemCoreClockUpdate ();
    SysTick_Config (fs);

    /* configure remaining periphery */
    initUSART ();
    evb_LedsInit ();
    pbInit ();

    /* initialize the sampling timer */
//  SampleTimerConfig (TIM3_TRGO_VALUE_0K4);    /* 400Hz sample rate */
    SampleTimerConfig (TIM3_TRGO_VALUE_0K256);   /* try 256Hz instead */

    /* buffer and magnetic compass init */
    initFFTBuffers ();
    configMagnCompass ();

    /* init the FFT module */
    status = arm_rfft_fast_init_f32 (&S, GMSS_FFT_SIZE);

    sprintf ((char *)txBuffer, "#gmss_1 : %1d,%3d @%d\n", GMSS_CHANNELS, GMSS_FFT_SIZE, SAMPLE_RATE);
    sendHeader ((char *)txBuffer, strlen((char *)txBuffer));

    /* main loop, 2.5ms cycle (400Hz magnetometer sample rate);
     * get latest magnetometer values, process, and store;
     */
    while (1)
    {
        if (sampleFlag)
        {
            sampleFlag = 0;

            /* run the I2C magnetometer read ... */
            sensitivity = readCompassMag (MagBuffer);

            /* process sensor data, including a 512-pt. FFT */
            i = processMagnData (mCycle, MagBuffer);

            // update the cycle counter
            if (++mCycle >= GMSS_FFT_SIZE)
                mCycle = 0;

            // start data transmission to host when ready
            if (i == SENSDATA_DO_TX)
                transmitSpectre ();
        }
    }
}



/* Configures the USART Peripheral;
 * using UART1 at escape routes PB10=TX, PB11=RX
 * [(alternatively : UART3 at escape routes PC4=TX, PC5=RX)]
 * [UART1 = PC4/PC5 = tied to internal ST-Link VCP pins !!!]
 */
static void  initUSART (void)
{
    USART_InitTypeDef  USART_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;
    GPIO_InitTypeDef   GPIO_InitStructure;

    /* Enable GPIO clock */
    RCC_AHBPeriphClockCmd (RCC_AHBPeriph_GPIOB, ENABLE);

    /* Enable USART clock */
    RCC_APB1PeriphClockCmd (RCC_APB1Periph_USART3, ENABLE);

    /* Connect PXx to USARTx_Tx and USARTx_Rx */
    GPIO_PinAFConfig (GPIOB, GPIO_PinSource10, GPIO_AF_7);
    GPIO_PinAFConfig (GPIOB, GPIO_PinSource11, GPIO_AF_7);

    /* Configure USART Tx and Rx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init (GPIOB, &GPIO_InitStructure);

    /* USART1-configuration = 115200 Baud, 8 Bits, 1 Stopbit, parity off, RTS/CTS offs,
     */
    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init (USART3, &USART_InitStructure);

    /* NVIC; configure the Priority Group to 2 bits */
    NVIC_PriorityGroupConfig (NVIC_PriorityGroup_2);

    /* Enable the USARTx Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init (&NVIC_InitStructure);

    /* enable USART and interrupts */
    USART_ITConfig (USART3, USART_IT_RXNE, ENABLE);
    USART_Cmd (USART3, ENABLE);
}



/* init Discovery board LEDs
 */
static void  evb_LedsInit (void)
{
    STM_EVAL_LEDInit (LED3);    /* red    */
    STM_EVAL_LEDInit (LED4);    /* blue   */
    STM_EVAL_LEDInit (LED5);    /* orange */
    STM_EVAL_LEDInit (LED6);    /* green  */
    STM_EVAL_LEDInit (LED7);    /* green  */
    STM_EVAL_LEDInit (LED8);    /* orange */
    STM_EVAL_LEDInit (LED9);    /* blue   */
    STM_EVAL_LEDInit (LED10);   /* red    */
}



/* all user LEDs off
 */
static void  evbLedsOff (void)
{
    STM_EVAL_LEDOff (LED3);
    STM_EVAL_LEDOff (LED4);
    STM_EVAL_LEDOff (LED5);
    STM_EVAL_LEDOff (LED6);
    STM_EVAL_LEDOff (LED7);
    STM_EVAL_LEDOff (LED8);
    STM_EVAL_LEDOff (LED9);
    STM_EVAL_LEDOff (LED10);
}



/* all user LEDs on
 */
static void  evbLedsOn (void)
{
    STM_EVAL_LEDOn (LED3);
    STM_EVAL_LEDOn (LED4);
    STM_EVAL_LEDOn (LED5);
    STM_EVAL_LEDOn (LED6);
    STM_EVAL_LEDOn (LED7);
    STM_EVAL_LEDOn (LED8);
    STM_EVAL_LEDOn (LED9);
    STM_EVAL_LEDOn (LED10);
}



/* init "User" button
 */
static void  pbInit (void)
{
    STM_EVAL_PBInit (BUTTON_USER, BUTTON_MODE_EXTI);
}



/* send data header;
 * sequential, access USART1 directly
 */
static void  sendHeader (char *buffer, uint32_t len)
{
    int  index;

    index = 0;

    // send string in a busy loop
    while (index < len)
    {
        USART3->TDR = txBuffer[index++];
        while ((USART3->ISR & USART_FLAG_TXE)==0);
    }
}



/* initialize the transmission of a data item via serial line;
 * using the serial interrupt, and thus fixed to use a predefined buffer (txBuffer);
 * function is executed in "real time", and supposedly finishes
 * before the transmit buffer is overwritten;
 * otherwise Tx data could overlap / be corrupted;
 */
static void  sendDataItem (void)
{
//    sprintf ((char *) txBuffer, "%hu\n", data);

    // initialize UART TXE interrupt, and send first character
    USART3->CR1 |= USART_CR1_TXEIE;
    txIndex      = 1;
    USART3->TDR  = txBuffer[0];
}



static void  initsendData (void)
{
    // initialize UART TXE interrupt, and send first character
    txIndex      = 1;                // next character
    USART3->TDR  = txBuffer[0];
    USART3->CR1 |= USART_CR1_TXEIE;
}



/* Configure the MEMS to magnetic compass application.
 */
void  configMagnCompass (void)
{
    LSM303DLHCMag_InitTypeDef          LSM303DLHC_InitStructure;
    LSM303DLHCAcc_InitTypeDef          LSM303DLHCAcc_InitStructure;
    LSM303DLHCAcc_FilterConfigTypeDef  LSM303DLHCFilter_InitStructure;

    /* Configure MEMS magnetometer main parameters: temp, working mode, full Scale and Data rate */
    LSM303DLHC_InitStructure.Temperature_Sensor = LSM303DLHC_TEMPSENSOR_DISABLE;
    LSM303DLHC_InitStructure.MagOutput_DataRate = LSM303DLHC_ODR_220_HZ;
    LSM303DLHC_InitStructure.MagFull_Scale      = LSM303DLHC_FS_1_3_GA;               // ±1.3Gauss (±8.1 Gauss)
    LSM303DLHC_InitStructure.Working_Mode       = LSM303DLHC_CONTINUOS_CONVERSION;
    LSM303DLHC_MagInit (&LSM303DLHC_InitStructure);

     /* Configuring the accelerometer main parameters - unused here */
    LSM303DLHCAcc_InitStructure.Power_Mode         = LSM303DLHC_NORMAL_MODE;
    LSM303DLHCAcc_InitStructure.AccOutput_DataRate = LSM303DLHC_ODR_50_HZ;
    LSM303DLHCAcc_InitStructure.Axes_Enable        = LSM303DLHC_AXES_ENABLE;
    LSM303DLHCAcc_InitStructure.AccFull_Scale      = LSM303DLHC_FULLSCALE_2G;
    LSM303DLHCAcc_InitStructure.BlockData_Update   = LSM303DLHC_BlockUpdate_Continous;
    LSM303DLHCAcc_InitStructure.Endianness         = LSM303DLHC_BLE_LSB;
    LSM303DLHCAcc_InitStructure.High_Resolution    = LSM303DLHC_HR_ENABLE;
    LSM303DLHC_AccInit (&LSM303DLHCAcc_InitStructure);

    /* Fill the accelerometer LPF structure */
    LSM303DLHCFilter_InitStructure.HighPassFilter_Mode_Selection   = LSM303DLHC_HPM_NORMAL_MODE;
    LSM303DLHCFilter_InitStructure.HighPassFilter_CutOff_Frequency = LSM303DLHC_HPFCF_16;
    LSM303DLHCFilter_InitStructure.HighPassFilter_AOI1             = LSM303DLHC_HPF_AOI1_DISABLE;
    LSM303DLHCFilter_InitStructure.HighPassFilter_AOI2             = LSM303DLHC_HPF_AOI2_DISABLE;
    LSM303DLHC_AccFilterConfig (&LSM303DLHCFilter_InitStructure);
}



/* read the magnetometer data
 * parameter  pfData = pointer to the data out
 * retval     the current (XY) sensitivity setting
 */
uint32_t  readCompassMag (float_t *pfData)
{
    static uint8_t  buffer[6] = {0};
    uint32_t        sensindex = 0;
    uint16_t        Magn_Sensitivity_XY = 0, Magn_Sensitivity_Z = 0;
    float_t         f;
    uint8_t         CTRLB = 0;

    LSM303DLHC_Read (MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, &CTRLB, 1);
    LSM303DLHC_Read (MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M, buffer, 1);
    LSM303DLHC_Read (MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_L_M, buffer+1, 1);
    LSM303DLHC_Read (MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_M, buffer+2, 1);
    LSM303DLHC_Read (MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_L_M, buffer+3, 1);
    LSM303DLHC_Read (MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_M, buffer+4, 1);
    LSM303DLHC_Read (MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_L_M, buffer+5, 1);

#if 0
    /* Switch the sensitivity set in the CRTLB */
    switch (CTRLB & 0xE0)
    {
        case LSM303DLHC_FS_1_3_GA:
            Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_3Ga;
            Magn_Sensitivity_Z  = LSM303DLHC_M_SENSITIVITY_Z_1_3Ga;
            sensindex           = SENS_INDEX_1_3;
            break;
        case LSM303DLHC_FS_1_9_GA:
            Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_9Ga;
            Magn_Sensitivity_Z  = LSM303DLHC_M_SENSITIVITY_Z_1_9Ga;
            sensindex           = SENS_INDEX_1_9;
            break;
        case LSM303DLHC_FS_2_5_GA:
            Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_2_5Ga;
            Magn_Sensitivity_Z  = LSM303DLHC_M_SENSITIVITY_Z_2_5Ga;
            sensindex           = SENS_INDEX_2_5;
            break;
        case LSM303DLHC_FS_4_0_GA:
            Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4Ga;
            Magn_Sensitivity_Z  = LSM303DLHC_M_SENSITIVITY_Z_4Ga;
            sensindex           = SENS_INDEX_4_0;
            break;
        case LSM303DLHC_FS_4_7_GA:
            Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4_7Ga;
            Magn_Sensitivity_Z  = LSM303DLHC_M_SENSITIVITY_Z_4_7Ga;
            sensindex           = SENS_INDEX_4_7;
            break;
        case LSM303DLHC_FS_5_6_GA:
            Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_5_6Ga;
            Magn_Sensitivity_Z  = LSM303DLHC_M_SENSITIVITY_Z_5_6Ga;
            sensindex           = SENS_INDEX_5_6;
            break;
        case LSM303DLHC_FS_8_1_GA:
            Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_8_1Ga;
            Magn_Sensitivity_Z  = LSM303DLHC_M_SENSITIVITY_Z_8_1Ga;
            sensindex           = SENS_INDEX_8_1;
            break;
    }

    pfData[0] = (float_t) ((int16_t)(((uint16_t)buffer[0] << 8) + buffer[1])*1000) / Magn_Sensitivity_XY;
    pfData[1] = (float_t) ((int16_t)(((uint16_t)buffer[2] << 8) + buffer[3])*1000) / Magn_Sensitivity_XY;
    pfData[2] = (float_t) ((int16_t)(((uint16_t)buffer[4] << 8) + buffer[5])*1000) / Magn_Sensitivity_Z;
#endif


    f         = (float_t) (int16_t)(((uint16_t)buffer[0] << 8) + buffer[1]);
    pfData[0] = MGN_SENS_FACTOR * f / MGN_FULLSCALE;
    f         = (float_t) (int16_t)(((uint16_t)buffer[2] << 8) + buffer[3]);
    pfData[1] = MGN_SENS_FACTOR * f / MGN_FULLSCALE;
    f         = (float_t) (int16_t)(((uint16_t)buffer[4] << 8) + buffer[5]);
    pfData[2] = MGN_SENS_FACTOR * f / MGN_FULLSCALE;

    // sensitivity is identical for both XY and Z axis, but the factors are slightly different
    return (sensindex);
}



/* LSM303 I2C timeout callback
 */
uint32_t  LSM303DLHC_TIMEOUT_UserCallback (void)
{
    return 0;
}



/* L3GD20 I2C timeout callback
 */
uint32_t  L3GD20_TIMEOUT_UserCallback (void)
{
    return 0;
}



/* clear the FFT buffer (for all 3 channels)
 */
static void  initFFTBuffers (void)
{
    int  i, k;

    for (k=0; k<GMSS_FFT_CHANNELS; k++)
        for (i=0; i<GMSS_FFT_SIZE; i++)
        {
            gmssFFTbuf[k][i] = 0.0f;
            fftInBuf[k][i]   = 0.0f;
            S3Data[k][i]     = 0.0f;
        }
}



/* gather <GMSS_FFT_SIZE> values for each channel,
 * and perform a FFT fore each (x,y,z);
 * the mCycle count runs from 0 to 255
 */
int32_t  processMagnData (int32_t mCycle, float32_t *MagBuffer)
{
    float_t *pfft;

    if (mCycle > (GMSS_FFT_SIZE+1))
        return (SENSDATA_NO_ACTION); // error ?!?

    fftInBuf[CH_X][mCycle-1] = MagBuffer[CH_X];
    fftInBuf[CH_Y][mCycle-1] = MagBuffer[CH_Y];
    fftInBuf[CH_Z][mCycle-1] = MagBuffer[CH_Z];

    if (mCycle < (GMSS_FFT_SIZE-1))
        return (SENSDATA_NO_ACTION);

    STM_EVAL_LEDOn (LED4);

    pfft = &(fftInBuf[CH_X][0]);
    arm_rfft_fast_f32 (&S, pfft, gmssFFTbuf[CH_X], 0);

    pfft = &(fftInBuf[CH_Y][0]);
    arm_rfft_fast_f32 (&S, pfft, gmssFFTbuf[CH_Y], 0);

    pfft = &(fftInBuf[CH_Z][0]);
    arm_rfft_fast_f32 (&S, pfft, gmssFFTbuf[CH_Z], 0);

    STM_EVAL_LEDOff (LED4);

    return (updateSpectreStatistics (mCycle));
}



/* average the spectre over <SPECTRE_AVG_CYCLES> FFT cycles;
 * until the <SPECTRE_AVG_CYCLES> count is reached, values are
 * only summed up, and SENSDATA_NO_ACTION is returned;
 * when <SPECTRE_AVG_CYCLES> is reached, the average is calculated,
 * the results are copied to t dedicated transmit buffer, and
 * SENSDATA_DO_TX is returned, signalling that TX data are ready 
 */
int32_t  updateSpectreStatistics (int32_t mCycle)
{
    static int32_t  sstime = 0; // SPECTRE_AVG_CYCLES;
    int32_t         i;

    for (i=0; i<GMSS_FFT_SIZE; i++)
    {
        S3Data[CH_X][i] += gmssFFTbuf[CH_X][i];
        S3Data[CH_Y][i] += gmssFFTbuf[CH_Y][i];
        S3Data[CH_Z][i] += gmssFFTbuf[CH_Z][i];
    }
    sstime++;
    if (sstime >= SPECTRE_AVG_CYCLES)
    {
        for (i=0; i<GMSS_FFT_SIZE; i++)
        {
            S3Data[CH_X][i] /= (float32_t) SPECTRE_AVG_CYCLES;
            S3Data[CH_Y][i] /= (float32_t) SPECTRE_AVG_CYCLES;
            S3Data[CH_Z][i] /= (float32_t) SPECTRE_AVG_CYCLES;
            STxData[CH_X][i] = S3Data[CH_X][i];
            STxData[CH_Y][i] = S3Data[CH_Y][i];
            STxData[CH_Z][i] = S3Data[CH_Z][i];
        }
        sstime = 0;
        return (SENSDATA_DO_TX);
    }
    else
        return (SENSDATA_NO_ACTION);
}



/* initiate the serial transmission;
 * prepare the first package, and get the interrupt chain going;
 * alltogether, (1 + GMSS_FFT_SIZE/2) triples are transmitted;
 * redundant items of the RFFT result are discarded
 */
void  transmitSpectre (void)
{
    txSendCount = 0;
    STM_EVAL_LEDOn (LED3);
    sprintf ((char *)txBuffer, "%1i:%f,%f,%f\n", txSendCount, STxData[CH_X][0], STxData[CH_Y][0], STxData[CH_Z][0]);
    txSendCount++;
    initsendData ();
}
