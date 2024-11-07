
/* version number;
 */
#define SW_VERSION_MAJOR           0
#define SW_VERSION_MINOR           1

#define PROTOCOL_VERSION           1

/* numbers */
#define PI_X_2                     (6.2831853f)
#define TX_BUF_SIZE                64

#define ADC_CHANNELS               2
#define ADC_IDX_FMOD               0

#define ADC_MIN_12b                0
#define ADC_MAX_12b                4095
#define ADC_SCALE_OFFSET           50
#define ADC_SCALE_MIN              (ADC_MIN_12b+ADC_SCALE_OFFSET)
#define ADC_SCALE_MAX              (ADC_MAX_12b-ADC_SCALE_OFFSET)
#define ADC_SCALE_DELTA            (ADC_SCALE_MAX-ADC_SCALE_MIN)

/* spectral data */
#define GMSS_CHANNELS              3
#define CH_X                       0
#define CH_Y                       1
#define CH_Z                       2
#define SPECTRE_AVG_CYCLES         60  // 50 cycles does not work reliably, Tx overlap !


/* magnetometer */
#define MGN_SENS_FACTOR            1.3f           /* run a 1.3 Gauss sensitivity scale */
#define MGN_FULLSCALE              32767.0f       /* fullscale factor                  */

#define GMSS_FFT_SIZE              256            /*!< FFT block size; 512 is not supported !  */
#define GMSS_FFT_CHANNELS          3              /*!< for all 3 channels (x,y,z) individually */

/* activation/deactivation of certain features
 */
#define ENABLE_DEBUG_OUTPUT

#define _ENABLE_USART_

/* if the ADC is not used, run the main loop via timer interrupt */
#ifndef _RUN_AP_ADC_
  #ifndef USE_TIM_INTERRUPT
    #define USE_TIM_INTERRUPT
  #endif
#endif
