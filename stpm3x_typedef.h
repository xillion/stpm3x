#ifndef _STPM3X_TYPEDEF_H_
#define _STPM3X_TYPEDEF_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

#define STPM_DATA_REG_NB_BLOCKS       49
#define STPM_DSP_DATA_REG_NB_BLOCKS   21
#define STPM_PH1_DATA_REG_NB_BLOCKS   12
#define STPM_PH2_DATA_REG_NB_BLOCKS   12
#define STPM_TOT_DATA_REG_NB_BLOCKS   4
#define STPM_CTRL_REG_NB_BLOCKS       21

#define METRO_BUFF_COM_MAXSIZE  ((uint8_t)(40))

#define CRC_8 (0x07)
#define STPM3x_FRAME_LEN (5)

#define WAIT_DURATION   500   /* 500 * 1 ms = 500 ms */

 /** @defgroup STPM BaudRate for UART
  * @{
  */
#define STPM_UART_BAUDRATE_2400   ((uint32_t)0x1A0B) /*!< 2400 */
#define STPM_UART_BAUDRATE_9600   ((uint32_t)0x683)  /*!< 9600 : it is the value after reset of STPM */
#define STPM_UART_BAUDRATE_19200  ((uint32_t)0x341)  /*!< 19200 */
#define STPM_UART_BAUDRATE_57600  ((uint32_t)0x116)  /*!< 57600 */
#define STPM_UART_BAUDRATE_115200 ((uint32_t)0x8B)   /*!< 115200 */
#define STPM_UART_BAUDRATE_230400 ((uint32_t)0x45)   /*!< 230400 */
#define STPM_UART_BAUDRATE_460800 ((uint32_t)0x23)   /*!< 460800 */

//#define METRO_STPM_PERIPH_BASE         0x0000
//#define METRO_STPM       ((METRO_STPM_TypeDef *)   METRO_STPM_PERIPH_BASE)   /*!< Metrology Peripheral definition */

typedef enum
{
    STPM3X_SCS = 0, // Chip-select pin     [input]
    STPM3X_SYN,     // Synchronization pin [input]
    STPM3X_INT1,    // Interrupt 1 pin     [output]
    STPM3X_INT2,    // Interrupt 2 pin     [output]
    STPM3X_EN,      // Shutdown pin        [input]
    STPM3X_PINS_AMOUNT
}stpm3x_pins_t;

typedef struct
{
  PinName pin;
  int     RESET;
  int     SET;
}stpm3x_pin_t;

typedef struct
{
  stpm3x_pin_t pins[STPM3X_PINS_AMOUNT];
} stpm3x_pinout_t;

typedef enum
{
    NULLDEV=0,
    STPM32,
    STPM33,
    STPM34
}stpm3x_device_t;

typedef enum
{
  RESET_SW,
  RESET_HW
}stmp3x_reset_t;

typedef enum 
{
  LATCH_HW = 1,  
  LATCH_SW,
  LATCH_AUTO
 }stpm3x_latch_t;

typedef enum 
{
  X2 = 0,  
  X4,
  X8, 
  X16  
}stpm3x_gain_t; 

/**
  * @brief STPM3X  Vref device definition
  *
  */  
   
typedef enum 
{
  EXT_VREF =0,
  INT_VREF  
}stpm3x_vref_t;

typedef enum
{
    F50HZ = 0,
    F60HZ
}stpm3x_ref_freq_t;

typedef enum
{
    HPF_ON = 0,
    HPF_OFF
}stpm3x_filter_t;

 /**
  * @brief STPM3X  Current CHANNEL definition
  *
  */  
   
typedef enum 
{
  PRIMARY = 0,  
  SECONDARY,
  ALGEBRAIC, 
  SIGMA_DELTA  
}stpm3x_led_channel_t; 


 /**
  * @brief STPM3X  LED Slection type
  *
  */  
   
typedef enum 
{
  LED1 = 1,  
  LED2 
}stpm3x_led_selection_t; 

 /**
  * @brief STPM3X  LED Slection type
  *
  */  
   
typedef enum 
{
  LED_ON = 0,  
  LED_OFF 
}stpm3x_led_output_t;

/**
  * @brief STPM3X  Power selection type
  *
  */  
   
typedef enum 
{
  W_ACTIVE = 1,  
  F_ACTIVE,
  REACTIVE, 
  APPARENT_RMS,
  APPARENT_VEC,
  MOM_WIDE_ACT,
  MOM_FUND_ACT
}stpm3x_power_selection_t;

typedef enum 
{
  LED_W_ACTIVE = 0,  
  LED_F_ACTIVE,
  LED_REACTIVE, 
  LED_APPARENT_RMS
}stpm3x_led_power_selection_t;


typedef enum 
{
  E_W_ACTIVE = 1,  
  E_F_ACTIVE,
  E_REACTIVE, 
  E_APPARENT,
  NB_MAX_TYPE_NRG
}stpm3x_energy_selection_t;

/**
  * @brief STPM3X  Calculation Power selection type
  *
  */  
   
typedef enum 
{
  FROM_RMS = 1,  
  FROM_PWIDE,
  FROM_PFUND
}stpm3x_calculation_power_selection_t;

/**
  * @brief STPM3X  Voltage read type
  *
  */  
typedef enum 
{
  V_WIDE = 1,  
  V_FUND
 }stpm3x_voltage_type_t;

/**
  * @brief STPM3X  Current read type
  *
  */  
typedef enum 
{
  C_WIDE = 1,  
  C_FUND
 }stpm3x_current_type_t;



/**
  * @brief STPM3X  Tamper Tolerance type
  *
  */  
typedef enum 
{
  TOL_12_5 = 0,  
  TOL_8_33,
  TOL_6_25,
  TOL_3_125,
  NO_CHANGE_TOL
 }stpm3x_tamper_tolerance_t;


/**
  * @brief STPM3X  ZCR Signal Selection
  *
  */  
typedef enum 
{
  ZCR_SEL_V1 = 0,  
  ZCR_SEL_C1,
  ZCR_SEL_V2,
  ZCR_SEL_C2,
  NO_CHANGE_ZCR
 }stpm3x_zcr_sel_t;

 
 /**
  * @brief STPM3X  CLK  Selection
  *
  */  
typedef enum 
{
  CLK_SEL_7KHz = 0,  
  CLK_SEL_4MHz,
  CLK_SEL_4MHz_50,
  CLK_SEL_16MHz,
  NO_CHANGE_CLK
 }stpm3x_clk_sel_t;
 
typedef enum 
{
  TC_n30, // -30
  TC_0,   //   0
  TC_30,  //  30
  TC_60,  //  60
  TC_90,  //  90
  TC_120, // 120
  TC_150, // 150
  TC_180  // 180
}stpm3x_termo_comp_t;
   
  /**
  * @brief STPM3X  Live Event type
  *
  */  
typedef enum 
{
  ALL_LIVE_EVENTS =0,
  LIVE_EVENT_REFRESHED,
  LIVE_EVENT_WRONG_INSERTION,
  LIVE_EVENT_VOLTAGE_SAG,  
  LIVE_EVENT_VOLTAGE_SWELL,
  LIVE_EVENT_CURRENT_SWELL,
  LIVE_EVENT_VOLTAGE_ZCR,
  LIVE_EVENT_CURRENT_ZCR,  
  LIVE_EVENT_VOLTAGE_PERIOD_STATUS,
  LIVE_EVENT_VOLTAGE_SIGNAL_STUCK,
  LIVE_EVENT_CURRENT_SIGNAL_STUCK,
  LIVE_EVENT_CURRENT_TAMPER,
  LIVE_EVENT_CURRENT_SIGN_CHANGE_APPARENT_POWER,
  LIVE_EVENT_CURRENT_SIGN_CHANGE_REACTIVE_POWER,
  LIVE_EVENT_CURRENT_SIGN_CHANGE_FUNDAMENTAL_POWER,
  LIVE_EVENT_CURRENT_SIGN_CHANGE_ACTIVE_POWER,
  LIVE_EVENT_CURRENT_OVERFLOW_APPARENT_NRJ,
  LIVE_EVENT_CURRENT_OVERFLOW_REACTIVE_NRJ,
  LIVE_EVENT_CURRENT_OVERFLOW_FUNDAMENTAL_NRJ,
  LIVE_EVENT_CURRENT_OVERFLOW_ACTIVE_NRJ,
  LIVE_EVENT_CURRENT_NAH
 }stpm3x_live_event_type_t;

  /**
  * @brief STPM3X Status type
  *
  */  
typedef enum 
{
  ALL_STATUS = 0,
  STATUS_REFRESHED,
  STATUS_TAMPER_DETECTED,
  STATUS_TAMPER_OR_WRONG,
  STATUS_VOLTAGE_SWELL_DOWN,
  STATUS_VOLTAGE_SWELL_UP,
  STATUS_VOLTAGE_SAG_DOWN,
  STATUS_VOLTAGE_SAG_UP,    
  STATUS_VOLTAGE_PERIOD_STATUS,
  STATUS_VOLTAGE_SIGNAL_STUCK,
  STATUS_CURRENT_OVERFLOW_APPARENT_NRJ,
  STATUS_CURRENT_OVERFLOW_REACTIVE_NRJ,
  STATUS_CURRENT_OVERFLOW_FUNDAMENTAL_NRJ,
  STATUS_CURRENT_OVERFLOW_ACTIVE_NRJ,
  STATUS_CURRENT_SIGN_APPARENT_POWER,
  STATUS_CURRENT_SIGN_CHANGE_REACTIVE_POWER,
  STATUS_CURRENT_SIGN_CHANGE_FUNDAMENTAL_POWER,
  STATUS_CURRENT_SIGN_CHANGE_ACTIVE_POWER,
  STATUS_CURRENT_SWELL_DOWN,
  STATUS_CURRENT_SWELL_UP,
  STATUS_CURRENT_NAH_TMP,
  STATUS_CURRENT_SIGNAL_STUCK,
  STATUS_NO_DEVICE
 }stpm3x_status_type_t;
 
  /**
  * @brief STPM3X Status type
  *
  */  
typedef enum 
{
  ALL_STPM_LINK_STATUS = 0,
  STATUS_STPM_UART_LINK_BREAK,
  STATUS_STPM_UART_LINK_CRC_ERROR,
  STATUS_STPM_UART_LINK_TIME_OUT_ERROR,
  STATUS_STPM_UART_LINK_FRAME_ERROR,
  STATUS_STPM_UART_LINK_NOISE_ERROR,
  STATUS_STPM_UART_LINK_RX_OVERRUN,
  STATUS_STPM_UART_LINK_TX_OVERRUN,    
  STATUS_STPM_SPI_LINK_RX_FULL,
  STATUS_STPM_SPI_LINK_TX_EMPTY,
  STATUS_STPM_LINK_READ_ERROR,
  STATUS_STPM_LINK_WRITE_ERROR,
  STATUS_STPM_SPI_LINK_CRC_ERROR,
  STATUS_STPM_SPI_LINK_UNDERRUN,
  STATUS_STPM_SPI_LINK_OVERRRUN,
 }stpm3x_link_irq_status_type_t;
  
 /**
  * @brief STPM3X  Boolean  type
  *
  */  
typedef enum 
{
  BOOL_FALSE = 0,
  BOOL_TRUE  
}stpm3x_bool_type_t;
  

typedef enum 
{
  INT_NONE_CHANNEL=0,
  INT_CHANNEL_1,  
  INT_CHANNEL_2,
  CHANNEL_TAMPER
}stpm3x_internal_channel_t; 

typedef enum
{
  ADDR_DSPCTRL1 		= 0x00,
  ADDR_DSPCTRL2 		= 0x02,
  ADDR_DSPCTRL3 		= 0x04,
  ADDR_DSPCTRL4 		= 0x06,
  ADDR_DSPCTRL5 		= 0x08,
  ADDR_DSPCTRL6 		= 0x0A,
  ADDR_DSPCTRL7 		= 0x0C,
  ADDR_DSPCTRL8 		= 0x0E,
  ADDR_DSPCTRL9 		= 0x10,
  ADDR_DSPCTRL10 		= 0x12,
  ADDR_DSPCTRL11 		= 0x14,
  ADDR_DSPCTRL12 		= 0x16,
  ADDR_DFECTRL1 		= 0x18,
  ADDR_DFECTRL2 		= 0x1A,
  ADDR_DSPIRQ1 		  = 0x1C,
  ADDR_DSPIRQ2 		  = 0x1E,
  ADDR_DSPSR1 		  = 0x20,
  ADDR_DSPSR2 		  = 0x22,
  ADDR_UARTSPICR1 	= 0x24,
  ADDR_UARTSPICR2 	= 0x26,
  ADDR_UARTSPISR 		= 0x28,

  ADDR_DSPEVENT1 		= 0x2A,
  ADDR_DSPEVENT2 		= 0x2C,
  ADDR_DSP_REG1		  = 0x2E,
  ADDR_DSP_REG2		  = 0x30,
  ADDR_DSP_REG3		  = 0x32,
  ADDR_DSP_REG4		  = 0x34,
  ADDR_DSP_REG5		  = 0x36,
  ADDR_DSP_REG6		  = 0x38,
  ADDR_DSP_REG7		  = 0x3A,
  ADDR_DSP_REG8		  = 0x3C,
  ADDR_DSP_REG9		  = 0x3E,
  ADDR_DSP_REG10		= 0x40,
  ADDR_DSP_REG11		= 0x42,
  ADDR_DSP_REG12		= 0x44,
  ADDR_DSP_REG13		= 0x46,
  ADDR_DSP_REG14		= 0x48,
  ADDR_DSP_REG15		= 0x4A,
  ADDR_DSP_REG16		= 0x4C,
  ADDR_DSP_REG17		= 0x4E,
  ADDR_DSP_REG18		= 0x50,
  ADDR_DSP_REG19		= 0x52,

  ADDR_CH1_REG1		  = 0x54,
  ADDR_CH1_REG2		  = 0x56,
  ADDR_CH1_REG3		  = 0x58,
  ADDR_CH1_REG4		  = 0x5A,
  ADDR_CH1_REG5		  = 0x5C,
  ADDR_CH1_REG6		  = 0x5E,
  ADDR_CH1_REG7		  = 0x60,
  ADDR_CH1_REG8		  = 0x62,
  ADDR_CH1_REG9		  = 0x64,
  ADDR_CH1_REG10	  = 0x66,
  ADDR_CH1_REG11	  = 0x68,
  ADDR_CH1_REG12	  = 0x6A,
  ADDR_CH2_REG1		  = 0x6C,
  ADDR_CH2_REG2		  = 0x6E,
  ADDR_CH2_REG3		  = 0x70,
  ADDR_CH2_REG4		  = 0x72,
  ADDR_CH2_REG5		  = 0x74,
  ADDR_CH2_REG6		  = 0x76,
  ADDR_CH2_REG7		  = 0x78,
  ADDR_CH2_REG8		  = 0x7A,
  ADDR_CH2_REG9		  = 0x7C,
  ADDR_CH2_REG10	  = 0x7E,
  ADDR_CH2_REG11	  = 0x80,
  ADDR_CH2_REG12	  = 0x82,
  
  ADDR_TOT_REG1		  = 0x84,
  ADDR_TOT_REG2		  = 0x86,
  ADDR_TOT_REG3		  = 0x88,
  ADDR_TOT_REG4		  = 0x8A
}stpm3x_address_t;

/*


*/

typedef struct 
{
    uint32_t CLRSS_TO :4;  //Set duration of secondary channel reset signal to clear sag and swell registers
    uint32_t CLRSS    :1;  //Clear sag and swell time register and history bits for primary channel, auto-reset to '0'
    uint32_t ENVREF   :1;  //Enable internal voltage reference for primary channel: 0: reference disabled 1: reference enabled
    uint32_t TC       :3;  //Temperature compensation coefficient selection for primary channel voltage reference V REF1
    uint32_t          :8;  //Reserved
    uint32_t AEM      :1;  //Apparent energy mode for primary channel: 0: use apparent RMS power 1: use apparent vectorial power
    uint32_t APM      :1;  //Apparent vectorial power mode for primary channel: 0: use fundamental power 1: use active power
    uint32_t BHPFV    :1;  //Bypass hi-pass filter for primary voltage channel: 0: HPF enabled 1: HPF bypassed
    uint32_t BHPFC    :1;  //Bypass hi-pass filter for primary current channel: 0: HPF enabled 1: HPF bypassed
    uint32_t ROC      :1;  //Add Rogowski integrator to primary current channel filtering pipeline: 0: integrator bypassed 1: integrator enabled
    uint32_t          :2;  //Reserved
    uint32_t LPW      :4;  //LED speed dividing factor: 0x0 = 2^(-4), 0xF = 2^11 Default 0x4 = 1
    uint32_t LPS      :2;  //LED pulse-out power selection: LPS [1:0]: 00,01,10,11 LED output: active, fundamental, reactive, apparent
    uint32_t LCS      :2;  //LED pulse-out channel selection: LCS [1:0]: 00,01,10,11 LED: primary channels, secondary channels, cumulative, sigma-delta bitstream
} dspctrl1_t;
#define dspctrl2_t dspctrl1_t;

/*


*/
typedef struct 
{
    uint32_t SAG_TIME_THR   :14; //Time counter threshold for voltage sag detection
    uint32_t ZCR_SEL        :2;  //Selection bit for ZCR/CLK pin
    uint32_t ZCR_EN         :1;  //Enable ZCR or CLK 0-CLK, 1-ZCR
    uint32_t TMP_TOL        :2;  //Selection bits for tamper tolerance
    uint32_t TMP_EN         :1;  //Enable tampering feature
    uint32_t SW_RST         :1;  //SW reset brings the configuration registers to default
    uint32_t SW_LTCH1       :1;  //Primary channel measurement register latch
    uint32_t SW_LTCH2       :1;  //Secondary channel measurement register latch
    uint32_t SW_ALTCH       :1;  //Automatic measurement register latch at 7.8125 kHz
    uint32_t LED1_OFF       :1;  //LED1 pin output disable/enable 1-off, 0-on
    uint32_t LED2_OFF       :1;  //LED2 pin output disable/enable 1-off, 0-on
    uint32_t EN_CUM         :1;  //Cumulative energy calculation
    uint32_t REF_FREQ       :1;  //Reference line frequency 0-50Hz, 1-60Hz
    uint32_t                :4;  //Reserved
} dspctrl3_t;


/*

*/
typedef struct 
{
    uint32_t PHC2 :10; //Secondary current channel phase compensation register
    uint32_t PHV2 :2;  //Secondary voltage channel phase compensation register
    uint32_t PHC1 :10; //Primary current channel phase compensation register
    uint32_t PHV1 :2;  //Primary voltage channel phase compensation register
    uint32_t      :8;  //Reserved
} dspctrl4_t;

/*

*/
typedef struct 
{
    uint32_t CHV       :12; //Calibration register of primary voltage channel
    uint32_t SWV_THR   :10; //Swell threshold of primary voltage channel
    uint32_t SAG_THR   :10; //Sag threshold of primary voltage channel
} dspctrl5_t;
#define dspctrl7_t dspctrl5_t

/*


*/
typedef struct 
{
    uint32_t CHC     :12; //Calibration register of primary current channel
    uint32_t SWC_THR :10; //Swell threshold of primary current channel
    uint32_t         :10; //Reserved
} dspctrl6_t;
#define dspctrl8_t dspctrl6_t

/*

*/
typedef struct 
{
    uint32_t AH_UP  :12; //Primary channel RMS upper threshold (for AH)
    uint32_t OFA    :10; //Offset for primary channel active power
    uint32_t OFAF   :10; //Offset for primary channel fundamental active power
} dspctrl9_t;
#define dspctrl11_t dspctrl9_t

/*

*/
typedef struct 
{
    uint32_t AH_DOWN    :12; //Primary channel RMS lower threshold (for AH)
    uint32_t OFR        :10; //Offset for primary channel reactive power
    uint32_t OFS        :10; //Offset for primary channel apparent power
} dspctrl10_t;
#define dspctrl12_t dspctrl10_t


 /*
 DFE Control Register 1 and 2

 */
typedef struct 
{
    uint32_t ENV    :1;  //Enable for primary voltage channel
    uint32_t        :15; //Reserved
    uint32_t ENC    :1;  //Enable for primary current channel
    uint32_t        :9;  //Reserved
    uint32_t GAIN   :2;  //Gain selection of primary current channel: GAIN1[1:0]: 00-x2, 01-x4, 10-x8, 11-x16
    uint32_t        :4;  //Reserved
} dfectrl1_t;
#define dfectrl2_t dfectrl1_t


/*
DSP IRQ (Interrupt Control Mask) Register #1 / #2

*/
typedef struct 
{
    uint32_t PH1_PH2_IRQ_CR :4; //
    uint32_t PH2_IRQ_CR     :8; //
    uint32_t PH1_IRQ_CR     :8; //
    uint32_t C_IRQ_CR       :4; //
    uint32_t V_IRQ_CR       :8; //
} dspirq1_t; 
#define dspirq2_t dspirq1_t

/*
DSP Status Register 1 and 2
*/
typedef struct 
{
    uint32_t PH_PWR_SIGN_A      :1;
    uint32_t PH_PWR_SIGN_R      :1;
    uint32_t PH_NRG_OVF_A       :1;
    uint32_t PH_NRG_OVF_R       :1;
    uint32_t PH2_PWR_SIGN_A     :1;
    uint32_t PH2_PWR_SIGN_F     :1;
    uint32_t PH2_PWR_SIGN_R     :1;
    uint32_t PH2_PWR_SIGN_S     :1;
    uint32_t PH2_NRG_OVF_A      :1;
    uint32_t PH2_NRG_OVF_F      :1;
    uint32_t PH2_NRG_OVF_R      :1;
    uint32_t PH2_NRG_OVF_S      :1;
    uint32_t PH1_PWR_SIGN_A     :1;
    uint32_t PH1_PWR_SIGN_F     :1;
    uint32_t PH1_PWR_SIGN_R     :1;
    uint32_t PH1_PWR_SIGN_S     :1;
    uint32_t PH1_NRG_OVF_A      :1;
    uint32_t PH1_NRG_OVF_F      :1;
    uint32_t PH1_NRG_OVF_R      :1;
    uint32_t PH1_NRG_OVF_S      :1;
    uint32_t C_SIG_STUCK        :1;
    uint32_t C_NAH              :1;
    uint32_t C_SWELL_START      :1;
    uint32_t C_SWELL_END        :1;
    uint32_t V_SIG_STUCK        :1;
    uint32_t V_PER_ERR          :1;
    uint32_t V_SAG_START        :1;
    uint32_t V_SAG_END          :1;
    uint32_t V_SWELL_START      :1;
    uint32_t V_SWELL_END        :1;
    uint32_t PH_TAMPER          :1;
    uint32_t TAMPER_OR_WRONG    :1;
} dspsr1_t;  
#define dspsr2_t dspsr1_t  

typedef struct 
{
    uint32_t CRC_POLYNOM    :8; //SMBus default polynomial used: x8+x2+x+1)
    uint32_t NOISE_EN       :1; //UART noise immunity feature enabled
    uint32_t BREAK_ON_ERR   :1; //UART break feature enabled
    uint32_t                :4; //Reserved
    uint32_t CRC_EN         :1; //8-bit CRC enable (5 packet required in each transmission)
    uint32_t LSB_FIRST      :1; //0: big-endian, 1: little-endian
    uint32_t TIME_OUT       :8; //Time out (ms)
    uint32_t                :8; //Reserved
} uartspicr1_t; 

typedef struct 
{
    uint32_t BAUD_RATE   :16; //Defaulted to 9600 baud
    uint32_t FRAME_DELAY :8;
    uint32_t             :8;
} uartspicr2_t;  

/*
UART & SPI IRQ Register
*/
typedef struct 
{
    uint32_t                            :1; //
    uint32_t CTRL_UART_CRC_ERR          :1; //Activate IRQ on both INT1, INT2 for selected signals
    uint32_t CTRL_UART_TIME_OUT_ERR     :1; //Activate IRQ on both INT1, INT2 for selected signals
    uint32_t CTRL_UART_FRAME_ERR        :1; //Activate IRQ on both INT1, INT2 for selected signals
    uint32_t CTRL_UART_NOISE_ERR        :1; //Activate IRQ on both INT1, INT2 for selected signals
    uint32_t CTRL_UART_RX_OVR           :1; //Activate IRQ on both INT1, INT2 for selected signals
    uint32_t CTRL_UART_TX_OVR           :1; //Activate IRQ on both INT1, INT2 for selected signals
    uint32_t                            :1; //
    uint32_t CTRL_SPI_RX_FULL           :1; //Activate IRQ on both INT1, INT2 for selected signals
    uint32_t CTRL_SPI_TX_EMPT           :1; //Activate IRQ on both INT1, INT2 for selected signals
    uint32_t CTRL_UART_SPI_READ_ERR     :1; //Activate IRQ on both INT1, INT2 for selected signals
    uint32_t CTRL_UART_SPI_WRITE_ERR    :1; //Activate IRQ on both INT1, INT2 for selected signals
    uint32_t CTRL_SPI_CRC_ERR           :1; //Activate IRQ on both INT1, INT2 for selected signals
    uint32_t CTRL_SPI_UNDERRUN          :1; //Activate IRQ on both INT1, INT2 for selected signals
    uint32_t CTRL_SPI_OVERRUN           :1; //Activate IRQ on both INT1, INT2 for selected signals
    uint32_t                            :1; //Reserved
    uint32_t STAT_UART_BREAK            :1; //Break frame (all zeros) received
    uint32_t STAT_UART_CRC_ERR          :1; //CRC error detected
    uint32_t STAT_UART_TIME_OUT_ERR     :1; //Timeout counter expired
    uint32_t STAT_UART_FRAME_ERR        :1; //Missing stop bit detected
    uint32_t STAT_UART_NOISE_ERR        :1; //Noisy bit detected
    uint32_t STAT_UART_RX_OVR           :1; //Active when received data have not been correctly processed
    uint32_t STAT_UART_TX_OVR           :1; //Occurs when master and slave have different baud rates and master transmits before reception has ended
    uint32_t                            :1; //Reserved
    uint32_t STAT_SPI_RX_FULL           :1; //Reception buffer full (for SPI diagnostic, not recommended for normal IRQ operations)
    uint32_t STAT_SPI_TX_EMPT           :1; //Transmission buffer empty (for SPI diagnostic, not recommended for normal IRQ operations)
    uint32_t STAT_UART_SPI_READ_ERR     :1; //Read address out of range
    uint32_t STAT_UART_SPI_WRITE_ERR    :1; //Write address out of range
    uint32_t STAT_SPI_CRC_ERR           :1; //CRC error detected
    uint32_t STAT_SPI_UNDERRUN          :1; //Occurs when a read-back operation (= write then read the same register) or latch + read is too fast
    uint32_t STAT_SPI_OVERRUN           :1; //Occurs when two consecutive write transactions are too fast and close to each other
    uint32_t                            :1; //Reserved
} uartspisr3_t;   


/***************************************************************
 *  Read only register file
****************************************************************/

/*

*/
typedef struct 
{
    uint32_t PH_PWR_SIGN_A  :1; //Sign total active power
    uint32_t PH_PWR_SIGN_R  :1; //Sign total reactive power
    uint32_t PH_NRG_OVF_A   :1; //Overflow total active energy
    uint32_t PH_NRG_OVF_R   :1; //Overflow total reactive energy
    uint32_t PH_PWR_A       :1; //Sign primary channel active power
    uint32_t PH_PWR_F       :1; //Sign primary channel active fundamental power
    uint32_t PH_PWR_R       :1; //Sign primary channel reactive power
    uint32_t PH_PWR_S       :1; //Sign primary channel apparent power
    uint32_t PH_NRG_A       :1; //Overflow primary channel active energy
    uint32_t PH_NRG_F       :1; //Overflow primary channel active fundamental energy
    uint32_t PH_NRG_R       :1; //Overflow primary channel reactive energy
    uint32_t PH_NRG_S       :1; //Overflow primary channel apparent energy
    uint32_t C_ZCR          :1; //Primary current zero-crossing
    uint32_t C_SIG_STUCK    :1; //Primary current sigma-delta bitstream stuck
    uint32_t C_NAH          :1; //Primary current AH accumulation
    uint32_t SWC_EV         :4; //Primary current swell event history
    uint32_t V_ZCR          :1; //Primary voltage zero-crossing
    uint32_t V_SIG_STUCK    :1; //Primary voltage sigma-delta bitstream stuck
    uint32_t V_PER_ERR      :1; //Primary voltage period error (out of range)
    uint32_t SWV_EV         :4; //Primary voltage swell event history
    uint32_t SAG_EV         :4; //Primary voltage sag event history
    uint32_t                :2; //Reserved
}dspev1_t;
#define dspev2_t dspev1_t

/*

*/
typedef struct 
{
    uint32_t PH1_PERIOD :12; //
    uint32_t            :4;  //
    uint32_t PH2_PERIOD :12; //
    uint32_t            :4;  //
} dsp_reg1_t;

/*

*/
typedef struct 
{
    uint32_t V_RMS :15;
    uint32_t C_RMS :17;
} dsp_reg14_t;

#define dsp_reg15_t dsp_reg14_t;

/*

*/
typedef struct 
{
    uint32_t SWV_TIME :15;
    uint32_t          :1;
    uint32_t SAG_TIME :15;
    uint32_t          :1;
} dsp_reg16_t;
#define dsp_reg18_t dsp_reg16_t
/*

*/
typedef struct 
{
    uint32_t SWC_TIME :15;
    uint32_t          :1;
    uint32_t C_PHA    :12;
    uint32_t          :1;
} dsp_reg17_t;

#define dsp_reg19_t dsp_reg17_t;



typedef struct
{
/* RW reg */
    dspctrl1_t    DSPCTRL1;       // !< DSP Control Register 1	     
    dspctrl2_t    DSPCTRL2;       // !< DSP Control Register 2	     
    dspctrl3_t    DSPCTRL3;       // !< DSP Control Register 3	     
    dspctrl4_t    DSPCTRL4;       // !< DSP Control Register 4	     
    dspctrl5_t    DSPCTRL5;       // !< DSP Control Register 5	     
    dspctrl6_t    DSPCTRL6;       // !< DSP Control Register 6	     
    dspctrl7_t    DSPCTRL7;       // !< DSP Control Register 7	     
    dspctrl8_t    DSPCTRL8;       // !< DSP Control Register 8	     
    dspctrl9_t    DSPCTRL9;       // !< DSP Control Register 9	     
    dspctrl10_t   DSPCTRL10;      // !< DSP Control Register 10	   
    dspctrl11_t   DSPCTRL11;      // !< DSP Control Register 11	   
    dspctrl12_t   DSPCTRL12;      // !< DSP Control Register 12	   
    dfectrl1_t    DFECTRL1;       // !< DFE Control Register 1	     
    dfectrl2_t    DFECTRL2;       // !< DFE Control Register 2	     
    dspirq1_t     DSPIRQ1;        // !< DSP Interrupt Register 1	    
    dspirq2_t     DSPIRQ2;        // !< DSP Interrupt Register 2	    
    dspsr1_t      DSPSR1;         // !< DSP Status Register 1	      
    dspsr2_t      DSPSR2;         // !< DSP Status Register 2     
    uartspicr1_t  UARTSPICR1;     // !< UART/SPI Control Register 1 
    uartspicr2_t  UARTSPICR2;     // !< UART/SPI Control Register 2
    uartspicr3_t  UARTSPISR;      // !< UART/SPI Status Register	  
    dspev1_t      DSPEVENT1;      // !< DSP Events Register 1 
    dspev2_t      DSPEVENT2;      // !< DSP Events Register 2
    dsp_reg1_t    DSP_REG1;       // !< DSP Register 1              
    uint32_t      DSP_REG2;       // !< DSP Register 2              
    uint32_t      DSP_REG3;       // !< DSP Register 3              
    uint32_t      DSP_REG4;       // !< DSP Register 4              
    uint32_t      DSP_REG5;       // !< DSP Register 5              
    uint32_t      DSP_REG6;       // !< DSP Register 6              
    uint32_t      DSP_REG7;       // !< DSP Register 7              
    uint32_t      DSP_REG8;       // !< DSP Register 8              
    uint32_t      DSP_REG9;       // !< DSP Register 9              
    uint32_t      DSP_REG10;      // !< DSP Register 10             
    uint32_t      DSP_REG11;      // !< DSP Register 11             
    uint32_t      DSP_REG12;      // !< DSP Register 12             
    uint32_t      DSP_REG13;      // !< DSP Register 13             
    uint32_t      DSP_REG14;      // !< DSP Register 14             
    uint32_t      DSP_REG15;      // !< DSP Register 15             
    uint32_t      DSP_REG16;      // !< DSP Register 16             
    uint32_t      DSP_REG17;      // !< DSP Register 17             
    uint32_t      DSP_REG18;      // !< DSP Register 18             
    uint32_t      DSP_REG19;      // !< DSP Register 19             
    uint32_t      CH1_REG1;       // !< Channel 1 Register 1
    uint32_t      CH1_REG2;       // !< Channel 1 Register 2
    uint32_t      CH1_REG3;       // !< Channel 1 Register 3
    uint32_t      CH1_REG4;       // !< Channel 1 Register 4
    uint32_t      CH1_REG5;       // !< Channel 1 Register 5
    uint32_t      CH1_REG6;       // !< Channel 1 Register 6
    uint32_t      CH1_REG7;       // !< Channel 1 Register 7
    uint32_t      CH1_REG8;       // !< Channel 1 Register 8
    uint32_t      CH1_REG9;       // !< Channel 1 Register 9
    uint32_t      CH1_REG10;      // !< Channel 1 Register 10
    uint32_t      CH1_REG11;      // !< Channel 1 Register 11
    uint32_t      CH1_REG12;      // !< Channel 1 Register 12
    uint32_t      CH2_REG1;       // !< Channel 2 Register 1
    uint32_t      CH2_REG2;       // !< Channel 2 Register 2
    uint32_t      CH2_REG3;       // !< Channel 2 Register 3
    uint32_t      CH2_REG4;       // !< Channel 2 Register 4
    uint32_t      CH2_REG5;       // !< Channel 2 Register 5
    uint32_t      CH2_REG6;       // !< Channel 2 Register 6
    uint32_t      CH2_REG7;       // !< Channel 2 Register 7
    uint32_t      CH2_REG8;       // !< Channel 2 Register 8
    uint32_t      CH2_REG9;       // !< Channel 2 Register 9
    uint32_t      CH2_REG10;      // !< Channel 2 Register 10
    uint32_t      CH2_REG11;      // !< Channel 2 Register 11
    uint32_t      CH2_REG12;      // !< Channel 2 Register 12
    uint32_t      TOT_REG1;       // !< Total  Register 1
    uint32_t      TOT_REG2;       // !< Total  Register 2
    uint32_t      TOT_REG3;       // !< Total  Register 3
    uint32_t      TOT_REG4;       // !< Total  Register 4

} stpm_regs_t;




#ifdef __cplusplus
}
#endif

#endif