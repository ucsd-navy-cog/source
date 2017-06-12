/****************  COMMON SETTINGS  ****************/

//total number of channels in this device
//#define DEFAULT_ADS_COUNT   8
//#define DEFAULT_ADS_NCH     8
//#define DEFAULT_EXT_CHS     0
//#define DEFAULT_ACC_CHS     3

//device id modifier
//id 0 is raw mapping for debug
//id 1 is newflex
//id 2 is wetcap
//id 3 is childsize 16-CH DAQ64
//#define DEFAULT_DEVICE_ID_CONSTANT 1

//selective channel mode enable
//0 runs all 64 channels of the ADS
//1 is 32 channels selection from 64 channels
//2 is 16 channels selection from 64 channels
//#define DEFAULT_SELECTIVE_CHANNEL_MODE 0

//channel mode settings according to the ADS datasheet
//0 ***normal electrode input***
//1 input shorted
//2 used in conjunction with RLD_MEAS bit for RLD measurements
//3 MVDD for supply measurement
//4 temperature sensor
//5 ***test signal***
//6 RLD_DRP
//7 RLD_DRN
//#define DEFAULT_ADS_CHANNEL_MODE 0

//0 ***is default and uses internal ADS impedance check function***
//1 uses the proprietary external impedance check hardware
//#define DEFAULT_EXTERNAL_IMPEDANCE_CHECK 1

//0 ***disables flash storage***
//1 allows the use of flash storage
//#define USE_FLASH_STORAGE 0


/****************  RARE SETTINGS  ****************/

//channel gain settings according to the ADS datasheet
//0 gain of 6
//1 gain of 1
//2 gain of 2
//3 ***gain of 3***
//4 gain of 4
//5 gain of 8
//6 gain of 12
//#define DEFAULT_ADS_CHANNEL_GAIN 3

//ADS high resolution enable according to the ADS datasheet
//0 ***low-power mode***
//1 high-resolution mode
//#define DEFAULT_ADS_HIGH_RESOLUTION 0

//ADS sampling rate according to the ADS datasheet
//0 f_MOD/16    (19.2k SPS low-power, 38.4 SPS high-resolution)
//1 f_MOD/32    (9.6k SPS low-power, 19.2k SPS high-resolution)
//2 f_MOD/64    (4.8k SPS low-power, 9.6k SPS high-resolution)
//3 f_MOD/128   (2.4k SPS low-power, 4.8k SPS high-resolution)
//4 f_MOD/256   (1.2k SPS low-power, 2.4k SPS high-resolution)
//5 f_MOD/512   (600 SPS low-power, 1.2k SPS high-resolution)
//6 ***f_MOD/1024  (300 SPS low-power, 600 SPS high-resolution)***
//#define DEFAULT_ADS_SAMPLING_RATE 6


/****************  SYSTEM SETTINGS  ****************/

//#define ADS_CHS         ADS_COUNT*ADS_NCH
//#define ADS_CHS_TX      ADS_CHS

//total packet formula is 3 bytes for every data channel (EEG, EXT and ACC, plus 4 byte tail, plus 2 byte header)
//#define ADS_PACKET_LENGTH   ((int)( (ADS_CHS / (1+SELECTIVE_CHANNEL_MODE))*3 + EXT_CHS*3 + ACC_CHS*3 + 4 + 2))

//#define ADS_CHS_DEFAULT             (DEFAULT_ADS_COUNT*DEFAULT_ADS_NCH)
//#define ADS_CHS_DEFAULT_MINUS_1     (ADS_CHS_DEFAULT-1)

//memory buffers
//#define MEMBUFLEN                       (1024*80)
//#define BUFFERSIZE                      ((int)(MEMBUFLEN/2)) //size of each ping-pong buffer, half of the total memory buffer
//#define CIRCULAR_BUFFERS_SIZE           (256*8)
//#define CIRCULAR_BUFFERS_SIZE_MINUS_1   (CIRCULAR_BUFFERS_SIZE-1)

//array sizes
//#define ADS_READ_BUFFER_SIZE    256
//#define ADS_TX_BUF_SIZE         2310
//#define EXT_READ_BUFFER_SIZE    256
//#define ACC_BUFFER              256

//NRF module setting
//0 is normal operating condition, using the nRF
//1 is when nRF is not in use/detached
//mode 1 is briefly test and it worked during the short test
//perhaps a better way to disable nRF should be looked for in the future
//#define NRF_DETACHED 0

//nRF LED pulse time length
//must be at least 1
//#define nRF_LED_PULSE_PERIOD 32

//interaction settings
//#define AUX_BUTTON_HOLD_COUNT 6000000

//SYSTEM CONSTANTS
#if defined(CLOCK_SPEED_40MHZ)
#if defined(CLOCK_SPEED_3M_BAUD)
#define FOSC        36000000
#else
#define FOSC        40000000
#endif
#else
#define FOSC        80000000
#endif

#define SYSCLK      FOSC
#define PBCLK       SYSCLK
#if defined(CLOCK_SPEED_3M_BAUD)
#define BT_BAUD     3000000.0
#else
#define BT_BAUD     1000000.0
#endif

//#define PACKET_SIZE 4

//Power
//#define AN_PWR      LATDbits.LATD4
//#define VCC_PWR     LATEbits.LATE5
//#define BT_PWR      LATEbits.LATE7
//#define ACC_PWR     LATBbits.LATB2
//
//#define PWR_BTN     PORTDbits.RD9
//#define AUX_BTN     PORTBbits.RB15

//LED
//#define LED2    LATBbits.LATB13

#if defined(USE_OLD_PORT_PIN_LAYOUT)
#define LED1    LATBbits.LATB12
#else
#define LED1    LATDbits.LATD4
#endif

//#define LED_G   LATBbits.LATB11
//#define LED_B   LATBbits.LATB10
//#define LED_R   LATBbits.LATB9

//ADS1298 Ports and Config
//ADS1298
//#define ADS_PDN     LATFbits.LATF1
//#define ADS_START   LATDbits.LATD5
//#define ADS_RST     LATEbits.LATE0
//#define ADS_CS      LATEbits.LATE1
//#define ADS_SCK     LATDbits.LATD1 //RP15
//#define ADS_DOUT    PORTDbits.RD2 //RP14
//#define ADS_DIN     LATDbits.LATD3 //RP20

//external impedance pin
//#define EXTERNAL_IMPEDANCE_PIN  LATDbits.LATD6

//AUX box
//#define AUX_CS      LATEbits.LATE2

//nordic ports
//#define nRFCSN      LATDbits.LATD10
//#define nRFCE       LATDbits.LATD7
//#define nRFSCK      LATDbits.LATD11
//#define nRFMO       LATFbits.LATF3
//#define nRFMI       PORTGbits.RG2
//#define nRFIRQ      PORTDbits.RD8

////battery settings
//#define BATTERY_IS_DEAD     2
//#define BATTERY_IS_LOW      1
//#define BATTERY_IS_OK       0

#if defined(USE_OLD_PORT_PIN_LAYOUT)
#define SPISTDIO      SPI3STATbits.SPIRBF
#define SPIBUF        SPI3BUF
#else
#define SPISTDIO      SPI2STATbits.SPIRBF
#define SPIBUF        SPI2BUF
#endif

// pin assignments for NTF24L01P
#if defined(USE_OLD_PORT_PIN_LAYOUT)
#define nRF_CE        LATDbits.LATD5
#define nRF_IRQ       PORTDbits.RD10
#define nRF_CSN       LATDbits.LATD4
#else
#define nRF_CSN    LATEbits.LATE6
#define nRF_CE     LATEbits.LATE5
#define nRF_IRQ    PORTDbits.RD8
#endif

//nRF_test defs
#define LED2    LATFbits.LATF3
#define LED3    LATBbits.LATB15
#define LED4    LATBbits.LATB13
#define LED5    LATBbits.LATB12
#define LED6    LATBbits.LATB0
#define LED7    LATBbits.LATB1
#define LED_G   LATBbits.LATB8
#define LED_B   LATBbits.LATB9
#define LED_R   LATBbits.LATB10

//#define nRF2_CSN    LATDbits.LATD4
//#define nRF2_CE     LATDbits.LATD5
////#define nRF2_CSN    LATGbits.LATG3
////#define nRF2_CE     LATGbits.LATG2
//#define nRF2_IRQ    PORTDbits.RD10

#define nRF3_CSN    LATEbits.LATE6
#define nRF3_CE     LATEbits.LATE7
#define nRF3_IRQ    PORTDbits.RD8
//
#define nRF4_CSN    LATBbits.LATB3
#define nRF4_CE     LATBbits.LATB2
#define nRF4_IRQ    PORTDbits.RD9

////2 3 4
//#define NRF_MODULE  2
////0 is receiver, 1 is transmitter
//#define NRF_MODE    1

//#define MAIN_TIMER  ((unsigned int)(SYSCLK/1000))        //exactly 2 ms