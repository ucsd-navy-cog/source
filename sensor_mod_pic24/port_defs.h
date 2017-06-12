
#if defined(ENABLE_4_MHZ)
#define FREQ_MULT (2)  // if FOSC is 4 mhz
#define FOSC 4000000.0
#else
#define FREQ_MULT (4)  // if FOSC is 8 mhz
#define FOSC 8000000.0
#endif

#define FCY (FOSC/2.0)      // JL: really should be FPB, since FCY is instructions frequency which is main clock divided by 4 (a.k.a. FOSC/4)

#define LED            LED1
#define LED1           LATAbits.LATA4
#define AN_PWR_DISABLE LATBbits.LATB14

#define CURRENT_CTR    LATBbits.LATB8
//#define FT_BAUD       (57600.0)
//#define FT_BAUD       (115200.0)
//#define FT_BAUD       (1000000.0)

#define SPISTDIO      SPI2STATbits.SPIRBF
#define SPIBUF        SPI2BUF

// pin assignments for NTF24L01P
#define nRF_CE        LATAbits.LATA3
#define nRF_IRQ       PORTBbits.RB2
#define nRF_CSN       LATAbits.LATA2

#define IMU_CS      LATBbits.LATB4
//#define IMU_DRDY    PORTBbits.RB7

#define ADC_CNV     LATBbits.LATB11

