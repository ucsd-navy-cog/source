// originally factored by C. Dabinett

#include "sysio.h"
#include "stdinc.h"

#include "peripheral/timer.h"

// PIC32MX795F512H Configuration Bit Settings
// DEVCFG3
// USERID = No Setting
#pragma config FSRSSEL = PRIORITY_7     // SRS Select (SRS Priority 7)
#pragma config FMIIEN = ON              // Ethernet RMII/MII Enable (MII Enabled)
#pragma config FETHIO = ON              // Ethernet I/O Pin Select (Default Ethernet I/O)
#pragma config FCANIO = ON              // CAN I/O Pin Select (Default CAN I/O)
#pragma config FUSBIDIO = OFF           // USB USID Selection (Controlled by Port Function)
#pragma config FVBUSONIO = OFF          // USB VBUS ON Selection (Controlled by Port Function)

// DEVCFG2
#if defined(CLOCK_SPEED_3M_BAUD)
#pragma config FPLLIDIV = DIV_2        // PLL Input Divider (12x Divider)
#pragma config FPLLMUL = MUL_18         // PLL Multiplier (24x Multiplier)
#else
#pragma config FPLLIDIV = DIV_1        // PLL Input Divider (12x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (24x Multiplier)
#endif
#pragma config UPLLIDIV = DIV_12        // USB PLL Input Divider (12x Divider)
#pragma config UPLLEN = OFF             // USB PLL Enable (Disabled and Bypassed)
#if defined(CLOCK_SPEED_40MHZ)
#pragma config FPLLODIV = DIV_2       // System PLL Output Clock Divider (PLL Divide by 256)
#else
#pragma config FPLLODIV = DIV_1       // System PLL Output Clock Divider (PLL Divide by 256)
#endif

// DEVCFG1
#pragma config FNOSC = FRCPLL              // Oscillator Selection Bits (Primary Osc (XT,HS,EC))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = HS             // Primary Oscillator Configuration (HS osc mode)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_1           // WATCH: Could set this to DIV_2
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))

// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (ICE EMUC2/EMUD2 pins shared with PGC2/PGD2)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)


void _mon_putc (char c) // enable stdout (write, printf, etc)
{
#if defined(USE_OLD_PORT_PIN_LAYOUT)
  while (U3STAbits.UTXBF != 0); // Wait till transmission is complete
  U3TXREG = (unsigned int)(unsigned char)c;
#else
  while (U2STAbits.UTXBF != 0); // Wait till transmission is complete
  U2TXREG = (unsigned int)(unsigned char)c;
#endif
}


void sys_init_pic32mx_mcu_and_ports()
{

  //============================================
  // Configure the PIC32MX MCU with all available options (PBBus, Cache, 0 wait states on RAM)
  //============================================
  //SYSTEMConfig(SYSCLK,SYS_CFG_ALL);
  SYSTEMConfigPerformance(SYSCLK);

  //============================================
  // DISABLE JTAG
  //============================================
  DDPCONbits.JTAGEN = 0;
  U1PWRCbits.USBPWR = 0;
  U1CONbits.HOSTEN = 0;
  U1CONbits.USBEN = 0;
  U1OTGCONbits.OTGEN = 0;
  U1OTGCONbits.DPPULUP = 1;
  U1OTGCONbits.DMPULUP = 1;

  //============================================
  // WAIT FOR POWER UP
  //============================================
  // uint i;
  // for(i = 0; i < (SYSCLK/64); i++);
  DELAY_MS(50); // safer

  //============================================
  // Configure Basic PINS and Ports
  //============================================
  TRISB = 0xFFFF;
  TRISC = 0xFFFF;
  TRISD = 0xFFFF;
  TRISE = 0xFFFF;
  TRISF = 0xFFFF;
  TRISG = 0xFFFF;
  //--
  AD1PCFG = 0xFFFF; // all pins digital
  //--
  TRISBbits.TRISB0 = 0; // LED Array
  TRISBbits.TRISB1 = 0;
  TRISBbits.TRISB8 = 0;
  TRISBbits.TRISB9 = 0;
  TRISBbits.TRISB10 = 0;
  TRISBbits.TRISB12 = 0;
  TRISBbits.TRISB13 = 0;
  TRISBbits.TRISB15 = 0;
  TRISFbits.TRISF3 = 0;
  LED2 = 0;
  LED3 = 0;
  LED4 = 0;
  LED5 = 0;
  LED6 = 0;
  LED7 = 0;
  LED_G = 1;
  LED_B = 1;
  LED_R = 1;

  //============================================
  // Configure Blue Tooth Ports etc
  //============================================

#if defined(USE_OLD_PORT_PIN_LAYOUT)
  
    //U2RXA at RG7
  mPORTGSetPinsDigitalIn(BIT_7);
  //U2TXA at RG8
  mPORTGSetPinsDigitalOut(BIT_8);

  //U2CTSA at RG9 as input
  //mPORTGSetPinsDigitalIn(BIT_9);
  //U2RTSA at RG6 as output
  //mPORTGSetPinsDigitalOut(BIT_6);
#else
    //U2RXA at RF4
  mPORTFSetPinsDigitalIn(BIT_4);
  //U2TXA at RF5
  mPORTFSetPinsDigitalOut(BIT_5);

//    //U2CTSA at RB8 as input
//    mPORTBSetPinsDigitalIn(BIT_8);
//    //U2RTSA at RB14 as output
//    mPORTBSetPinsDigitalOut(BIT_14);  
  
#endif
  
#if defined(USE_OLD_PORT_PIN_LAYOUT)
  //configure for operational mode
  UARTConfigure(UART3, (UART_ENABLE_HIGH_SPEED));
  //UARTConfigure(UART3, (UART_ENABLE_PINS_CTS_RTS | UART_ENABLE_HIGH_SPEED));
  //UARTSetFifoMode(UART2A, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
  UARTSetLineControl(UART3, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
  UARTSetDataRate(UART3, PBCLK, BT_BAUD);
  UARTEnable(UART3, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

  // ADDED - RX3 Interrupt - Handler is below
  INTEnable(INT_SOURCE_UART_RX(UART3), INT_ENABLED);
  INTSetVectorPriority(INT_VECTOR_UART(UART3), INT_PRIORITY_LEVEL_2);
  INTSetVectorSubPriority(INT_VECTOR_UART(UART3), INT_SUB_PRIORITY_LEVEL_0);
#else
  //configure for operational mode
  UARTConfigure(UART2, (UART_ENABLE_HIGH_SPEED));
  //UARTConfigure(UART2, (UART_ENABLE_PINS_CTS_RTS | UART_ENABLE_HIGH_SPEED));
  //UARTSetFifoMode(UART2A, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
  UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
  UARTSetDataRate(UART2, PBCLK, BT_BAUD);
  UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

  // ADDED - RX2 Interrupt - Handler is below
  INTEnable(INT_SOURCE_UART_RX(UART2), INT_ENABLED);
  INTSetVectorPriority(INT_VECTOR_UART(UART2), INT_PRIORITY_LEVEL_2);
  INTSetVectorSubPriority(INT_VECTOR_UART(UART2), INT_SUB_PRIORITY_LEVEL_0);  
#endif

  //============================================
  // Finished
  //============================================

  DELAY_MS(50);
}

void __delay_us(uint32 delay_in_microsecs)
{
  const uint32 clkfact = (uint32)(SYSCLK / 2000000.0);
  uint32 t = clkfact * (delay_in_microsecs ? delay_in_microsecs : 1);
#if 0 // thrashes profiling.. but more efficient
  WriteCoreTimer(0);
  while (ReadCoreTimer() < t);
#else // profiling savvy - overrun protection
  uint32 c = ReadCoreTimer();
  uint32 n = c + t; 
  if (n < c)
    while (ReadCoreTimer() > c); // run to max limit (until 0)
  while (ReadCoreTimer() < n);
#endif
}

static void null_callback(uint32 p)
{  
}

static TCallback set_tcb = null_callback;

void sys_set_timer1_callback( TCallback cb )
{
  if (cb)
    set_tcb = cb;
}

void __ISR(_TIMER_1_VECTOR, ipl2) _T1Interrupt()
{
  set_tcb(0);

  INTClearFlag(INT_T1);
}

void sys_enable_timer1( double interval_us )
{
#if defined(CLOCK_SPEED_40MHZ)
  if (interval_us < 1000.0)
    interval_us = 1000;
  else if (interval_us > 400000.0) // max 62500 -> PR1 for 16 bit timer @ 40mhz / 36 mhz
    interval_us = 400000;
#else
  if (interval_us < 1000.0)
    interval_us = 1000;
  else if (interval_us > 200000.0)
    interval_us = 200000;
#endif
  //--
  CloseTimer1();
  //--
  //uint32 tp = (uint32)(((double)PBCLK / 256.0) * (interval_us / 1000000.0));
  //OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_256, tp);

  uint32 tp = (uint32)(((double)PBCLK / 8.0) * (interval_us / 1000000.0)) + TIMER1_PR_TWEAK_OFFSET;
  OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_8, tp); 

  INTSetVectorPriority(INT_TIMER_1_VECTOR, INT_PRIORITY_LEVEL_2);
  ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);

#if !defined(ENABLE_UPLOAD_DATA)
  printf("timer period: %u\n",tp); // TODO: Remove on release
#endif
}

void sys_enable_mv_ints()
{
  INTEnableSystemMultiVectoredInt();
  // same as  asm("EI"); ??
}

void sys_begin_profile( uint32 * ts )
{
  if (ts)
    *ts = ReadCoreTimer();
}

double sys_query_profile( uint32 ts )
{
  const uint32 clkfact = (uint32)(SYSCLK / 2000000.0);
  uint32 n = ReadCoreTimer();
  if (n > ts)
    return ((double)(n-ts))/(double)clkfact;
  else 
    return ((double)((~ts)+n+1))/(double)clkfact;
}

static bool  rx_uart_has_data = false;
static uint8 rx_uart_byte = 0x00;

#if defined(USE_OLD_PORT_PIN_LAYOUT)
void __ISR(_UART_3_VECTOR, ipl2) UART3HANDLER(void)
{
  if (INTGetFlag(INT_U3RX)) { // OR INT_U1RX???
    if( (U3STAbits.OERR == 1) || (U3STAbits.PERR == 1) || (U3STAbits.FERR == 1) ){
      U3STAbits.OERR = 0; // clears if the Receive buffer has overflowed
      U3STAbits.PERR = 0; // parity error
      U3STAbits.FERR = 0; // framing error
      U3RXREG;
    }
    else {
      rx_uart_has_data = true;
      rx_uart_byte = (uint8)U3RXREG;
    }
    INTClearFlag(INT_SOURCE_UART_RX(UART3));
  }
}
#else
void __ISR(_UART_2_VECTOR, ipl2) UART2HANDLER(void)
{
  if (INTGetFlag(INT_U2RX)) {
    if( (U2STAbits.OERR == 1) || (U2STAbits.PERR == 1) || (U2STAbits.FERR == 1) ){
      U2STAbits.OERR = 0; // clears if the Receive buffer has overflowed
      U2STAbits.PERR = 0; // parity error
      U2STAbits.FERR = 0; // framing error
      U2RXREG;
    }
    else {
      rx_uart_has_data = true;
      rx_uart_byte = (uint8)U2RXREG;
    }
    INTClearFlag(INT_SOURCE_UART_RX(UART2));
  }
}
#endif

bool sys_rx_uart_has_data()
{
  return rx_uart_has_data;
}

uint8 sys_get_rx_uart_byte()
{
  uint8 r = rx_uart_byte;
  rx_uart_byte = 0x00;
  rx_uart_has_data = false;
  return r;
}
