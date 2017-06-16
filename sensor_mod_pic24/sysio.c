
// originally factored by C. Dabinett

#include "sysio.h"
//#include "stdinc.h"

//=====================================================================================================================================

// refer: C:\Program Files (x86)\Microchip\xc16\v1.26\docs
//        C:\Program Files (x86)\Microchip\xc16\v1.26\docs\config_index.html
//        C:/Program Files (x86)/Microchip/xc16/v1.26/docs/config_docs/24FJ64GA102.html

// CONFIG4
#pragma config DSWDTPS = DSWDTPSF       // DSWDT Postscale Select (1:2,147,483,648 (25.7 days))
#pragma config DSWDTOSC = LPRC          // Deep Sleep Watchdog Timer Oscillator Select (DSWDT uses Low Power RC Oscillator (LPRC))
#pragma config RTCOSC = SOSC            // RTCC Reference Oscillator  Select (RTCC uses Secondary Oscillator (SOSC))
#pragma config DSBOREN = OFF             // Deep Sleep BOR Enable bit (BOR enabled in Deep Sleep)
#pragma config DSWDTEN = OFF            // Deep Sleep Watchdog Timer (DSWDT disabled)

// CONFIG3
#pragma config WPFP = WPFP63            // Write Protection Flash Page Segment Boundary (Highest Page (same as page 42))
#pragma config SOSCSEL = IO             // Secondary Oscillator Pin Mode Select (SOSC pins have digital I/O functions (RA4, RB4))
#pragma config WUTSEL = LEG             // Voltage Regulator Wake-up Time Select (Default regulator start-up time used)
#pragma config WPDIS = WPDIS            // Segment Write Protection Disable (Segmented code protection disabled)
#pragma config WPCFG = WPCFGDIS         // Write Protect Configuration Page Select (Last page and Flash Configuration words are unprotected)
#pragma config WPEND = WPENDMEM         // Segment Write Protection End Page Select (Write Protect from WPFP to the last page of memory)

// CONFIG2
#pragma config POSCMOD = NONE           // Primary Oscillator Select (Primary Oscillator disabled)
#pragma config I2C1SEL = SEC            // I2C1 Pin Select bit (Use alternate SCL1/SDA1 pins for I2C1)
#pragma config IOL1WAY = ON             // IOLOCK One-Way Set Enable (Once set, the IOLOCK bit cannot be cleared)
#pragma config OSCIOFNC = ON            // OSCO Pin Configuration (OSCO pin functions as port I/O (RA3))
#pragma config FCKSM = CSDCMD           // Clock Switching and Fail-Safe Clock Monitor (Sw Disabled, Mon Disabled)

//#if defined(BUILD_FOR_SENSOR_MODULE)
//#pragma config FNOSC = FRC              // Initial Oscillator Select (Fast RC Oscillator)
////#pragma config FNOSC = FRCPLL
//#else
////#pragma config FNOSC = FRCPLL              // Initial Oscillator Select (Fast RC Oscillator)
//#pragma config FNOSC = FRC              // Initial Oscillator Select (Fast RC Oscillator)
//#endif

#pragma config FNOSC = FRCDIV              // Initial Oscillator Select (Fast RC Oscillator)

#pragma config IESO = ON                // Internal External Switchover (IESO mode (Two-Speed Start-up) enabled)

// CONFIG1

// for watchdog
//#pragma config WDTPS = PS1              // Watchdog Timer Postscaler (1:32,768)
//#pragma config FWPSA = PR32             // WDT Prescaler (Prescaler ratio of 1:128)
//// 1/32768 * 32 = 0.0009765625 secs resolution
// use _SWDTEN = 1; // to turn watchdog on (0 = off)

#pragma config WDTPS = PS32768          // Watchdog Timer Postscaler (1:32,768)
#pragma config FWPSA = PR128            // WDT Prescaler (Prescaler ratio of 1:128)
#pragma config WINDIS = OFF             // Windowed WDT (Standard Watchdog Timer enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF             // Watchdog Timer (Watchdog Timer is disabled)
#pragma config ICS = PGx1               // Emulator Pin Placement Select bits (Emulator functions are shared with PGEC1/PGED1)
#pragma config GWRP = OFF               // General Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF                // General Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG port is disabled)

//=====================================================================================================================================

static volatile int system_power_on = 0;
static unsigned current_ctr_state = 0; // tracks CURRENT_CTR

static void WriteIMUReg(int addr, int dat);

void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt(void)
{
  if(system_power_on == 1)
  {
    WriteIMUReg(0x6B, 0x81); // TURN IMU OFF
    IMU_CS = 1;

    asm("RESET");
  }

  IFS1bits.INT1IF = 0;
}

void sys_handle_power_button()
{
  AD1PCFG = 0xFFFF;//AD1CON1 = 0x00; //digital
  TRISA = 0xFFFF;
  TRISB = 0xFFFF;

  TRISBbits.TRISB15 = 1;
  RPINR0bits.INT1R = 15;
  INTCON2bits.INT1EP = 1; //falling edge interrupt
  IPC5bits.INT1IP = 6;    //interrupt priority 6
  IFS1bits.INT1IF = 0;
  IEC1bits.INT1IE = 1;

  TRISAbits.TRISA4 = 0;
 
  TRISBbits.TRISB14 = 0; // Analog power enable
  AN_PWR_DISABLE = 1;
  
  Sleep();
  system_power_on = 1;
}

static void WriteIMUReg(int addr, int dat)
{
  uint imu_dat;
#if defined(ENABLE_EXT_ADC)
  bool adc_en = are_ext_adc_interrupts_enabled();
#endif

  //switchSPIModeIMU();
  IEC1bits.INT1IE = 0;
#if defined(ENABLE_EXT_ADC)  
  if (adc_en)
    IEC1bits.T4IE = 0;      // enable timer4 interrupts
#endif
  IMU_CS = 0;
  //    __delay32(40);

  addr = addr | 0x00;

  SPI1BUF = addr;
  while(!SPI1STATbits.SPIRBF);
  imu_dat = SPI1BUF;

  SPI1BUF = dat;
  while(!SPI1STATbits.SPIRBF);
  imu_dat = SPI1BUF;

  IMU_CS = 1;
  IEC1bits.INT1IE = 1;
#if defined(ENABLE_EXT_ADC)  
  if (adc_en)
    IEC1bits.T4IE = 1;      // enable timer4 interrupts
#endif
  //    __delay32(40); 
  //switchSPIModeADC();
}

static uint8 imu_adc_control_state = ACTIVE_IMU_ACC | ACTIVE_ADC_PPG;

#if defined(ENABLE_IMU)

static uint ReadIMUReg(int addr)
{
  uint imu_dat;
#if defined(ENABLE_EXT_ADC)
  bool adc_en = are_ext_adc_interrupts_enabled();
#endif
  
  //switchSPIModeIMU();
  IEC1bits.INT1IE = 0;
#if defined(ENABLE_EXT_ADC)
  if (adc_en)
    IEC1bits.T4IE = 0;      // enable timer4 interrupts
#endif

  
  IMU_CS = 0;
  //    __delay32(40);

  addr = addr | 0x80;

  SPI1BUF = addr;
  while(!SPI1STATbits.SPIRBF);
  imu_dat = SPI1BUF;

  SPI1BUF = 0;
  while(!SPI1STATbits.SPIRBF);
  imu_dat = SPI1BUF;

  IMU_CS = 1;
  IEC1bits.INT1IE = 1;
#if defined(ENABLE_EXT_ADC)  
  if (adc_en)
    IEC1bits.T4IE = 1;      // enable timer4 interrupts
#endif
  //    __delay32(40);
  //switchSPIModeADC();

  return imu_dat;
}

static void IMUPWRON()
{
  WriteIMUReg(0x6A, 0x00);
  WriteIMUReg(0x6B, 0x01);//WriteIMUReg(0x6B, 0x00);
  //  WriteIMUReg(0x6C, 0x00);
}

static void IMUACCEnable()
{
  uint imu_dat = ReadIMUReg(0x6C);
  imu_dat &= (0xC7);

  WriteIMUReg(0x6C, imu_dat);
}

static void IMUACCDisable()
{
  uint imu_dat = ReadIMUReg(0x6C);
  imu_dat |= (0x38);

  WriteIMUReg(0x6C, imu_dat);
}

static void IMUGyroEnable()
{
  uint imu_dat = ReadIMUReg(0x6C);
  imu_dat &= (0xF8);

  WriteIMUReg(0x6C, imu_dat);
}

static void IMUGyroDisable()
{
  uint imu_dat = ReadIMUReg(0x6C);
  imu_dat |= (0x07);

  WriteIMUReg(0x6C, imu_dat);
}

//static void IMUBrdTempEnable()
//{
//  uint imu_dat = ReadIMUReg(0x6B);
//  imu_dat &= (0xF7);
//
//  WriteIMUReg(0x6B, imu_dat);
//}
//
//static void IMUBrdTempDisable()
//{
//  uint imu_dat = ReadIMUReg(0x6B);
//  imu_dat |= (0x08);
//
//  WriteIMUReg(0x6B, imu_dat);
//}

//static void IMUACCLOWPWREnable()
//{
//  uint imu_dat = ReadIMUReg(0x6B);
//  imu_dat |= (0x20);
//
//  WriteIMUReg(0x6B, imu_dat);
//}
//
//static void IMUACCLOWPWRDisable()
//{
//  uint imu_dat = ReadIMUReg(0x6B);
//  imu_dat &= (0xDF);
//
//  WriteIMUReg(0x6B, imu_dat);
//}

//static void IMUGyroLOWPWREnable()
//{
//  uint imu_dat = ReadIMUReg(0x1E);
//  imu_dat |= (0x80);
//
//  WriteIMUReg(0x1E, imu_dat);
//}
//
//static void IMUGyroLOWPWRDisable()
//{
//  uint imu_dat = ReadIMUReg(0x1E);
//  imu_dat &= (0x7F);
//
//  WriteIMUReg(0x1E, imu_dat);
//}
//


static void InitIMU()
{

  DELAY_MS(100);

  WriteIMUReg(0x70, 0x40); // what does this do ???

  if(ReadIMUReg(0x75) != 0x12)
  {
    LED = 0;
    //        while(1);
  }
  else
  {
    IMUPWRON();
    //--
    //IMUACCLOWPWREnable();
    //IMUGyroLOWPWREnable();
    //--
    if (imu_adc_control_state & ACTIVE_IMU_ACC)
      IMUACCEnable();
    else
      IMUACCDisable(); // power savings fix
    //--
    if (imu_adc_control_state & ACTIVE_IMU_GYRO)
      IMUGyroEnable();
    else
      IMUGyroDisable(); // power savings fix

    //IMUACCLOWPWRDisable();
  }

}

#else

static void IMUPWROFF()
{
  WriteIMUReg(0x6B, 0x81);
  IMU_CS = 1;
}

#endif

static void InitPorts() // general for either ext ADC and/or IMU (default)
{
  TRISBbits.TRISB11 = 0;
  ADC_CNV = 0;

  //initalize SPI Pins
  TRISBbits.TRISB9 = 0;

  //init port directions
  TRISBbits.TRISB4 = 0;   //IMU CS
  IMU_CS = 1;

  TRISBbits.TRISB6 = 0;   //SDO or IMU input
  RPOR3bits.RP6R = 7;     //SDO or IMU input

  TRISBbits.TRISB10 = 1;

  RPINR20bits.SDI1R = 10; //ADC data out on RB10 - set to some default 

  //ADC_SCK
  RPOR4bits.RP9R = 8;

  //initialize SPI port to correct clock polarity for AD7685, maximum clock rate, 16 bit mode
  SPI1CON1bits.DISSCK = 0;
  SPI1CON1bits.DISSDO = 0;
  SPI1CON1bits.MODE16 = 0;
  SPI1CON1bits.SMP = 1;
  SPI1CON1bits.CKE = 0;
  SPI1CON1bits.CKP = 1;
  SPI1CON1bits.SSEN = 0;
  SPI1CON1bits.MSTEN = 1;
  SPI1CON1bits.SPRE = 7; //1:1
  SPI1CON1bits.PPRE = 3; //1:1

  SPI1CON2 = 0;

  // for ext ADC - but commented out...
  //    IPC2bits.SPI1IP = 2;
  //    IFS0bits.SPI1IF = 0;
  //    IEC0bits.SPI1IE = 1;

  //SPI Enabled
  SPI1STATbits.SPIEN = 1;
}

void sys_init_pic24_ports()
{
#if defined(ENABLE_4_MHZ)
    CLKDIVbits.RCDIV = 1; // 4 mhz
#else
    CLKDIVbits.RCDIV = 0; // 8 mhz
#endif

    //reset port directions
    AD1PCFG = 0xFFFF;
    TRISA = 0xFFFF;
    TRISB = 0xFFFF;

    TRISAbits.TRISA4 = 0; // set led1 pin dir
    LED = 0;   

    TRISBbits.TRISB14 = 0; // Analog power enable
    AN_PWR_DISABLE = 0;
     
    // borrowed
    TRISBbits.TRISB8 = 0;
    CURRENT_CTR = current_ctr_state;

#if 0  // not appropriate for sensors according to Jun
//    //configure UART pins for debug output
//    //UART TX
//    RPOR6bits.RP13R = 3;
//    //UART RX
//    RPINR18bits.U1RXR = 12;
//    //UARTCTS
//    RPINR18bits.U1CTSR = 5;
//    //BT CTS
//    TRISBbits.TRISB14 = 0;
//    LATBbits.LATB14 = 0;
//    
//    //enable UART for full flow control
//    U1MODE = 0x8000;
//    U1MODEbits.IREN = 0;
//    U1MODEbits.RTSMD = 0;
//    U1MODEbits.PDSEL = 0;
//    U1MODEbits.STSEL = 0;
//    U1MODEbits.BRGH = 1;
//
//    U1STAbits.UTXEN = 1;
//    U1STAbits.UTXINV = 0;
//
//    U1MODEbits.UEN = 0b00; //no flow
//
//    double brg_temp = FCY / (4.0 * FT_BAUD) - 1.0;
//    U1BRG = (int)(brg_temp + 0.5);
#endif
    
    InitPorts();

#if defined(ENABLE_IMU)
    InitIMU();
#else
    IMUPWROFF();
#endif

    DELAY_MS(2);
   
}

void __attribute__((__interrupt__, __shadow__, __no_auto_psv__)) _T1Interrupt(void) // do not remove
{
    IFS0bits.T1IF = 0;     //  clear timer1 interrupt status flag
}

void sys_init_timer1()
{
  T1CON = 0x00;           // stops timer1 and reset its control register - set all T1CONbits to 0
  TMR1 = 0x00;            // clear contents of timer1 register (__counter__)

  PR1 = 40000; // some moderate high value

  T1CONbits.TCS = 0;      // 0 = internal clock (FOSC/2), 1 = external clock from TxCK pin
  // T1CONbits.TSYNC      // only applicable if using an external clock (TCS == 1)
  // T1CONbits.TGATE = 0; // gated time accumulation enable bit
  T1CONbits.TSIDL = 0;    // whether timer interrupts DO NOT continue to fire when pic is idle (e.g. Idle()). 0 = continue to fire interrupt during Idle().

  IPC0bits.T1IP = 0x07;   // set timer1 priority to level 7 (highest) - setting from 1 to 7, seems to make an unmeasurable positive difference.
  IFS0bits.T1IF = 0;      // clear timer1 interrupt status flag

  IEC0bits.T1IE = 1;      // enable timer1 interrupts

  T1CONbits.TON = 0;      // start timer1 - this is toggled for idling in sensor (power savings)
}

void sys_init_timer32()
{
  T2CON = 0x00;
  T3CON = 0x00;
  TMR3 = 0x00; 
  TMR2 = 0x00;
  PR3 = 0xFFFF; //Load the Period register3 with the value 0xFFFF
  PR2 = 0xFFFF; //Load the Period register2 with the value 0xFFFF
  IPC1bits.T2IP = 0x07;
  IPC2bits.T3IP = 0x07;
  T2CONbits.T32 = 1;

//#if !defined(TIMER_4_INTERRUPTS)
  //T4CON = 0x00;
  //T5CON = 0x00;
  //TMR5 = 0x00; 
  //TMR4 = 0x00;
  //PR5 = 0xFFFF; //Load the Period register3 with the value 0xFFFF
  //PR4 = 0xFFFF; //Load the Period register2 with the value 0xFFFF
  //T4CONbits.T32 = 1;
//#endif

}

void sys_start_timer32()
{
  TMR3HLD = 0x00;
  TMR2 = 0x00;
  T2CONbits.TON = 1;
}

uint32 sys_query_timer32()
{
  uint32 r;
  uint16 lsw, msw;
  lsw = TMR2;
  msw = TMR3HLD;
  r = (((uint32)msw) << 16) + (uint32)lsw;
  return (r/(uint32)FREQ_MULT);
}

uint32 sys_stop_timer32()
{
  uint32 r;
  uint16 lsw, msw;
  T2CONbits.TON = 0;
  lsw = TMR2;
  msw = TMR3HLD;
  r = (((uint32)msw) << 16) + (uint32)lsw;
  return (r/(uint32)FREQ_MULT);
}

void sys_cancel_timer32()
{
  T2CONbits.TON = 0;
}

//void sys_restart_timer32()
//{
//  TMR3HLD = 0x00;
//  TMR2 = 0x00;
//}


//#if !defined(TIMER_4_INTERRUPTS)
//void sys_start_timer32_2()
//{
//  TMR5HLD = 0x00;
//  TMR4 = 0x00;
//  T4CONbits.TON = 1;  
//}
//
//uint32 sys_query_timer32_2()
//{
//  uint32 r;
//  uint16 lsw, msw;
//  lsw = TMR4;
//  msw = TMR5HLD;
//  r = (((uint32)msw) << 16) + (uint32)lsw;
//  return (r/4L); // 4 assumes 8 mhz freq
//}
//
//uint32 sys_stop_timer32_2()
//{
//  uint32 r;
//  uint16 lsw, msw;
//  T4CONbits.TON = 0;
//  lsw = TMR4;
//  msw = TMR5HLD;
//  r = (((uint32)msw) << 16) + (uint32)lsw;
//  return (r/4L); // 4 assumes 8 mhz freq
//}

#if defined(ENABLE_EXT_ADC)
//static void set_timer4_interval(uint8 cs)
//{
//  double fp;
//  TMR4 = 0x00;
//  if (cs == ACTIVE_ADC_PPG) // PPG only?
//    fp = 1000000.0 / (double)BASE_TIMER4_INT_FREQ_HZ_LOW * 4.0; // freq period
//  else // else 12k
//    fp = 1000000.0 / (double)BASE_TIMER4_INT_FREQ_HZ * 4.0; // freq period
//  PR4 = (uint)fp; // this only handles up to 64k, probably why 32000 interrupts didn't work. would need a 32 bit timer.
//}

//void  enable_ext_adc_interrupts();  // re-enables ext adc interrupting
//void  disable_ext_adc_interrupts(); // disables ext adc interrupting

static uint base_pr4 = 1;

void sys_init_timer_4_inter()
{
  T4CON = 0x00;           // stops timer4 and reset its control register - set all T4CONbits to 0
  TMR4 = 0x00;            // clear contents of timer4 register (__counter__)

  PR4 = 40000; // arbitrary, its off

  //T4CONbits.TCS = 0;      // 0 = internal clock (FOSC/2), 1 = external clock from TxCK pin
  //// T1CONbits.TSYNC      // only applicable if using an external clock (TCS == 1)
  //// T1CONbits.TGATE = 0; // gated time accumulation enable bit
  //T4CONbits.TSIDL = 0;    // whether timer interrupts DO NOT continue to fire when pic is idle (e.g. Idle()). 0 = continue to fire interrupt during Idle().

  IPC6bits.T4IP = 0x07;   // set timer4 priority to level 7 (highest) - setting from 1 to 7, seems to make an unmeasurable positive difference.
  IFS1bits.T4IF = 0;      // clear timer4 interrupt status flag

  IEC1bits.T4IE = 1;      // enable timer4 interrupts

  T4CONbits.TCKPS = 0; // x1, 1=x8

#if !defined(PPG_ONLY_EXP)
  if (imu_adc_control_state & ACTIVE_ADC_RESP) {
    double fp = 1000000.0 / (double)BASE_TIMER4_INT_FREQ_HZ * FREQ_MULT; // freq period
    base_pr4 = (uint)fp + TIMER4_DRIFT_TWEAK_VAL;
    PR4 = base_pr4; // this only handles up to 64k, probably why 32000 interrupts didn't work. would need a 32 bit timer.    
    enable_ext_adc_interrupts();
  }
#else
  imu_adc_control_state &= ~(ACTIVE_ADC_RESP);
#endif

}
#endif

#if defined(ENABLE_IMU)

uint8 get_imu_control_state()
{
  return imu_adc_control_state & ACTIVE_IMU_ALL;
}

void set_imu_control_state( uint8 imu_cs )
{
  uint8 new_cs = imu_cs & ACTIVE_IMU_ALL;
  //--
  if ((new_cs & ACTIVE_IMU_ACC) && (imu_adc_control_state & ACTIVE_IMU_ACC) == 0)
    IMUACCEnable();
  else if ((new_cs & ACTIVE_IMU_ACC) == 0 && (imu_adc_control_state & ACTIVE_IMU_ACC))
    IMUACCDisable();
  //--
  if ((new_cs & ACTIVE_IMU_GYRO) && (imu_adc_control_state & ACTIVE_IMU_GYRO) == 0)
    IMUGyroEnable();
  else if ((new_cs & ACTIVE_IMU_GYRO) == 0 && (imu_adc_control_state & ACTIVE_IMU_GYRO))
    IMUGyroDisable();
  //--
  imu_adc_control_state &= ~(ACTIVE_IMU_ALL);
  imu_adc_control_state |= new_cs;
}

void sys_get_imu_data(IMU_DATA_STATE * imu_data_out)
{
  // CLOCKED: 158 us -- approx 10 us each

#if !defined(SPOOF_IMU_DATA)
  imu_data_out->acc_x_msb = imu_adc_control_state & ACTIVE_IMU_ACC ? (uint8)ReadIMUReg(0x3B) : 0;
  imu_data_out->acc_x_lsb = imu_adc_control_state & ACTIVE_IMU_ACC ? (uint8)ReadIMUReg(0x3C) : 0;
  imu_data_out->acc_y_msb = imu_adc_control_state & ACTIVE_IMU_ACC ? (uint8)ReadIMUReg(0x3D) : 0;
  imu_data_out->acc_y_lsb = imu_adc_control_state & ACTIVE_IMU_ACC ? (uint8)ReadIMUReg(0x3E) : 0;
  imu_data_out->acc_z_msb = imu_adc_control_state & ACTIVE_IMU_ACC ? (uint8)ReadIMUReg(0x3F) : 0;
  imu_data_out->acc_z_lsb = imu_adc_control_state & ACTIVE_IMU_ACC ? (uint8)ReadIMUReg(0x40) : 0;
  imu_data_out->gyro_x_msb = imu_adc_control_state & ACTIVE_IMU_GYRO ? (uint8)ReadIMUReg(0x43) : 0;
  imu_data_out->gyro_x_lsb = imu_adc_control_state & ACTIVE_IMU_GYRO ? (uint8)ReadIMUReg(0x44) : 0;
  imu_data_out->gyro_y_msb = imu_adc_control_state & ACTIVE_IMU_GYRO ? (uint8)ReadIMUReg(0x45) : 0;
  imu_data_out->gyro_y_lsb = imu_adc_control_state & ACTIVE_IMU_GYRO ? (uint8)ReadIMUReg(0x46) : 0;
  imu_data_out->gyro_z_msb = imu_adc_control_state & ACTIVE_IMU_GYRO ? (uint8)ReadIMUReg(0x47) : 0;
  imu_data_out->gyro_z_lsb = imu_adc_control_state & ACTIVE_IMU_GYRO ? (uint8)ReadIMUReg(0x48) : 0;
  imu_data_out->brd_temp_msb = imu_adc_control_state & ACTIVE_IMU_TEMP ? (uint8)ReadIMUReg(0x41) : 0;
  imu_data_out->brd_temp_lsb = imu_adc_control_state & ACTIVE_IMU_TEMP ? (uint8)ReadIMUReg(0x42) : 0;
#else
  static uint16 acc_x = 0x0000;
  static uint16 acc_y = 0x0000;
  static uint16 acc_z = 0x0000;
  static uint16 gyro_x = 0x0000;
  static uint16 gyro_y = 0x0000;
  static uint16 gyro_z = 0x0000;
  static uint16 brd_temp = 0x0000;
  const uint16 acc_x_incr = 0x0056;
  const uint16 acc_y_incr = 0x0123;
  const uint16 acc_z_incr = 0x0678;
  const uint16 gyro_x_incr = 0x0111;
  const uint16 gyro_y_incr = 0x0276;
  const uint16 gyro_z_incr = 0x0040;
  const uint16 brd_temp_incr = 0x0100;
  
  if (imu_adc_control_state & ACTIVE_IMU_ACC) {
    acc_x += acc_x_incr;
    imu_data_out->acc_x_msb = ((acc_x >> 8) & 0xFF);
    imu_data_out->acc_x_lsb = (acc_x & 0xFF);
  }
  if (imu_adc_control_state & ACTIVE_IMU_ACC) {
    acc_y += acc_y_incr;
    imu_data_out->acc_y_msb = ((acc_y >> 8) & 0xFF);
    imu_data_out->acc_y_lsb = (acc_y & 0xFF);
  }
  if (imu_adc_control_state & ACTIVE_IMU_ACC) {
    acc_z += acc_z_incr;
    imu_data_out->acc_z_msb = ((acc_z >> 8) & 0xFF);
    imu_data_out->acc_z_lsb = (acc_z & 0xFF);   
  }
  
  if (imu_adc_control_state & ACTIVE_IMU_GYRO) {
    gyro_x += gyro_x_incr;
    imu_data_out->gyro_x_msb = ((gyro_x >> 8) & 0xFF);
    imu_data_out->gyro_x_lsb = (gyro_x & 0xFF);
  }
  if (imu_adc_control_state & ACTIVE_IMU_GYRO) {
    gyro_y += gyro_y_incr;
    imu_data_out->gyro_y_msb = ((gyro_y >> 8) & 0xFF);
    imu_data_out->gyro_y_lsb = (gyro_y & 0xFF);
  }
  if (imu_adc_control_state & ACTIVE_IMU_GYRO) {
    gyro_z += gyro_z_incr;
    imu_data_out->gyro_z_msb = ((gyro_z >> 8) & 0xFF);
    imu_data_out->gyro_z_lsb = (gyro_z & 0xFF);   
  }
  
  if (imu_adc_control_state & ACTIVE_IMU_TEMP) {
    brd_temp += brd_temp_incr;
    imu_data_out->brd_temp_msb = ((brd_temp >> 8) & 0xFF);
    imu_data_out->brd_temp_lsb = (brd_temp & 0xFF);
  }
#endif

}

#endif


#if defined(ENABLE_EXT_ADC)

void set_adc_control_state(uint8 adc_cs)
{
  uint8 new_cs = adc_cs & ACTIVE_ADC_ALL;
  uint8 dif;
  //--
#if !defined(PPG_ONLY_EXP)
  //if ((new_cs & ACTIVE_ADC_RESP) && (imu_adc_control_state & ACTIVE_ADC_RESP) == 0) {
  if ((new_cs & ACTIVE_ADC_RESP) && !are_ext_adc_interrupts_enabled()) {
    double fp = 1000000.0 / (double)BASE_TIMER4_INT_FREQ_HZ * FREQ_MULT; // freq period
    base_pr4 = (uint)fp + TIMER4_DRIFT_TWEAK_VAL;
    PR4 = base_pr4; // this only handles up to 64k, probably why 32000 interrupts didn't work. would need a 32 bit timer.
    enable_ext_adc_interrupts();
  }
  //else if ((new_cs & ACTIVE_ADC_RESP) == 0 && (imu_adc_control_state & ACTIVE_ADC_RESP))
  else if ((new_cs & ACTIVE_ADC_RESP) == 0 && are_ext_adc_interrupts_enabled())
#else
  new_cs = adc_cs & ACTIVE_ADC_PPG;
#endif
    disable_ext_adc_interrupts();
  //--
  dif = ACTIVE_ADC_ALL;
  imu_adc_control_state &= ~(dif);
  imu_adc_control_state |= new_cs;
}

static void enable_ext_adc_io()
{
  SPI1STATbits.SPIEN = 0; // turn off SPI
  SPI1CON1bits.DISSDO = 1;
  SPI1CON1bits.MODE16 = 1; // set 16 bit mode
  SPI1CON1bits.CKE = 1;
  SPI1CON1bits.CKP = 0; 
  RPINR20bits.SDI1R = 13;
  SPI1STATbits.SPIEN = 1; // turn on SPI
}

static void disable_ext_adc_io()
{
  SPI1STATbits.SPIEN = 0; // turn off SPI
  SPI1CON1bits.DISSDO = 0; // set back to something the IMU is happy with
  SPI1CON1bits.MODE16 = 0;
  SPI1CON1bits.CKE = 0;
  SPI1CON1bits.CKP = 1;
  RPINR20bits.SDI1R = 10;
  SPI1STATbits.SPIEN = 1; // turn on SPI
}

static int32 ext_adc_data_avg_accum = 0;
//static int16 ext_adc_data_last_sample = 0;
static int32 ext_adc_data_avg_2p = 0;
//static int32 ext_adc_data_avg_2n = 0;

static int32 adc_data_result_1 = 0; // ext_adc_data_avg_accum
static int32 adc_data_result_2 = 0; // ext_adc_data_avg_p_alt - ext_adc_data_avg_n_alt
static bool  adc_data_valid = false;
static uint adc_data_counter = 0;

static uint adc_base_count_ceiling = ((BASE_TIMER4_INT_FREQ_HZ / EXPECT_CM_INTERV) * 2) - 1;

static bool  adc_interrupts_enabled = false;

// used for adaptive adjustment of 12k freq
static int32 t4_interupt_counter = 0;
int32 get_t4_interr_counter()
{
  return t4_interupt_counter;
}
void reset_t4_interr_counter()
{
  t4_interupt_counter = 0;
}
void adjust_t4_interr_period_register(int incr)
{
  PR4 = (uint)((int)base_pr4 + incr);
}

static bool process_interr_ext_adc_samples()
{
  
#if 1
  //int16  sample, sample_alt;//, temp;

  static int   sample_state = 0;
  static int32 ext_1_data_average = 0;
  static int32 ext_2p_data_average = 0;
  static int32 ext_2n_data_average = 0;

  enable_ext_adc_io();

  ADC_CNV = 1;

  DELAY_US(3); 
  
  if(sample_state == 0)
  {

      CURRENT_CTR = 1;
            
      SPI1BUF = 0;
      while(!SPI1STATbits.SPIRBF);
      SPI1BUF;//ext_1_data_average += SPI1BUF;
      SPI1BUF = 0;
      while(!SPI1STATbits.SPIRBF);
      ext_2n_data_average += SPI1BUF;
      
      sample_state = 1;
  }
  else
  {

      CURRENT_CTR = 0;
      
      SPI1BUF = 0;
      while(!SPI1STATbits.SPIRBF);
      ext_1_data_average += SPI1BUF;
      SPI1BUF = 0;
      while(!SPI1STATbits.SPIRBF);
      ext_2p_data_average += SPI1BUF;
      
      sample_state = 0;
      
  }
 
  ADC_CNV = 0;
  
//  current_ctr_state = !current_ctr_state; // toggle GPIO
//  CURRENT_CTR = current_ctr_state;
//
//  //--
//  
//  //--
//  // READ PPG
//  SPI1BUF = 0;
//  while(!SPI1STATbits.SPIRBF);
//  sample = SPI1BUF;
//  //--
//  // READ RESP / ECG
//  SPI1BUF = 0;
//  while(!SPI1STATbits.SPIRBF);
//  sample_alt = SPI1BUF; // alternating based on CURRENT_CTR
//  //--  
//  ADC_CNV = 0;
//  //--
//
////  ADC_CNV = 1;
////  //--
////  //--
////  //current_ctr_state = !current_ctr_state; // toggle GPIO
////  //CURRENT_CTR = current_ctr_state;
////  //--
////  // READ PPG
////  SPI1BUF = 0;
////  while(!SPI1STATbits.SPIRBF);
////  temp = SPI1BUF;
////  //--
////  // READ RESP / ECG
////  SPI1BUF = 0;
////  while(!SPI1STATbits.SPIRBF);
////  sample_alt = SPI1BUF; // alternating based on CURRENT_CTR
////  //--  
////  ADC_CNV = 0;
////  //--

  disable_ext_adc_io();

#endif

  /***************************************************************************************************
   *    Bio-Impedance Algorithm by Mike
   *    --------------------------------
   *    Subroutine runs at specified rate typically from 1000 - 32000 per second.
   *
   *    For each run:
   *
   *    1. Take ADC sample
   *    2. Toggle current injection GPIO pin
   *    3. Subtract current ADC sample from previous ADC sample and add to an accumulator variable
   *    4. Count how many runs
   *
   *    If number of runs = N (a parameter)
   *    Output accumulator variable to wireless
   *    Reset accumulator and run counter
   **************************************************************************************************/

#if defined(SPOOF_ADC_DATA) // SPOOF WAVEFORM
  static int32 foobar = -5000;
  static bool foobar_dir = true;
  if (foobar_dir) {
    foobar += 10;
    if (foobar >= 5000)
      foobar_dir = false;
  }
  else
  {
    foobar -= 10;
    if (foobar <= -5000)
      foobar_dir = true;
  }
#endif

//  if (imu_adc_control_state & ACTIVE_ADC_PPG) {
//#if defined(SPOOF_ADC_DATA)
//    ext_adc_data_avg_accum = foobar; // (ext_adc_data_last_sample - sample);
//#else
//    ext_adc_data_avg_accum = (int32)sample; // mimic read_ext_adc_samples_normal() @ 250/s - read most recent ppg spi value
//#endif
//    //---ext_adc_data_last_sample = sample;
//  }
//
//  if (imu_adc_control_state & ACTIVE_ADC_RESP) {
//    //if (current_ctr_state)
//#if defined(SPOOF_ADC_DATA)
//      ext_adc_data_avg_2p = foobar;
//#else
//      ext_adc_data_avg_2p += (int32)sample_alt;
//#endif
//    //else
//    //  ext_adc_data_avg_2n += (int32)sample_alt;
//  }

//  #if defined(SPOOF_ADC_DATA)
//      ext_adc_data_avg_accum = foobar; // (ext_adc_data_last_sample - sample);
//      ext_adc_data_avg_2p = foobar;
//  #else
//      ext_adc_data_avg_accum = (int32)sample; // mimic read_ext_adc_samples_normal() @ 250/s - read most recent ppg spi value
//      ext_adc_data_avg_2p += (int32)sample_alt;
//  #endif

  if(adc_data_counter >= adc_base_count_ceiling)
  {
#if defined(SPOOF_ADC_DATA)
    adc_data_result_1 = foobar;
    adc_data_result_2 = foobar;
#else
    adc_data_result_1 = ext_1_data_average;// / (BASE_TIMER4_INT_FREQ_HZ / EXPECT_CM_INTERV);
    adc_data_result_2 = ext_2p_data_average - ext_2n_data_average;
#endif

    //int32 t;
    //t = adc_data_result_1 > 0 ? adc_data_result_1 : -adc_data_result_1;
    //if (t > 500)
    //  adc_data_result_1 = 0;

    //t = adc_data_result_2 > 0 ? adc_data_result_2 : -adc_data_result_2;
    //if (t > 500)
    //  adc_data_result_2 = 0;

    ext_1_data_average = 0;
    adc_data_counter = 0;
    ext_2p_data_average = 0;
    ext_2n_data_average = 0;
    
    return true;
  }
  else
  {
    adc_data_counter++;
  }
      

//  if (++adc_data_counter >= adc_base_count_ceiling) {
//    adc_data_result_1 = ext_adc_data_avg_accum;
//    adc_data_result_2 = ext_adc_data_avg_2p;// - ext_adc_data_avg_2n;
//    ext_adc_data_avg_accum = 0;
//    adc_data_counter = 0;
//    ext_adc_data_avg_2p = 0;
//    //ext_adc_data_avg_2n = 0;
//    return true;
//  }

  return false;
}


void __attribute__((__interrupt__, __shadow__, __no_auto_psv__)) _T4Interrupt(void)
{

  t4_interupt_counter ++;
  
  if (process_interr_ext_adc_samples()) {
    adc_data_valid = true;
  }
 
  //TOGGLE_LED();
  //test_counter++;

  //DELAY_US(30);
  
  IFS1bits.T4IF = 0;     //  clear timer4 interrupt status flag
}

bool read_interr_ext_adc_samples( int32 * sample1_out, int32 * sample2_out )
{
  if (adc_data_valid) {
    //adc_data_valid = false;

    *sample1_out = adc_data_result_1;
    *sample2_out = adc_data_result_2;
    //adc_data_result_1 = 0;
    //adc_data_result_2 = 0;
    return true;
  }
  return false;
}

bool read_ext_adc_samples_normal( int32 * sample1_out, int32 * sample2_out )
{
  if ((imu_adc_control_state & ACTIVE_ADC_ANY) == ACTIVE_ADC_PPG) {

    int32 sample, sample_alt; // int32 == fix (don't change)

    enable_ext_adc_io();

#if !defined(PPG_ONLY_EXP)
    ADC_CNV = 1;
    //--
    DELAY_US(3);
    //--
    //current_ctr_state = !current_ctr_state; // toggle GPIO
    //CURRENT_CTR = current_ctr_state;
    //--
    // READ PPG
    SPI1BUF = 0;
    while(!SPI1STATbits.SPIRBF);
    sample = (int32)(int)SPI1BUF;
    //--
    // READ RESP / ECG
    SPI1BUF = 0;
    while(!SPI1STATbits.SPIRBF);
    sample_alt = (int32)(int)SPI1BUF; // alternating based on CURRENT_CTR
    //--  
    ADC_CNV = 0;
    
#else // defined(PPG_ONLY_EXP)

#if defined(PPG_ONLY_EXP_OPTION_LEFT)

    int16 tempvar1, tempvar2, tempvar3;

    ADC_CNV = 1;

    current_ctr_state = !current_ctr_state; // toggle GPIO
    CURRENT_CTR = current_ctr_state;
    DELAY_US(100);
    SPI1BUF = 0;
    while(!SPI1STATbits.SPIRBF);
    tempvar1 = SPI1BUF;

    current_ctr_state = !current_ctr_state; // toggle GPIO
    CURRENT_CTR = current_ctr_state;
    DELAY_US(100);
    SPI1BUF = 0;
    while(!SPI1STATbits.SPIRBF);
    tempvar2 = SPI1BUF;

    current_ctr_state = !current_ctr_state; // toggle GPIO
    CURRENT_CTR = current_ctr_state;
    DELAY_US(100);
    SPI1BUF = 0;
    while(!SPI1STATbits.SPIRBF);
    tempvar3 = SPI1BUF;

    ADC_CNV = 0;

    sample = tempvar2 - tempvar1;
    sample_alt = tempvar3 - tempvar1;

#else

    static int16 phase = 0;

    ADC_CNV = 1;

    sample_alt = sample = 0;

    if (phase == 0) {
      SPI1BUF = 0;
      while(!SPI1STATbits.SPIRBF);
      sample = SPI1BUF;
      phase = 1;
    }
    else if (phase == 1) {
      SPI1BUF = 0;
      while(!SPI1STATbits.SPIRBF);
      sample_alt = SPI1BUF;
      phase = 0;
    }

    current_ctr_state = !current_ctr_state; // toggle GPIO
    CURRENT_CTR = current_ctr_state;

    ADC_CNV = 0;

#endif


#endif
    
    disable_ext_adc_io();
    
#if !defined(SPOOF_ADC_DATA) // SPOOF WAVEFORM
    *sample1_out = (int32)sample;
    *sample2_out = (int32)sample_alt;
#else
    static int16 spoof_sample = -5000;
    static bool spoof_up_dir = true;
    if (spoof_up_dir) {
      spoof_sample += 10;
      if (spoof_sample >= 5000)
        spoof_up_dir = false;
    }
    else {
      spoof_sample -= 10;
      if (spoof_sample <= -5000)
        spoof_up_dir = true;
    }
    *sample1_out = (int32)spoof_sample;
    *sample2_out = (int32)spoof_sample;
#endif

    return true;
  }
  return false;
}

void  enable_ext_adc_interrupts()
{
  TMR4 = 0;
  t4_interupt_counter = 0;
  ext_adc_data_avg_accum = 0;
  //ext_adc_data_last_sample = 0;
  adc_data_counter = 0;
  ext_adc_data_avg_2p = 0;
  //ext_adc_data_avg_2n = 0;
  T4CONbits.TON = 1;
  adc_interrupts_enabled = true;
}

void  disable_ext_adc_interrupts()
{
  adc_interrupts_enabled = false;
  T4CONbits.TON = 0;
  ext_adc_data_avg_accum = 0;
  //ext_adc_data_last_sample = 0;
  adc_data_counter = 0;
  ext_adc_data_avg_2p = 0;
  //ext_adc_data_avg_2n = 0;
  adc_data_valid = false;
}

bool are_ext_adc_interrupts_enabled()
{
  return adc_interrupts_enabled;
}

#endif
