
//
// MPLAB Error reporting (and inappropriate hinting) FIX: Becareful with overriding build options on a per file basis (esp source modules). Do project wide settings if you can. Otherwise <xc.h> etc will fail to resolve in the IDE.
//

#include "stdinc.h"
#include "stdtypes.h"
#include "sysio.h"
#include "netio.h"

#define NRF24_CHANNEL   (99)  // 0-125
//#define RX_PAYLOAD_SIZE (6)   // should match stream_XXX_bytes_r() below
#define RX_PAYLOAD_SIZE (10)   // should match stream_XXX_bytes_r() below

// presumptive helper macro (v is an array)
#define SB(v,i) net_spi_data_w(v[i]);
#define RB(v,i) net_spi_data_r(v[i]);
#define stream_32_bytes_w(v) SB(v,0); SB(v,1); SB(v,2); SB(v,3); SB(v,4); SB(v,5); SB(v,6); SB(v,7);\
                             SB(v,8); SB(v,9); SB(v,10); SB(v,11); SB(v,12); SB(v,13); SB(v,14); SB(v,15);\
                             SB(v,16); SB(v,17); SB(v,18); SB(v,19); SB(v,20); SB(v,21); SB(v,22); SB(v,23);\
                             SB(v,24); SB(v,25); SB(v,26); SB(v,27); SB(v,28); SB(v,29); SB(v,30); SB(v,31);

// this should match RX_PAYLOAD_SIZE (above)
//#define stream_6_bytes_r(v)  RB(v,0); RB(v,1); RB(v,2); RB(v,3); RB(v,4); RB(v,5);
#define stream_10_bytes_r(v)  RB(v,0); RB(v,1); RB(v,2); RB(v,3); RB(v,4); RB(v,5); RB(v,6); RB(v,7); RB(v,8); RB(v,9);

static uint8 custom_addrs[6][5] = {
#if 0
  { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 },
  { 0xB3, 0xB4, 0xB5, 0xB6, 0xE1 },
  { 0xB3, 0xB4, 0xB5, 0xB6, 0xCD },
  { 0xB3, 0xB4, 0xB5, 0xB6, 0xA3 },
  { 0xB3, 0xB4, 0xB5, 0xB6, 0x2E },
  { 0xB3, 0xB4, 0xB5, 0xB6, 0x15 }
#else // fix - reverse the byte order
  { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 },
  { 0xE1, 0xB6, 0xB5, 0xB4, 0xB3 },
  { 0xCD, 0xB6, 0xB5, 0xB4, 0xB3 },
  { 0xA3, 0xB6, 0xB5, 0xB4, 0xB3 },
  { 0x2E, 0xB6, 0xB5, 0xB4, 0xB3 },
  { 0x15, 0xB6, 0xB5, 0xB4, 0xB3 }
#endif
};

static uint16 counter_val = 0; // will need to be reset if in idle mode, then comes back up ?

static uint8 s_data[32];
static uint8 r_data[32];

#define SENSOR_SEND_POINT            (900)
#define SENSOR_INTERVAL_WINDOW       (550)   // target time between each sensor tranmission
#define TIME_SLOT_CAP                (3750)  // before CM moves on - this includes it's own small overhead and the 100 us reserved for uploading data to GUI (PM) - Rule: SENSOR_SEND_POINT + (SENSOR_INTERVAL_WINDOW*4) < TIME_SLOT_CAP

const uint SENSOR_OFFSET = (SENSOR_INTERVAL_WINDOW * SENSOR_MODULE) + SENSOR_SEND_POINT;

//#define ENABLE_TIME_REPORTING

static void sensor_loop()
{
#if defined(ENABLE_IMU)
  IMU_DATA_STATE imu_data;
#endif
  uint32 cm_delay = 8000; // us -- initialize to a really large value - it'll result in an initial jump is power, then followed by really low power drop until CM comes back online.
#if defined(ENABLE_TIME_REPORTING)
  static uint16 timer = 0;
  static uint16 t_counter = 0;
  static bool send_timer = false;
#endif
  uint initial_time;
  uint time_to_delay;
  //uint sync_threshold;
  //uint pr_div;

  // POWER SAVING IDLE() REMOVED FROM HERE... Not much savings anyway and addresses the issue with predictable timing afterwards.
    
  net_turn_radio_on(); // approx 11 us

//#if defined(ENABLE_EXT_ADC)
  sys_start_timer32();  // not compatible with code right below here ??
//#endif

  // wait and listen - timed at 198 us
  while (!(net_irq_pin_active() && net_irq_rx_dr_active())) {
    if (sys_query_timer32() > cm_delay) { // is CM even broadcasting? Nope.
      net_turn_radio_off();
#if defined(ENABLE_EXT_ADC)
      bool inten = are_ext_adc_interrupts_enabled();
      if (inten)
        disable_ext_adc_interrupts();
#endif
      (void)sys_cancel_timer32();
      // possibly turn off IMU as well, etc - esp if gyro is enabled.
      PR1 = 4000 * FREQ_MULT; // sleep for 4000 us (a whole interval)
      T1CONbits.TON = 1;
      Idle();
      T1CONbits.TON = 0;
      cm_delay = 400; // reduce final delay to save power since CM is not available.
      net_turn_radio_on();
#if defined(ENABLE_EXT_ADC)
      if (inten)
        enable_ext_adc_interrupts();
#endif
      sys_start_timer32();
    }
  }
  
  // fix - restart after nrf24 receive interrupt occurs
  sys_stop_timer32();
  
  bool adc_ints_en = are_ext_adc_interrupts_enabled(); // use original state 

//#if 0 // defined(ENABLE_EXT_ADC)
//  static int32  cm_interv_counter = 0;
//  static int    moving_delta = 0;
//  //int           temp_delta = 0; // temporary test reg only
//  
//  if (adc_ints_en && sys_query_timer32() > 16000) { // only enforce this synchron if reasonably connected else reset
//    reset_t4_interr_counter(); // reset both the 250/s counter and 12k/s counters
//    cm_interv_counter = 0;
//    moving_delta = 0;
//  }
//#endif

  sys_start_timer32();
  
  net_turn_radio_off(); // turn off radio asap
   
  // clear: control TX -> sensor RX
  memset(r_data,0,RX_PAYLOAD_SIZE);

  net_receive_begin();
  //stream_6_bytes_r(r_data);
  stream_10_bytes_r(r_data);
  net_receive_end();

  net_clear_all_irqs();
  net_clear_rx_fifo();

  // validate incoming packet
  if (r_data[0] != 0x34 || r_data[RX_PAYLOAD_SIZE-1] != 0x34) {
    goto fail;
  }
    
  if (!adc_ints_en)
    prep_battery_voltage_sampling(); 
  
#if 0 // defined(ENABLE_EXT_ADC)
  if (adc_ints_en) { // only check on valid packets - e.g. after fail test above
    int32 lc = get_t4_interr_counter();

    if (lc > 0x7800000) { // approx 46 hours secs (0x7800000 * 12000)
      reset_t4_interr_counter();
      cm_interv_counter = 0;
      moving_delta = 0;
    }
    else if ((++cm_interv_counter % EXPECT_CM_INTERV) == 0) {
      moving_delta = (int)(lc - (int32)(cm_interv_counter * (BASE_TIMER4_INT_FREQ_HZ/EXPECT_CM_INTERV)));

      if (moving_delta > 5) {
        adjust_t4_interr_period_register(1);
      }
      else
        adjust_t4_interr_period_register(0);

      //temp_delta = moving_delta;

    }

    //if ((counter_here % 1000) == 0) {
    //  uint32 lc = get_long_counter_a();
    //  s_data[29] = (lc >> 8) & 0xFF;
    //  s_data[30] = lc & 0xFF;
    //  counter_here = 0;
    //}

  }
  else {
    reset_t4_interr_counter();
    cm_interv_counter = 0;
    moving_delta = 0;
  }
#endif

  // build packet: sensor TX -> control RX
  memset(s_data,0,32);

  //
  // BEGIN: Dynamic IO
  //
  uint8 cmd_byte = r_data[SENSOR_MODULE+1];
  uint8 cmd_byte2 = r_data[RX_PAYLOAD_SIZE-SENSOR_MODULE-2];

#if defined(ENABLE_IMU)
  if (cmd_byte == cmd_byte2 && (cmd_byte & UPDATE_IMU_ADC_FLAG)) { // check for update from control module to change imu control state for this sensor
    //if ((cmd_byte & (ACTIVE_ADC_PPG|ACTIVE_IMU_ACC)) == (ACTIVE_ADC_PPG|ACTIVE_IMU_ACC))
      set_imu_control_state(cmd_byte);
  }
#endif
  
#if defined(ENABLE_EXT_ADC)
  if (cmd_byte == cmd_byte2 && (cmd_byte & UPDATE_IMU_ADC_FLAG)) { // check for update from control module
    //if ((cmd_byte & (ACTIVE_ADC_PPG|ACTIVE_IMU_ACC)) == (ACTIVE_ADC_PPG|ACTIVE_IMU_ACC))
      set_adc_control_state(cmd_byte);
  }
#endif

#if defined(ENABLE_IMU)
   
  memset(&imu_data,0,sizeof(IMU_DATA_STATE));
  
  if (get_imu_control_state()) {
    sys_get_imu_data(&imu_data);
  }
  
#endif


  // DONE: Protocol Schema

  s_data[0] = 0x34;
  s_data[1] = SENSOR_MODULE + 1;
  *(uint16*)&s_data[2] = ++counter_val;
  s_data[31] = 0x34;

#if defined(ENABLE_IMU)  // at present the sensor always sends data, whether valid or nil.
  s_data[4] = imu_data.acc_x_msb;
  s_data[5] = imu_data.acc_x_lsb;
  s_data[6] = imu_data.acc_y_msb;
  s_data[7] = imu_data.acc_y_lsb;
  s_data[8] = imu_data.acc_z_msb;
  s_data[9] = imu_data.acc_z_lsb;

  s_data[10] = imu_data.gyro_x_msb;
  s_data[11] = imu_data.gyro_x_lsb;
  s_data[12] = imu_data.gyro_y_msb;
  s_data[13] = imu_data.gyro_y_lsb;
  s_data[14] = imu_data.gyro_z_msb;
  s_data[15] = imu_data.gyro_z_lsb;

  s_data[16] = imu_data.brd_temp_msb;
  s_data[17] = imu_data.brd_temp_lsb;
    
#endif

#if defined(ENABLE_EXT_ADC)
  if (adc_ints_en) {
    int32 adc_data = 0;
    int32 adc_data2 = 0;
    if (read_interr_ext_adc_samples(&adc_data,&adc_data2)) {     
      //adc_data = 5000;
      //adc_data2 = -5000;
      memcpy(&s_data[18],&adc_data,sizeof(int32));
      memcpy(&s_data[22],&adc_data2,sizeof(int32));      
      //*((int32*)&s_data[20]) = 2400;//adc_data; // MUST BE ALIGNED ON DWORD BOUNDARY FOR CM
      //*((int32*)&s_data[24]) = 4800;//adc_data2; // MUST BE ALIGNED ON DWORD BOUNDARY FOR CM
      //s_data[18] = ((uint32)adc_data >> 16) & 0xFF; // optimization: send only 24 bits (12000 * 1024 <= 24 bits) worse case
      //s_data[19] = ((uint32)adc_data >> 8) & 0xFF;
      //s_data[20] = (uint32)adc_data & 0xFF;
      //s_data[21] = ((uint32)adc_data2 >> 16) & 0xFF; // optimization: send only 24 bits (12000 * 1024 <= 24 bits) worse case
      //s_data[22] = ((uint32)adc_data2 >> 8) & 0xFF;
      //s_data[23] = (uint32)adc_data2 & 0xFF;
  
    }
  }
  else {
    int32 adc_data = 0;
    int32 adc_data2 = 0;
    if (read_ext_adc_samples_normal(&adc_data,&adc_data2)) {
      memcpy(&s_data[18],&adc_data,sizeof(int32));
      memcpy(&s_data[22],&adc_data2,sizeof(int32));      
      //*((int32*)&s_data[18]) = adc_data; // MUST BE ALIGNED ON DWORD BOUNDARY FOR CM
      //s_data[18] = ((uint32)adc_data >> 16) & 0xFF; // optimization: send only 24 bits (12000 * 1024 <= 24 bits) worse case
      //s_data[19] = ((uint32)adc_data >> 8) & 0xFF;
      //s_data[20] = (uint32)adc_data & 0xFF;
    }
  }
    
#endif
  
  //get_battery_voltage
  if (!adc_ints_en)
      finish_battery_voltage_sampling();
  s_data[26] = get_battery_voltage_val();
    
  // 27-30 available, 29-30 used for internal debugging

  //s_data[29] = (temp_delta >> 8) & 0xFF;
  //s_data[30] = temp_delta & 0xFF;
  
#if defined(ENABLE_TIME_REPORTING)
  // defer to next iteration
  if (send_timer) {
    s_data[29] = (timer >> 8) & 0xFF;
    s_data[30] = timer & 0xFF;
    send_timer = false;
  }
#endif

  net_set_tx_mode();

  net_queue_transmit_begin();
  stream_32_bytes_w(s_data);
  net_queue_transmit_end();
   
  initial_time = sys_query_timer32();
  
  // sync to a real time slot for this sensor
  if (initial_time <= SENSOR_OFFSET - 40) {

    time_to_delay = (SENSOR_OFFSET - initial_time);
    //---sync_threshold = initial_time + time_to_delay;

#if defined(ENABLE_EXT_ADC)
    if (adc_ints_en) {
      //---sys_start_timer32();
      PR1 = time_to_delay * FREQ_MULT;
      T1CONbits.TON = 1;
      loop_3:
      Idle();
      if (sys_query_timer32() < SENSOR_OFFSET)
        goto loop_3;
      T1CONbits.TON = 0;
      //---sys_cancel_timer32();
    }
    else {
#endif
        PR1 = (time_to_delay * FREQ_MULT);
        T1CONbits.TON = 1;
        Idle();
        T1CONbits.TON = 0;
#if defined(ENABLE_EXT_ADC)
    }
#endif

  }

  net_turn_radio_on(); // begin transmitting

  // wait for transmission to complete
  // takes approx 286 us
  while (!(net_irq_pin_active() && net_irq_tx_ds_or_max_rt_active()));
  //DELAY_US(290);

  // turn off asap
  net_turn_radio_off();

  fail:;
  
  net_clear_all_irqs();
  net_clear_tx_fifo();

  net_set_rx_mode();

  ///////////////////////////////////////////////////////////

  time_to_delay = sys_stop_timer32();

#if defined(ENABLE_TIME_REPORTING)
  if ((++t_counter % 1000) == 0) {
    timer = time_to_delay;
    t_counter = 0;
    send_timer = true;
  }
#endif

  time_to_delay = (TIME_SLOT_CAP - time_to_delay);

  // 12400 - max
#if defined(ENABLE_EXT_ADC)
  if (adc_ints_en) {
//    time_to_delay = time_to_delay - 195; // probably overhead for nrf read at top (~194) - OR interrupt overhead during Idle() and processing HERE -- tuned for 12k
//    sys_start_timer32();
//    PR1 = time_to_delay * FREQ_MULT; // 10100 - static for 12k
//    T1CONbits.TON = 1;
//    loop_4:
//    Idle();
//    if (sys_query_timer32() < time_to_delay)
//      goto loop_4;
//    T1CONbits.TON = 0;
//    sys_cancel_timer32();
  }
  else {
#endif
//    PR1 = (time_to_delay * FREQ_MULT); // dev ref: 11904... 11800
//    T1CONbits.TON = 1;
//    Idle();
//    T1CONbits.TON = 0;
    
////    PR1 = (time_to_delay * FREQ_MULT) / 16; // dev ref: 11904... 11800
////    CLKDIVbits.RCDIV = 4;
////    T1CONbits.TON = 1;
////    Idle();
////    CLKDIVbits.RCDIV = 0; // 0= 8mhz, 1=4mhz etc
////    T1CONbits.TON = 0;
    
#if defined(ENABLE_EXT_ADC)
  }
#endif
  
  //----OLD ----- net_turn_radio_on(); // approx 11 us

}

int main()
{
  
  DELAY_MS(100); // PIC24 major bug fix.. the board needed more start up time to load cold settings to/from flash
                 // important for power down/up/post-rf-interference dynamic discovery
    
  sys_handle_power_button();

  sys_init_pic24_ports();

  net_init_nRF24_ports();

  ledon();

  net_turn_radio_off();

  net_config_nRF24(0,0x01,5,0,0,NRF24_CHANNEL,
                  &custom_addrs[1][0],
                  &custom_addrs[1][0],
                  custom_addrs[2][0],
                  custom_addrs[3][0],
                  custom_addrs[4][0],
                  custom_addrs[5][0],
                  &custom_addrs[0][0],
                  RX_PAYLOAD_SIZE);
  //--
  //---net_report_regs();
  //--
  net_clear_all_irqs();
  net_clear_tx_fifo();
  net_clear_rx_fifo();

  sys_init_timer1();
  sys_init_timer32();

  //-------OLD---------net_turn_radio_on(); // turn on RX for sensor
  
  
#if defined(ENABLE_EXT_ADC)
  sys_init_timer_4_inter();
#endif
 
  while (true) {
    //--
    sensor_loop();
    //--
  }
 
  return 0;
}


