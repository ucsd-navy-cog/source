
#include "stdinc.h"
#include "stdtypes.h"
#include "sysio.h"
#include "netio.h"

#define REPORT_INTERVAL       (4000)
#define CONTROL_MODULE_TIMING (4000.0) // becareful with modifying this.. there may be some new untested side effects now..
#define NRF24_CHANNEL         (99)  // 0-125 mhz (2.4 - 2.525)
//#define TX_PAYLOAD_SIZE       (6)   // to sensors - should match stream_XXX_bytes_w() below
#define TX_PAYLOAD_SIZE       (10)   // to sensors - should match stream_XXX_bytes_w() below

#define SENSOR_CUTOFF         (800) // (CONTROL_MODULE_TIMING - SENSOR_CUTOFF) == is the real time point that the CM cuts off waiting for responses from sensors. Sensor 3 is esp sensitive to this.
                                    // subtracted delta (in us) should be sufficient for uploading data to Presentation Module. UPDATE. we need to speed up upload... -100 to -200 deficit.

// presumptive helper macro (v is an array)
#define SB(v,i) net_spi_data_w(v[i]);
#define RB(v,i) net_spi_data_r(v[i]);
#define stream_32_bytes_r(v) RB(v,0); RB(v,1); RB(v,2); RB(v,3); RB(v,4); RB(v,5); RB(v,6); RB(v,7);\
                             RB(v,8); RB(v,9); RB(v,10); RB(v,11); RB(v,12); RB(v,13); RB(v,14); RB(v,15);\
                             RB(v,16); RB(v,17); RB(v,18); RB(v,19); RB(v,20); RB(v,21); RB(v,22); RB(v,23);\
                             RB(v,24); RB(v,25); RB(v,26); RB(v,27); RB(v,28); RB(v,29); RB(v,30); RB(v,31);
// should match TX_PAYLOAD_SIZE
//#define stream_6_bytes_w(v)  SB(v,0); SB(v,1); SB(v,2); SB(v,3); SB(v,4); SB(v,5);
#define stream_10_bytes_w(v)  SB(v,0); SB(v,1); SB(v,2); SB(v,3); SB(v,4); SB(v,5); SB(v,6); SB(v,7); SB(v,8); SB(v,9);

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

static uint32 sensor_counts[4] = {0,0,0,0};
static uint32 last_sensor_counts[4] = {0,0,0,0};
static uint16 sensor_counter_vals[4] = {0,0,0,0};

static IMU_DATA_STATE imu_data[4];
static IMU_DATA_STATE imu_data_history[4]; // for interpolation of previous imu data

static int32 ppg_adc_data_i32[4];
static int32 ppg_adc_data_i32_history[4];

static int32 resp_adc_data_i32[4];
static int32 resp_adc_data_i32_history[4];

static uint8 battery_volt[4];
static uint8 battery_volt_history[4];

static uint32 sensor_handling_ts;

//static uint8 rdata[32];
//static int32 spoof_temp[4] = {0};

static void report_counter(uint32 factor_denom)
{
#if !defined(ENABLE_UPLOAD_DATA)
  static uint32 counter = 0;
  uint32 c = ++counter / 4; // BUG CREEP: Overflow issue on reporting
  uint32 dc = factor_denom / 4;

  if ((counter % factor_denom) == 0) {
    char buf[128];
    uint32 loss[4];
    uint32 i;
    for (i=0;i<4;i++) {
      loss[i] = last_sensor_counts[i] + dc - sensor_counts[i];
      if (loss[i] > dc)
        loss[i] = 0;
    }  
    sprintf(buf,"%lu/(%lu) --- %lu %lu %lu %lu --- %lu %lu %lu %lu" ,counter,c,
      sensor_counts[0],sensor_counts[1],sensor_counts[2],sensor_counts[3],
      loss[0], loss[1], loss[2], loss[3]
    );

    for (i=0;i<4;i++)
      last_sensor_counts[i] = sensor_counts[i];

    puts(buf);

#if 0 // test time stamps between reports - should be 72000000 for 3M mode
    static uint32 last_ct = 0;
    uint32 ct = ReadCoreTimer();
    printf("%u\n",ct - last_ct);
    last_ct = ct;
#endif

    //printf("spoof data = %ld %ld %ld %ld\n",spoof_temp[0],spoof_temp[1],spoof_temp[2],spoof_temp[3]);
    //spoof_temp[0] = spoof_temp[1] = spoof_temp[2] = spoof_temp[3] = 0;
#if 0
    for (i=0;i<32;i++)
      printf("%02X ",rdata[i]);
    printf("\n");
#endif

  }
#endif
}

static void notify_sensors() // broadcast to common address for all available sensors
{
  //uint32 counter = 0;
  uint8 sdata[TX_PAYLOAD_SIZE];

  // build packet: control TX -> sensor RX
  sdata[0] = 0x34;
  sdata[TX_PAYLOAD_SIZE-1] = 0x34;
  //--
  sdata[1] = 0x00; // set IMU state for SM0
  sdata[2] = 0x00; // set IMU state for SM1
  sdata[3] = 0x00; // set IMU state for SM2
  sdata[4] = 0x00; // set IMU state for SM3
  sdata[5] = 0x00; // set IMU state for SM3
  sdata[6] = 0x00; // set IMU state for SM2
  sdata[7] = 0x00; // set IMU state for SM1
  sdata[8] = 0x00; // set IMU state for SM0
  //--

  // Read command bytes from PM (presentation module)
  if (sys_rx_uart_has_data()) {
    uint sid;
    uint8 cb, sb;
    cb = sys_get_rx_uart_byte();
    sid = ((cb & 0x60) >> 5);
    sb = UPDATE_IMU_ADC_FLAG;
    if (cb & CMD_SET_TEMP)
      sb |= ACTIVE_IMU_TEMP;
    if (cb & CMD_SET_GYRO)
      sb |= ACTIVE_IMU_GYRO;
    if (cb & CMD_SET_ACCEL)
      sb |= ACTIVE_IMU_ACC;
    if (cb & CMD_SET_PPG)
      sb |= ACTIVE_ADC_PPG;
    if (cb & CMD_SET_RESP)
      sb |= ACTIVE_ADC_RESP;
    sdata[sid+1] = sb;
    sdata[TX_PAYLOAD_SIZE-sid-2] = sb;
    // sdata[sid+1]  = UPDATE_IMU_ADC_FLAG | ACTIVE_IMU_GYRO | ACTIVE_IMU_ACC | ACTIVE_IMU_TEMP | ACTIVE_ADC_PPG;  -- TESTING ONLY
  }

  net_queue_transmit_begin();
  //stream_6_bytes_w(sdata);
  stream_10_bytes_w(sdata);
  net_queue_transmit_end();
  net_turn_radio_on();

  //printf("here 101\n");

  while (!(net_irq_pin_active() && net_irq_tx_ds_or_max_rt_active()));

  //printf("here 102\n");

  net_turn_radio_off();
  net_clear_all_irqs();
  net_clear_tx_fifo();
  net_set_rx_mode();
  net_turn_radio_on();

  //printf("here 103\n");

}

static bool active_sensors = false;
static bool got_sensor_data[4];

static void get_sensor_data()
{
  uint8 rdata[32];

  uint32 i,n;
  const double threshold = CONTROL_MODULE_TIMING - SENSOR_CUTOFF;

  active_sensors = false;

  for (n=0;n<4;n++) {
    bool failed = true;

    while (true) {
      //--
      double r = sys_query_profile(sensor_handling_ts);
      if (r > threshold)
        break;
      //--
      if ((net_irq_pin_active() && net_irq_rx_dr_active())) {
        failed = false;
        active_sensors = true;
        break;
      }
    }
   
    net_clear_all_irqs();

    if (failed)
      goto fail;

    //
    // validate size of payload coming in (==32). if over it's a bad packet anyway. if it's under then the protocol is messed up... e.g. sender isnt sending 32 bytes.
    //
    uint8 numbytes = 0;
    net_spi_data_begin(nrf24l01_R_RX_PL_WID);
    net_spi_data_r(numbytes);
    net_spi_data_end();
    //printf("rxfifo = %u\n",(uint)numbytes);
    if (numbytes != 32) {
      net_clear_rx_fifo();
      goto fail;
    }

    for (i=0;i<32;i++)
      rdata[i] = 0x00;

    net_receive_begin();
    stream_32_bytes_r(rdata);
    net_receive_end();
    net_clear_rx_fifo();

    // validate incoming packet
    // validate acceptable sensor id's
    if ((rdata[0] != 0x34) || (rdata[31] != 0x34) || rdata[1] > 0x04 || !rdata[1])
      failed = true;
   
    if (!failed) {
      uint sid = (uint)rdata[1] - 1;
      //--
      sensor_counts[sid]++;
      sensor_counter_vals[sid] = *(uint16*)&rdata[2]; // 2/3 - // MUST BE ALIGNED ON WORD BOUNDARY
      //--
      memcpy(&imu_data[sid],&rdata[4],sizeof(IMU_DATA_STATE));
      memcpy(&ppg_adc_data_i32[sid],&rdata[18],sizeof(int32));
      memcpy(&resp_adc_data_i32[sid],&rdata[22],sizeof(int32));
      battery_volt[sid] = (rdata[26]);
      
      //--
      memcpy(&imu_data_history[sid],&imu_data[sid],sizeof(IMU_DATA_STATE));
      memcpy(&ppg_adc_data_i32_history[sid],&ppg_adc_data_i32[sid],sizeof(int32));
      memcpy(&resp_adc_data_i32_history[sid],&resp_adc_data_i32[sid],sizeof(int32));
      memcpy(&battery_volt_history[sid],&battery_volt[sid],sizeof(uint8));
      
      got_sensor_data[sid] = true;

#if !defined(ENABLE_UPLOAD_DATA)
#if 1
      uint16 time_used = ((uint16)rdata[29] << 8) + rdata[30]; 
      if (time_used)
        printf("time_used = %u\n",time_used);
#endif
#endif
    }

    fail:
    report_counter(REPORT_INTERVAL);

  }

  net_turn_radio_off();
  net_set_tx_mode(); // get parallelization going here

}


#define SENSOR_BUFFER_SIZE (126)
static void upload_sensor_data()
{
  uint8 buffer[SENSOR_BUFFER_SIZE];
  static uint8 sample_interv_count = 0;  
  uint i = 0, n;
  register int16 lsb1, lsb2, msb;

  // header - 2 bytes
  buffer[i++] = 0xFF;  // 0xFF sync header
  buffer[i++] = sample_interv_count++ & 0x7F; //0-127 packet counter

  // body - currently 3 * 10 * 4 = 120 bytes
  for (n=0;n<4;n++) {

    msb = imu_data[n].acc_x_msb;
    lsb2 = imu_data[n].acc_x_lsb;
    lsb1 = 0x00;
    buffer[i++] = (msb & 0xFE);
    buffer[i++] = (((msb & 0x01) << 7) | ((lsb2 & 0xFC) >> 1));
    buffer[i++] = (((lsb2 & 0x03) << 6) | ((lsb1 & 0xF8) >> 2));

    msb = imu_data[n].acc_y_msb;
    lsb2 = imu_data[n].acc_y_lsb;
    lsb1 = 0x00;
    buffer[i++] = (msb & 0xFE);
    buffer[i++] = (((msb & 0x01) << 7) | ((lsb2 & 0xFC) >> 1));
    buffer[i++] = (((lsb2 & 0x03) << 6) | ((lsb1 & 0xF8) >> 2));

    msb = imu_data[n].acc_z_msb;
    lsb2 = imu_data[n].acc_z_lsb;
    lsb1 = 0x00;
    buffer[i++] = (msb & 0xFE);
    buffer[i++] = (((msb & 0x01) << 7) | ((lsb2 & 0xFC) >> 1));
    buffer[i++] = (((lsb2 & 0x03) << 6) | ((lsb1 & 0xF8) >> 2));

    msb = imu_data[n].gyro_x_msb;
    lsb2 = imu_data[n].gyro_x_lsb;
    lsb1 = 0x00;
    buffer[i++] = (msb & 0xFE);
    buffer[i++] = (((msb & 0x01) << 7) | ((lsb2 & 0xFC) >> 1));
    buffer[i++] = (((lsb2 & 0x03) << 6) | ((lsb1 & 0xF8) >> 2));

    msb = imu_data[n].gyro_y_msb;
    lsb2 = imu_data[n].gyro_y_lsb;
    lsb1 = 0x00;
    buffer[i++] = (msb & 0xFE);
    buffer[i++] = (((msb & 0x01) << 7) | ((lsb2 & 0xFC) >> 1));
    buffer[i++] = (((lsb2 & 0x03) << 6) | ((lsb1 & 0xF8) >> 2));

    msb = imu_data[n].gyro_z_msb;
    lsb2 = imu_data[n].gyro_z_lsb;
    lsb1 = 0x00;
    buffer[i++] = (msb & 0xFE);
    buffer[i++] = (((msb & 0x01) << 7) | ((lsb2 & 0xFC) >> 1));
    buffer[i++] = (((lsb2 & 0x03) << 6) | ((lsb1 & 0xF8) >> 2));

    msb = imu_data[n].brd_temp_msb;
    lsb2 = imu_data[n].brd_temp_lsb;
    lsb1 = 0x00;
    buffer[i++] = (msb & 0xFE);
    buffer[i++] = (((msb & 0x01) << 7) | ((lsb2 & 0xFC) >> 1));
    buffer[i++] = (((lsb2 & 0x03) << 6) | ((lsb1 & 0xF8) >> 2));
      
    //msb = ppg_adc_data[n][1];
    //lsb2 = ppg_adc_data[n][2];
    //lsb1 = 0x00;//ppg_adc_data[n][2];
    msb = (ppg_adc_data_i32[n] >> 8) & 0xFF;//(ppg_adc_data_i32[n] >> 16) & 0xFF;
    lsb2 = (ppg_adc_data_i32[n]) & 0xFF;//(ppg_adc_data_i32[n] >> 8) & 0xFF;
    lsb1 = 0x0;//(ppg_adc_data_i32[n]) & 0xFF;
    buffer[i++] = (msb & 0xFE);
    buffer[i++] = (((msb & 0x01) << 7) | ((lsb2 & 0xFC) >> 1));
    buffer[i++] = (((lsb2 & 0x03) << 6) | ((lsb1 & 0xF8) >> 2));
       
    //msb = resp_adc_data[n][0];
    //lsb2 = resp_adc_data[n][1];
    //lsb1 = resp_adc_data[n][2];
//    msb = (resp_adc_data_i32[n] >> 16) & 0xFF;
//    lsb2 = (resp_adc_data_i32[n] >> 8) & 0xFF;
//    lsb1 = (resp_adc_data_i32[n]) & 0xFF;
    msb = (resp_adc_data_i32[n] >> 8) & 0xFF;
    lsb2 = (resp_adc_data_i32[n]) & 0xFF;
    lsb1 = 0x0;

    //msb = (foobar >> 16) & 0xFF;
    //lsb2 = (foobar >> 8) & 0xFF;
    //lsb1 = (foobar) & 0xFF;
    buffer[i++] = (msb & 0xFE);
    buffer[i++] = (((msb & 0x01) << 7) | ((lsb2 & 0xFC) >> 1));
    buffer[i++] = (((lsb2 & 0x03) << 6) | ((lsb1 & 0xF8) >> 2));

    
    // battery voltage
//    static int ff = 0;
//    if (ff < 10) {
//        msb = 0;
//        ff++;
//    }
//    else {
        msb = (battery_volt[n] << 1) & 0xFF;
//        ff = 0;
//    }
    lsb2 = 0x0;
    lsb1 = 0x0;
    buffer[i++] = (msb & 0xFE);
    buffer[i++] = (((msb & 0x01) << 7) | ((lsb2 & 0xFC) >> 1));
    buffer[i++] = (((lsb2 & 0x03) << 6) | ((lsb1 & 0xF8) >> 2));
        
  }

  // footer - 4 bytes
  buffer[i++] = 0x12;
  
  buffer[i++] = 0x12;
  //buffer[i++] = battery_volt[0];

  buffer[i++] = 0x00;
  buffer[i++] = 0x00;

  // flush buffer out
#if defined(ENABLE_UPLOAD_DATA)
  for (i=0;i<SENSOR_BUFFER_SIZE;i++) {
#if defined(USE_OLD_PORT_PIN_LAYOUT)
    while (U3STAbits.UTXBF != 0);
    U3TXREG = buffer[i];
#else
    while (U2STAbits.UTXBF != 0);
    U2TXREG = buffer[i];    
#endif
  }
#endif

}

void ctrl_cb(uint32 p)
{
  uint i;

  sys_begin_profile(&sensor_handling_ts);
  //--
  notify_sensors();
  //--
  for (i=0;i<4;i++)
    got_sensor_data[i] = false;
  //--
  get_sensor_data();
  //--

  if (active_sensors) {
    uint i;
    // apply interpolated fix ups for failed sensor data reads
    for (i=0;i<4;i++) {
      if (!got_sensor_data[i]) {
        memcpy(&imu_data[i],&imu_data_history[i],sizeof(IMU_DATA_STATE));
        memcpy(&ppg_adc_data_i32[i],&ppg_adc_data_i32_history[i],sizeof(int32));
        memcpy(&resp_adc_data_i32[i],&resp_adc_data_i32_history[i],sizeof(int32));
        memcpy(&battery_volt[i],&battery_volt_history[i],sizeof(uint8));
      }
    }
    //--
    upload_sensor_data();

    // reset back to known
    memset(imu_data,0,sizeof(IMU_DATA_STATE) * 4);
    memset(ppg_adc_data_i32,0,sizeof(int32) * 4);
    memset(resp_adc_data_i32,0,sizeof(int32) * 4);
    memset(battery_volt,0,sizeof(uint8) * 4);
  }

}

int main()
{

    sys_init_pic32mx_mcu_and_ports();
    net_init_nRF24_ports();

    //LED2 = 1;
    
    net_turn_radio_off();

    // baby unit test of time profiler
    //uint32 ts;
    //sys_begin_profile(&ts);
    //DELAY_MS(50);
    //printf("%f\n",sys_query_profile(ts)); // reports 50001.1

    // initialize to a known state
    memset(imu_data,0,sizeof(IMU_DATA_STATE) * 4);
    memset(imu_data_history,0,sizeof(IMU_DATA_STATE) * 4);
    memset(ppg_adc_data_i32,0,sizeof(int32) * 4);
    memset(ppg_adc_data_i32_history,0,sizeof(int32) * 4);
    memset(resp_adc_data_i32,0,sizeof(int32) * 4);
    memset(resp_adc_data_i32_history,0,sizeof(int32) * 4);
    memset(battery_volt,0,sizeof(uint8) * 4);
    memset(battery_volt_history,0,sizeof(uint8) * 4);

    net_config_nRF24(0,0x01,5,0,0,NRF24_CHANNEL,
      &custom_addrs[0][0],
      &custom_addrs[1][0],
      custom_addrs[2][0],
      custom_addrs[3][0],
      custom_addrs[4][0],
      custom_addrs[5][0],
      &custom_addrs[1][0],
      32);

#if !defined(ENABLE_UPLOAD_DATA)
    net_report_regs();
#endif

    net_clear_all_irqs();
    net_clear_tx_fifo();
    net_clear_rx_fifo();

    net_set_tx_mode(); // put in stdby tx mode

    sys_set_timer1_callback(ctrl_cb);
    sys_enable_timer1(CONTROL_MODULE_TIMING);
    sys_enable_mv_ints(); // multivec ints

    //printf("Starting...\n");

    while(1)
      DELAY_US(1); // Idle() / Sleep() do not exist on Pic32. This doesn't imply that its not possible.
  
    return 0;
}

