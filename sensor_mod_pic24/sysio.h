
#ifndef SYSIO_H
#define SYSIO_H

#include "stdinc.h"
#include "stdtypes.h"

#define DELAY_MS(x) __delay32((FCY/1000)*x)
#define DELAY_US(x) __delay32((FCY/1000000)*x)

void sys_handle_power_button();
void sys_init_pic24_ports();

#define ledon()   LED1=1;
#define ledoff()  LED1=0;

#define TOGGLE_LED() LED1=~LED1;

void   sys_init_timer1();

void   sys_init_timer32();

void   sys_start_timer32();
uint32 sys_query_timer32();
uint32 sys_stop_timer32();
void   sys_cancel_timer32();
//void   sys_restart_timer32();

//void   sys_start_timer32_2();
//uint32 sys_query_timer32_2();
//uint32 sys_stop_timer32_2();
// OR
#if defined(ENABLE_EXT_ADC)
void   sys_init_timer_4_inter();
#endif

typedef struct {
  uint8 acc_x_msb;
  uint8 acc_x_lsb;
  uint8 acc_y_msb;
  uint8 acc_y_lsb;
  uint8 acc_z_msb;
  uint8 acc_z_lsb;
  uint8 gyro_x_msb;
  uint8 gyro_x_lsb;
  uint8 gyro_y_msb;
  uint8 gyro_y_lsb;
  uint8 gyro_z_msb;
  uint8 gyro_z_lsb;
  uint8 brd_temp_msb;
  uint8 brd_temp_lsb;
}IMU_DATA_STATE;

#define ACTIVE_IMU_ACC      0x01
#define ACTIVE_IMU_GYRO     0x02
#define ACTIVE_IMU_TEMP     0x04   // board temp
#define ACTIVE_ADC_PPG      0x08
#define ACTIVE_ADC_RESP     0x10
// bits 5-6 are available and reserved
#define UPDATE_IMU_ADC_FLAG 0x80   // NEEDED...

#define ACTIVE_IMU_ANY      (ACTIVE_IMU_ACC|ACTIVE_IMU_GYRO|ACTIVE_IMU_TEMP)
#define ACTIVE_IMU_ALL      ACTIVE_IMU_ANY

#define ACTIVE_ADC_ANY      (ACTIVE_ADC_PPG|ACTIVE_ADC_RESP)
#define ACTIVE_ADC_ALL      ACTIVE_ADC_ANY

#if defined(ENABLE_IMU)
uint8 get_imu_control_state();
void  set_imu_control_state(uint8 imu_cs);
void  sys_get_imu_data(IMU_DATA_STATE * imu_data_out);
#endif

#if defined(ENABLE_EXT_ADC)
void  set_adc_control_state(uint8 adc_cs);

bool  read_interr_ext_adc_samples(int32 * sample1_out, int32 * sample2_out); // returns true when count is reached
bool  are_ext_adc_interrupts_enabled(); // runtime check to see if ext adc interrupts are on

bool  read_ext_adc_samples_normal(int32 * sample1_out, int32 * sample2_out); // called once every 4ms (250/s)

int32  get_t4_interr_counter();
void   reset_t4_interr_counter();
void   adjust_t4_interr_period_register(int incr); // for sync

#endif


#endif  /* SYSIO_H */


