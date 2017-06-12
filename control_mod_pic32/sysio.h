
#ifndef SYSIO_H
#define SYSIO_H

#include "stdtypes.h"

#define DELAY_MS(x) __delay_us((x)*1000)
#define DELAY_US(x) __delay_us((x))

void sys_init_pic32mx_mcu_and_ports();

void __delay_us(uint32 delay_in_microsecs);

typedef void (*TCallback)(uint32 p);

void sys_set_timer1_callback(TCallback cb);
void sys_enable_timer1(double interval_us); // 0-200k/400k us max, depending on clock speed
void sys_enable_mv_ints();

void   sys_begin_profile(uint32 * ts);
double sys_query_profile(uint32 ts); // returns number of microsecs passed since ts

bool  sys_rx_uart_has_data();
uint8 sys_get_rx_uart_byte();

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

// mostly for easy (and proper) formation of messages to send to Sensors. It's the inverse on the sensor end (obviously).
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

//------------------------

#define CMD_SET_RESP    0x01
#define CMD_SET_PPG     0x02
#define CMD_SET_ACCEL   0x04
#define CMD_SET_GYRO    0x08
#define CMD_SET_TEMP    0x10  // board temp

#endif

