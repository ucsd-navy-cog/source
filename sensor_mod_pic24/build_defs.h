 
//#define ENABLE_4_MHZ       // else 8 MHZ

#define ENABLE_IMU

#define ENABLE_EXT_ADC
// There is no linear ramp for the freq of these interrupts...as prior tests have shown - no magical formula is possible for variable adc inter freqs
// Don't set to more than 16000 - since a 16-bit timer is used.
#if defined(ENABLE_4_MHZ)
#define BASE_TIMER4_INT_FREQ_HZ        (4000) // WARNING: should be multiples of 250 !!!!
#else
#define BASE_TIMER4_INT_FREQ_HZ        (6000) // WARNING: should be multiples of 250 !!!!
#endif
#define EXPECT_CM_INTERV               (250)   // expected interval per second by CM
#define TIMER4_DRIFT_TWEAK_VAL         (0) //(-2)   // in lieu of 1000000 / 12000 * 4 == 333.333 (333) - this small discrepancy fix is too small for PIC24 to track...

//#define SPOOF_IMU_DATA
//#define SPOOF_ADC_DATA

// EXPERIMENTAL BEGIN
//#define PPG_ONLY_EXP  // 250/s e.g. RESP support crippled

#if defined(PPG_ONLY_EXP)
#define PPG_ONLY_EXP_OPTION_LEFT
/*
  IF PPG_ONLY_EXP_OPTION_LEFT
    Toggle GPIO
    delay 100 us
    get A/D sample - save in tempvar1
    Toggle GPIO
    delay 100 us
    get A/D sample - save in tempvar2
    Toggle GPIO
    delay 100 us
    get A/D sample - save in tempvar3
    send tempvar2-tempvar1 to channel 7
    send tempvar3-tempvar1 to channel 8 (last)
  ELSE // PPG_ONLY_EXP_OPTION_LEFT not defined
    default temp_phase_var to 0
    if temp_phase_var == 0
      get A/D sample - save in tempvar
      send tempvar to channel 7
      set temp_phase_var to 1
      break
    if temp_phase_var == 1
      get A/D sample - save in tempvar
      send tempvar to channel 8
      set temp_phase_var to 0
      break
    Toggle GPIO
*/

#endif
// EXPERIMENTAL END

#define SENSOR_MODULE           (0)       // rebuild for each target sensor. valid: 0-3

