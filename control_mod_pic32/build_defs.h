
#define ENABLE_UPLOAD_DATA  // else reporting to stdout - also applies to BT reads as well
#define CLOCK_SPEED_40MHZ // ELSE 80 MHZ -- BECAREFUL WITH SETTING THIS TO 80MHZ ... SIDE EFFECTS NOT FULLY TESTED
#define CLOCK_SPEED_3M_BAUD // for 3M baud BT -- drops clock down to 36 MHZ

#define TIMER1_PR_TWEAK_OFFSET (-1) // small tweak to make the CM signal a lot more accurate.

#define USE_OLD_PORT_PIN_LAYOUT  // for original PIC32 CM
