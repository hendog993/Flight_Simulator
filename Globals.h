#include <Arduino.h>
#include <stdint.h>

#define EXIT_SUCCESS 0 
#define EXIT_FAILURE 1
#define MAX_ANALOG_IN_VALUE 1024 

/* LCD Configuration Data */ 
#define NUM_ROWS_LCD 2
#define NUM_COLUMNS_LCD 16 

/*      Interface Control
     ========= Analog Pins =========
     pin A0 - Potentionmeter Input
     pin A1 - Joystick Input VRx
     pin A2 - Joystick Input VRy
  
     ========= Digital Pins =========
     pin  2 - Rotary Encoder DT      !! Interrupt Driven Pin
     pin  3 - Rotary Encoder CLK     !! Interrupt Driven Pin
     pin  4 - Joy Stick SW
     pin  5 - Crash status 
     pin  6 - Grounded/airborne status 
     ping 7 - Stall Warning 
   
     pin 11 - Rotary Encoder SW
     pin 12 - Status Light
     pin 22 - LCD RS
     pin 23 - LCD E
     pin 24 - LCD D4
     pin 25 - LCD D5
     pin 26 - LCD D6
     pin 27 - LCD D7
*/


/* =================== PIN DECLARATIONS ==================== */

// LCD Pins
#define lcd_rs 22
#define lcd_e  23
#define lcd_d4 24
#define lcd_d5 25
#define lcd_d6 26
#define lcd_d7 27

/* Pot Pin Input */
#define throttle_in A0

/* Rotary Encoder Pins - all pins are interrupt capable */
#define rotEnc_dt 2
#define rotEnc_clk 3
#define rotEnc_sw 18 

/* Joy Stick Pins */
#define js_vrx A1
#define js_vry A2
#define js_sw  4

/* Digital Output */
#define clock_status_out 12
#define crash_status_out 5 
#define airborne_status_out 6
#define stall_warning_out 7 
#define landing_gear_out NULL 
#define error_assert NULL
#define flight_warning_out NULL 
 
/* Digital Inputs */
#define landing_gear_up 0

// =================== END PIN DECLARATIONS ====================

/* Air Data Declarations */
#define MAX_AIRSPEED 176      // knots
#define MAX_PITCH_DELTA 2     // degrees per second 
#define MAX_ROLL_DELTA  3     // degrees per second
#define MAX_RATE_OF_CLIMB 721  // feet per minute 
#define MAX_ALTITUDE    14000  // feet      
#define LIFTOFF_AIRSPEED 60    // knots 
#define LIFTOFF_YOKE_THRESHOLD 650 
#define MAX_PITCH 12
#define MIN_PITCH -12
#define MAX_ROLL 12 
#define MIN_ROLL -12

/* Stall Criteria */
#define STALL_AIRSPEED_MIN 40
#define STALL_AIRSPEED_MAX 70
#define STALL_PITCH_MIN 8
#define STALL_PITCH_MAX 10

const byte period = 50;  //the value is a number of milliseconds inbetween calculations 
const uint16_t num_milliseconds_in_second = 1000 ; 
const byte frequency = num_milliseconds_in_second/period ; 

/* Yoke Offset Values specific to hardware. Must configure this individually */ 
#define YOKE_X_READING_RTZ_OFFSET 528
#define YOKE_Y_READING_RTZ_OFFSET 512



 
