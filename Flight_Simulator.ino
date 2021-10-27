/*                        FLIGHT SIMULATOR PROJECT 
 *                        Authors : Henry Gilbert and Kolby Baas
 *                        Date    : 10/1/2021                        asdf
                                    |
                                    |
                                    |
                                  .-'-.
                                 ' ___ '
                       ---------'  .-.  '---------
       _________________________'  '-'  '_________________________
        ''''''-|---|--/    \==][^',_m_,'^][==/    \--|---|-''''''
                      \    /  ||/   H   \||  \    /
                       '--'   OO   O|O   OO   '--'
 * 
 */ 

// TODO figure out what rotary encoder will do. Right now it has no real purpose. - can be input for lat, lon, heading, etc. 
// TODO figure out what aircraft criteria to add - stall, going too fast, etc. 
// TODO major software config - transmit serial data to instrument panel bus to Arduino NANO, which controlls stepper motors for instruments. 
// TODO instrument design - proof of concept. 
// TODO figure out crash criteria and how to incorporate into main loop - quit executing when crashed/
// TODO figure out how to make actual error codes work and throw error
// TODO add master loop method to stop simulation in the result of an error or a crash 
// TODO add failure condition for a software error and allow it to throw a digital output - fault assert
// TODO add initial conditions? Entry with rotary encoder - not sure yet 
// TODO add way to calculate general equation coefficients for the max rate of change of all thawed measurements. Keep it general with as least hard coding as possible . 
// TODO figure out if passing the entire pointer is the way to go. Might need to to actually update any values, but pulling data FROM should just be the measurement .
// TODO add automated flight controls - hold pitch, hold airspeed, hold altitude, hold heading, etc. 
// TODO add ternary operators for condensed 
// TODO formalize input control panel - need something to do master reset so instruments aren't stuck on power down . 

#include "Globals.h"
#include <LiquidCrystal.h>
 
/* =================== OBJECT  DECLARATIONS ==================== */
LiquidCrystal lcd ( lcd_rs, lcd_e, lcd_d4, lcd_d5, lcd_d6, lcd_d7 );


/* =================== STRUCT  DECLARATIONS ==================== */


// TODO should we include digital inputs here? Service all digital inputs and analog at once? Might be better to break it up. 
typedef struct Frozen_Data_t {
    // Analog Data
    uint16_t yoke_x ;
    uint16_t yoke_y ;
    uint16_t analog_pot ; 

    // Digital Data 
    bool     stick_switch ; // joystick switch
} Frozen_Data;

Frozen_Data flsim_inputs ;

typedef struct Thawed_Data_t {
    // TODO figure out these uits/types, what do they need to be in.
    float ground_speed ;      // knots 
    float true_airspeed ;     // knots
    float altitude ;          // feet 
    float pitch ;             // degrees
    float roll ;              // degrees
    float yaw ;               // degrees   figure out how this works with turn 0 
    float heading ;           // degrees
    float rate_of_climb ;     // feet per second 
    float angle_of_attack ; 
} Thawed_Data ;

Thawed_Data flight_status ;

typedef struct Aircraft_Situation_t {
    bool landing_gear_down_status = false ;   
    bool airborne_status = false ; 
    bool stall_status = false ; 
    bool crash_stats = false; 
    uint16_t fuel_level ; 
    float latitude ; 
    float longitude ; 
    float oat ; 
    float tat ; 
} Aircraft_Situation ; 

Aircraft_Situation aircraft_situation ; 

typedef struct Wind_t {
   
    int vel_x ;
    int vel_y ; 
} Wind ; 

Wind flight_wind ; 

// =================== END STRUCT DECLARATIONS ====================



// =================== FUNCTION DECLARATIONS ====================

/*--------------------------------------------------------------------------------------------
Function: service_frozen_data
Inputs: Pointer for frozen data 
Outputs: Success or fail 
Description: Services digital and analog inputs - stores data in appropriate structs. 
----------------------------------------------------------------------------------------------*/

int32_t service_frozen_data ( Frozen_Data * frozenPtr )
{
    int32_t success = EXIT_FAILURE ;
   
    // Read input values
    uint16_t x_val = analogRead ( js_vrx ) ;
    uint16_t y_val = analogRead ( js_vry ) ;
    uint16_t thr_val = analogRead ( throttle_in ) ;
    bool sw_val = digitalRead ( js_sw ) ;
  
    if (  ( x_val < 0 ||  x_val > MAX_ANALOG_IN_VALUE )  ||      // Joystick X axis failure condition. Can't be outside of 0-1024.
          ( y_val < 0 ||  y_val > MAX_ANALOG_IN_VALUE )  ||      // Joystick Y axis failure condition. Can't be outside of 0-1024.
          ( thr_val < 0 ||  thr_val > MAX_ANALOG_IN_VALUE ) ||   // Pot axis failure condition. Can't be outside of 0-1024.
          ( (sw_val < 0 ) || (sw_val > 1 ) )      // Switch failure condition - can't be anything not zero or one.
       )
    {
        return success ; // Invalid input values - exit function.
    }
    
    success = EXIT_SUCCESS ; // Success
  
    // Store values into pointer struct .
    frozenPtr->yoke_x = x_val ;
    frozenPtr->yoke_y = y_val ;
    frozenPtr->analog_pot = thr_val ;
    frozenPtr->stick_switch = sw_val ;
  
    return success ;
}



/*--------------------------------------------------------------------------------------------
Function: thaw_data
Inputs: frozen data poiner, thawed data pointer
Outputs: success or fail code 
Description: calls each subfunction for portion of thawed data that must be serviced and stores values in struct. 
              Heirarchy of Data - acceleration comes first, then pitch, roll, TODO verify this with governing equations . 
----------------------------------------------------------------------------------------------*/
int32_t thaw_data ( Frozen_Data * frozenPtr ,
                    Thawed_Data * thawedPtr , 
                    Aircraft_Situation * aircraftSituationPtr )
{
    int32_t success = EXIT_FAILURE ;
  
    if ( ( frozenPtr == NULL ) ||
         ( thawedPtr == NULL ) || 
         ( aircraftSituationPtr == NULL ) ) 
    {
        return success ;  // Error Null Pointers 
    }

    // Perform calculations to measurements that can be calculated when grounded . 
    success = thaw_true_airspeed ( frozenPtr , thawedPtr ) ;
    
    // Check to see if aircraft is grounded or airborne 
    success = check_aircraft_situation ( frozenPtr , thawedPtr , aircraftSituationPtr ) ;
    
    // Only perform these calculations if aircraft is airborne 
    if ( aircraftSituationPtr->airborne_status == true ) 
    { 
        // digitalWrite ( airborne_status_out, HIGH ) ; 
        success = thaw_groundspeed ( frozenPtr , thawedPtr ) ;
        success = thaw_pitch ( frozenPtr , thawedPtr ) ;
        success = thaw_roll ( frozenPtr , thawedPtr ) ;
        success = thaw_rate_of_climb ( frozenPtr , thawedPtr ) ;
        success = thaw_altitude ( frozenPtr , thawedPtr ) ;
        success = thaw_heading ( frozenPtr , thawedPtr ) ;
        success = thaw_yaw ( frozenPtr , thawedPtr ) ;
    }

    success = service_aircraft_situation_outputs ( aircraftSituationPtr ) ;
    
    return success ;
}

/*--------------------------------------------------------------------------------------------
Function: check_aircraft_situation
Inputs:
Outputs:
Description:   All subroutines here will be check_XXXX_status 
----------------------------------------------------------------------------------------------*/
int32_t check_aircraft_situation (  Frozen_Data * frozenPtr ,
                                    Thawed_Data * thawedPtr , 
                                    Aircraft_Situation * aircraftSituationPtr ) 
{
    int32_t success = EXIT_FAILURE ;
  
    success = check_grounded_status ( frozenPtr , thawedPtr ,aircraftSituationPtr ) ;
    success = check_stall_status ( thawedPtr , aircraftSituationPtr ) ;
    
    return success ;
}

/*--------------------------------------------------------------------------------------------
Function: check_stall_status
Inputs:
Outputs:
Description: Stall the aircraft is BOTH pMin< pitch < pMAX AND tasMIN < true airspeed < tasMAX 
             TODO this will eventually be calculated with drag coefficient 
----------------------------------------------------------------------------------------------*/
int32_t check_stall_status ( Thawed_Data * thawedPtr , 
                             Aircraft_Situation * aircraftSituationPtr ) 
{
    int32_t success = EXIT_FAILURE ; 

    // Must be airborne TODO need a way to get out of stall status 
    if ( aircraftSituationPtr->airborne_status == true )
    {
        aircraftSituationPtr->stall_status = ( (( thawedPtr->true_airspeed >= STALL_AIRSPEED_MIN ) && (thawedPtr->true_airspeed <= STALL_AIRSPEED_MAX )) && 
                                               (( thawedPtr->pitch >= STALL_PITCH_MIN) && (thawedPtr->pitch <= STALL_PITCH_MAX )) ) ? true : false ;      
    }
    success = EXIT_SUCCESS ; 
    return success ; 
}

/*--------------------------------------------------------------------------------------------
Function: 
Inputs:
Outputs:
Description:
----------------------------------------------------------------------------------------------*/
int32_t check_grounded_status ( Frozen_Data * frozenPtr ,
                                Thawed_Data * thawedPtr ,
                                Aircraft_Situation * aircraftSituationPtr ) 
{
    int32_t success = EXIT_FAILURE ; 
    
    // Check grounded status for takeoff 
    if ( ( thawedPtr->true_airspeed >= LIFTOFF_AIRSPEED ) && 
         ( frozenPtr->yoke_x > LIFTOFF_YOKE )  
       )
     {
        aircraftSituationPtr->airborne_status = true ; 
     } 

    // Check groundedd status while airborne for landing 
    if ( aircraftSituationPtr->airborne_status == true )
    {
      //  aircraftSituationPtr->airborne_status = ( thawedPtr->altitude <= 0 ) ? true : false ;     // Todo eventually we will have alt set and this will be calculated .        
    }

    success = EXIT_SUCCESS ; 
    return success ;
}

/*--------------------------------------------------------------------------------------------
Function: 
Inputs:
Outputs:
Description:
----------------------------------------------------------------------------------------------*/
int32_t check_crash_stats (Thawed_Data * thawedPtr ,
                           Aircraft_Situation * aircraftSituationPtr) 
{
    /* crash criteria: landing without landing gear down, landing at  too low of a pitch , landing too fast , landing sideways , trying to land too slow , 
     * 
     */
    int32_t success = EXIT_FAILURE ; 

    // Landing at too low of a pitch 

    

    success = EXIT_SUCCESS ; 
    return success ; 
  
}

/*--------------------------------------------------------------------------------------------
Function: 
Inputs:
Outputs:
Description:
----------------------------------------------------------------------------------------------*/
int32_t service_aircraft_situation_outputs ( Aircraft_Situation * aircraftSituationPtr  ) 
{
    int32_t success = EXIT_FAILURE ; 
    
    digitalWrite ( airborne_status_out , aircraftSituationPtr -> airborne_status  ) ; 
    digitalWrite ( stall_warning_out , aircraftSituationPtr -> stall_status ) ; 
    
    success = EXIT_SUCCESS ; 
    return success ; 
}



/*--------------------------------------------------------------------------------------------
Function: thaw_true_airspeed 
Inputs: frozen data pointer , thawed data pointer 
Outputs: success or error code 
Description: calculates the new airspeed based on throttle input. Since there is always a constant decelleration, 
                 the formula for airspeed is : 
                 V = airspeed
                 T = period = 50ms (config data)
                 D = Decelleration - constant (think of cars, as they are rolling there is a constant applied decelleration. 
                                     A car will stop moving if you quit applying acceleration
                 P = potentiometer value - 0-1024
                 A = alpha coefficient for converting pot value into standard offset 
                 Vnew = Vold + T(AP-D)     OR dV = T(AP-D) . 
                 The max change in velocity should be 10 knots per second (this must be fine tuned). This is per SECOND.
                 Per calculation, aka per 50 ms, dV max = 10/20 = 0.5. D=5/20 = .25
                        .5 = (A*P-.25). 
                        .75 = A*P. At max accel, P = 1024. So A, the offset value, equals 0.00073. 
                 At max throttle, dV = +10 kts/second
                 At min throttle, dV = -5  kts/second
----------------------------------------------------------------------------------------------*/
int32_t thaw_true_airspeed (Frozen_Data * frozenPtr,  Thawed_Data * thawedPtr )
{
    int32_t success = EXIT_FAILURE ;

    if (  ( frozenPtr == NULL ) ||
          ( thawedPtr == NULL ) ) 
    {
        return success ; // Error - can't have null pointers. TODO necessary because of initial check ? TODO 
    }

    // TODO make these global variables? Are they going to be used elsewhere? Also, possibly make these the general case calculations 
    float new_airspeed ; 
    float old_airspeed = thawedPtr->true_airspeed;
    float A = 0.00073f;
    float D = .25f;
    
    new_airspeed = old_airspeed + (A * frozenPtr->analog_pot - D);   // TODO update this. I don't like adding things directly to a struct if the values aren't right initially 
    
    // Limit the airspeed between the min 0 and the max 176. 
    // Todo make these inline conditional if with ternary operator 
    if ( new_airspeed > MAX_AIRSPEED ) new_airspeed = MAX_AIRSPEED ; 
    if  (new_airspeed < 0 ) new_airspeed = 0 ; 

    thawedPtr->true_airspeed  = new_airspeed ; 
    success = EXIT_SUCCESS ; // Exit success 
    return success ;
}


/*---------------------------------------------------------------------------------------
Function: thaw_groundspeed  
Inputs: frozen data pointer , thawed data pointer 
Outputs: error code 
Description:  calculates groundspeed as a function of airspeed - cos of pitch X true airspeed 
----------------------------------------------------------------------------------------------*/
int32_t thaw_groundspeed ( Frozen_Data * frozenPtr,  Thawed_Data * thawedPtr ) 
{
    int32_t success = EXIT_FAILURE ;
    
    if (  ( frozenPtr == NULL ) ||
          ( thawedPtr == NULL ) ) 
    {
        return success ; // Error - can't have null pointers. TODO necessary because of initial check ? TODO 
    }

    float new_groundspeed = thawedPtr->true_airspeed * cos ( radians (thawedPtr->pitch ) ) ;
    thawedPtr->ground_speed = new_groundspeed ; 

    success = EXIT_SUCCESS ; // Exit success 
    
    return success ;
}



/*---------------------------------------------------------------------------------------
Function: thaw_pitch 
Inputs: frozen data pointer , thawed data pointer 
Outputs: error code 
Description:  Thaws pitch based on true airspeed and grounded status . Max rate of change of pitch is +-2 degrees per second, or .1 degree per calculation (20 calc/second)
              dP = A * (yoke_reading-528 )    Yoke reading-528 is necessary to offset the joystick initial value
              At max dP,  2  = A * (1024 - 528 ) A equals 0.0002020.  
----------------------------------------------------------------------------------------------*/
int32_t thaw_pitch ( Frozen_Data * frozenPtr,  Thawed_Data * thawedPtr  )
{
    int32_t success = EXIT_FAILURE ;

    double new_pitch ; 
    double old_pitch = thawedPtr->pitch ;
    float A = 0.0002020f ; // TODO remove magic numbers , 
    int16_t yoke_reading = frozenPtr->yoke_x ; 
    
    new_pitch = old_pitch + (A * (yoke_reading - 528) );   // TODO update this. I don't like adding things directly to a struct if the values aren't right initially, also magic number. 

    if ( new_pitch > MAX_PITCH ) new_pitch = MAX_PITCH ; 
    if ( new_pitch < MIN_PITCH ) new_pitch = MIN_PITCH ; 

    thawedPtr->pitch  = new_pitch ; 
    
    success = EXIT_SUCCESS ; // Exit success 
    return success ;
}

/*---------------------------------------------------------------------------------------
Function: thaw_roll 
Inputs: frozen data pointer , thawed data pointer 
Outputs: error code 
Description:  Max roll delta is 2 degrees per second . 
----------------------------------------------------------------------------------------------*/
// NOTE : positive roll means, as a pilot, turning clockwise 
int32_t thaw_roll ( Frozen_Data * frozenPtr,  Thawed_Data * thawedPtr )
{
    int32_t success = EXIT_FAILURE ;
    double new_roll ; 
    double old_roll = thawedPtr->roll ;
    float A = -0.0004f ;
    int16_t yoke_reading = frozenPtr->yoke_y ; 
    
    new_roll = old_roll + (A * (yoke_reading - 512) );   // TODO update this. I don't like adding things directly to a struct if the values aren't right initially 

    if ( new_roll > MAX_PITCH ) new_roll = MAX_ROLL ; 
    if ( new_roll < MIN_PITCH ) new_roll = MIN_ROLL ; 

    thawedPtr->roll  = new_roll ; 
    
    success = EXIT_SUCCESS ; // Exit success 
    return success ;
}


/* function - thaw_altitude 
   input - frozen pointer, thawed pointer 
   return - exit status code 
   Description - altitude is only calculated when not grounded. Altitude is a function of time - 
                        alt = alt_old + sin ( true_airspeed ) - needs to be time compensated . 
                        At max pitch and max airspeed, dAltMAX = 36 feet per second, or 1.83 feet per calculation  
*/

/*--------------------------------------------------------------------------------------------
Function: thaw_altitde 
Inputs: frozen poiner , thawed pointer 
Outputs: exit status code 
Description:
----------------------------------------------------------------------------------------------*/
// TODO throw warning if altitude becomes too high - figure out soft limit and critical limit 
int32_t thaw_altitude ( Frozen_Data * frozenPtr,  Thawed_Data * thawedPtr )
{
    int32_t success = EXIT_FAILURE ;

    float new_altitude ; 
    float old_altitude = thawedPtr->altitude ; 
    
    new_altitude = old_altitude + ( thawedPtr->true_airspeed * sin ( radians ( thawedPtr ->pitch ) ) )/20 ;

    // Serial.println(new_altitude - old_altitude);
    thawedPtr->altitude = new_altitude ; 

    success = EXIT_SUCCESS ;
    return success ;
}

/*--------------------------------------------------------------------------------------------
Function: thaw_rate_of_climb 
Inputs:
Outputs:
Description:
----------------------------------------------------------------------------------------------*/
int32_t thaw_rate_of_climb (Frozen_Data * frozenPtr,  Thawed_Data * thawedPtr ) 
{
    int32_t success = EXIT_FAILURE ;
    thawedPtr->rate_of_climb = thawedPtr->true_airspeed * sin( radians ( thawedPtr -> pitch ) ) ; 
    success = EXIT_SUCCESS ; 
    return success ; 
  
}

/*--------------------------------------------------------------------------------------------
Function: 
Inputs:
Outputs:
Description:
----------------------------------------------------------------------------------------------*/

int32_t thaw_heading ( Frozen_Data * frozenPtr,  Thawed_Data * thawedPtr )
{
    int32_t success = EXIT_FAILURE ;

    success = EXIT_SUCCESS ; 
    return success ;
}


/*--------------------------------------------------------------------------------------------
Function:  thaw_yaw
Inputs:
Outputs:
Description:
----------------------------------------------------------------------------------------------*/

int32_t thaw_yaw ( Frozen_Data * frozenPtr,  Thawed_Data * thawedPtr )
{
    int32_t success = EXIT_FAILURE ;

    success = EXIT_SUCCESS ; 
    return success ;
}




/*--------------------------------------------------------------------------------------------
Function: service_rotary_encoder
Inputs: Pointer to global counter variable 
Outputs: Failure code / status 
Description: Increments/decrements the counter based on the turn conditions of the encoder. 
----------------------------------------------------------------------------------------------*/
int32_t service_rotary_encoder ( byte * counter  ) {
    
}

// ==================== END FUNCTION DECLARATIONS ==================







// ==================== DEBUG FUNCTIONS ======================

/*
*   Function : read_frozen_struct 
*   Description : debug function - prints data 
*/
void read_frozen_struct ( Frozen_Data * frozenPtr ) 
{
    Serial.print ( "Yoke X: ") ;
    Serial.print ( frozenPtr->yoke_x ) ;
    Serial.print ( "    Yoke Y: ") ;
    Serial.print ( frozenPtr->yoke_y ) ;
    Serial.print ( "    Throttle In : ") ;
    Serial.print ( frozenPtr->analog_pot ) ;
    Serial.print ( "\n ") ;
}

void print_raw_inputs () 
{
    // Print the analog input potentiometer/throttle value
    Serial.print ( "Analog Input Pot : " ) ;
    Serial.print ( analogRead( throttle_in ) ) ;
  
    // Print the x input of the joy stick
    Serial.print( "    Joystick X :" ) ;
    Serial.print ( analogRead ( js_vrx ) ) ;
  
    // Print the y input of the joy stick
    Serial.print( "    Joystick Y :" ) ;
    Serial.print ( analogRead ( js_vry ) ) ;
  
    // Print the switch value of the joy stick
    Serial.print( "    Joystick SW :" ) ;
    Serial.print ( digitalRead ( js_sw ) ) ;
  
    // Rotary Encoder Inputs
    Serial.print ( "   Rotary CLK:  " ) ;
    Serial.print ( digitalRead ( rotEnc_clk ) ) ;
  
    Serial.print ( "   Rotary DT:   " ) ;
    Serial.print ( digitalRead ( rotEnc_dt ) ) ;
  
    Serial.print ( "\n " ) ;
}

void print_thawed_data ( Frozen_Data * frozenPtr  , Thawed_Data * thawedPtr,Aircraft_Situation * aircraftSituationPtr  ) 
{
    Serial.print ( "Airspeed : ") ; 
    Serial.print ( thawedPtr->true_airspeed ) ;
    Serial.print ( "    GS :" ) ; 
    Serial.print ( thawedPtr->ground_speed ) ;
    Serial.print ( "    ROC :" ) ;
    Serial.print ( thawedPtr->rate_of_climb ) ;
    Serial.print ( "    Pitch :" ) ;
    Serial.print ( thawedPtr->pitch ) ;
    Serial.print ( "    Roll :" ) ; 
    Serial.print ( thawedPtr->roll ) ;
    Serial.print ( "    Alt :" ) ; 
    Serial.print ( thawedPtr->altitude ) ;
    Serial.print ( "\n" ) ; 
}


// ==================== END DEBUG FUNCTIONS ======================









// =================== MAIN FUNCTIONS  ====================


// Timer Variables // 
unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 50;  //the value is a number of milliseconds
int32_t error_code ; 
byte time_diff ; 
byte rotEnc_count ; 

#define NUM_ROWS_LCD 2
#define NUM_COLUMNS_LCD 16 

void setup() {
    // set up the LCD's number of columns and rows:
    Serial.begin( 74880 );
    lcd.begin( NUM_COLUMNS_LCD, NUM_ROWS_LCD );

    // Joy stick inputs 
    pinMode( js_vrx, INPUT );
    pinMode( js_vry, INPUT );
    pinMode( js_sw , INPUT );

    // Status Lights 
    pinMode ( crash_status_out , OUTPUT ) ; 
    pinMode ( airborne_status_out , OUTPUT ) ; 
    pinMode ( stall_warning_out , OUTPUT ) ; 
    
    // Rotary Encoder inputs 
    pinMode( rotEnc_dt, INPUT_PULLUP );
    pinMode( rotEnc_clk, INPUT_PULLUP );
    pinMode ( rotEnc_sw, INPUT_PULLUP ) ;

    // Rotary Encoder interrupt routines
    attachInterrupt( digitalPinToInterrupt( rotEnc_dt ) , service_rotary_encoder , CHANGE );
    attachInterrupt( digitalPinToInterrupt( rotEnc_clk ) , service_rotary_encoder , CHANGE );

    /* Pin Mode for Clock */
    pinMode( clock_status_out, OUTPUT );
 
    startMillis = millis();
}

void loop() {    
    currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
    if ( currentMillis - startMillis >= period )  //test whether the period has elapsed
    {
        lcd.setCursor(0,0);
        time_diff  = currentMillis - startMillis ;
        Serial.print ( time_diff ) ; 
        Serial.print ( ":   " ) ;
        
        // ================= BEGIN MAIN EXECUTION CODE  ====================== 
        digitalWrite(clock_status_out, !digitalRead(clock_status_out));  // Display the state of the timer - leave for debug purposes 
        error_code = service_frozen_data ( &flsim_inputs) ;
        // read_frozen_struct ( &flsim_inputs ) ;                      // Debug function - not critical to operation 
        print_thawed_data ( &flsim_inputs , &flight_status ,&aircraft_situation ); 
        error_code = thaw_data ( &flsim_inputs , &flight_status , &aircraft_situation) ;
        lcd.print( error_code );

   
        // =================   END MAIN EXECUTION CODE ======================
        startMillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.
    }
}


// =================== END MAIN FUNCTIONS  ====================
