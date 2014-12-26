/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduPlane V2.78"
/*
   Lead developer: Andrew Tridgell
 
   Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Amilcar Lucas, Gregory Fletcher, Paul Riseborough, Brandon Jones, Jon Challinger
   Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier, Yury MonZon

   Please contribute your ideas! See http://dev.ardupilot.com for details

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdio.h>
#include <stdarg.h>

// Common dependencies
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Menu.h>
#include <AP_Param.h>
// AP_HAL
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>

// Application dependencies
#include <GCS_MAVLink.h>        // MAVLink GCS definitions
#include <AP_GPS.h>             // ArduPilot GPS library
#include <DataFlash.h>          // ArduPilot Mega Flash Memory Library
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>        	// ArduPilot barometer library
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_AHRS.h>         	// ArduPilot Mega DCM Library
#include <RC_Channel.h>         // RC Channel Library
#include <AP_RangeFinder.h>     // Range finder library
#include <Filter.h>             // Filter library
#include <AP_Buffer.h>          // APM FIFO Buffer
#include <AP_Relay.h>           // APM relay
#include <AP_ServoRelayEvents.h>
#include <AP_Camera.h>          // Photo or video camera
#include <AP_Mount.h>           // Camera/Antenna mount
#include <AP_Airspeed.h>        // needed for AHRS build
#include <AP_Vehicle.h>         // needed for AHRS build
#include <AP_Declination.h>     // ArduPilot Mega Declination Helper Library
#include <SITL.h>               // software in the loop support
#include <AP_Scheduler.h>       // main loop scheduler
#include <AP_RCMapper.h>        // RC input mapping library
#include <AP_Notify.h>          // Notify library
#include <AP_BattMonitor.h>     // Battery monitor library
#include <AP_BoardConfig.h>     // board configuration library
#if EPM_ENABLED == ENABLED
#include <AP_EPM.h>				// EPM cargo gripper stuff
#endif

#include <AP_Navigation.h>		// Plane
#include <AP_L1_Control.h>		// Plane
#include <AP_SpdHgtControl.h>	// Plane
#include <AP_TECS.h>			// Plane
#include <AP_Arming.h>			// Plane

// AP_HAL to Arduino compatibility layer
#include "compat.h"

// Configuration
#include "defines.h"
#include "config.h"

// Local modules
#include "Parameters.h"
#include "GCS.h"
#include <APM_OBC.h>			// Plane
#include <APM_Control.h>		// Plane

static bool isplane = true;

// key aircraft parameters passed to multiple libraries
static AP_Vehicle::FixedWing aparm;	// Plane

////////////////////////////////////////////////////////////////////////////////
// cliSerial
////////////////////////////////////////////////////////////////////////////////
// cliSerial isn't strictly necessary - it is an alias for hal.console. It may
// be deprecated in favor of hal.console in later releases.
static AP_HAL::BetterStream* cliSerial;

// N.B. we need to keep a static declaration which isn't guarded by macros
// at the top to cooperate with the prototype mangler.

////////////////////////////////////////////////////////////////////////////////
// AP_HAL instance
////////////////////////////////////////////////////////////////////////////////

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

////////////////////////////////////////////////////////////////////////////////
// Parameters
////////////////////////////////////////////////////////////////////////////////
//
// Global parameters are all contained within the 'g' class.
//
static Parameters g;

// main loop scheduler
static AP_Scheduler scheduler;

// AP_Notify instance
static AP_Notify notify;

// primary control channels
static RC_Channel *channel_roll;		// Plane
static RC_Channel *channel_pitch;		// Plane
static RC_Channel *channel_throttle;	// Plane
static RC_Channel *channel_rudder;		// Plane 

////////////////////////////////////////////////////////////////////////////////
// prototypes
////////////////////////////////////////////////////////////////////////////////
static void update_events(void);
static void print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode);
static void plane_gcs_send_text_fmt(const prog_char_t *fmt, ...);

////////////////////////////////////////////////////////////////////////////////
// Dataflash
////////////////////////////////////////////////////////////////////////////////
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
static DataFlash_APM2 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
static DataFlash_APM1 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
static DataFlash_File DataFlash("logs");
//static DataFlash_SITL DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4
static DataFlash_File DataFlash("/fs/microsd/APM/LOGS");
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
static DataFlash_File DataFlash("logs");
#else
static DataFlash_Empty DataFlash;
#endif

////////////////////////////////////////////////////////////////////////////////
// the rate we run the main loop at
////////////////////////////////////////////////////////////////////////////////
static const AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_50HZ;

// has a log download started?
static bool in_log_download;			// Plane

// scaled roll limit based on pitch
static int32_t roll_limit_cd;			// Plane
static int32_t pitch_limit_min_cd;		// Plane

////////////////////////////////////////////////////////////////////////////////
// Sensors
////////////////////////////////////////////////////////////////////////////////
//
// There are three basic options related to flight sensor selection.
//
// - Normal flight mode. Real sensors are used.
// - HIL Attitude mode. Most sensors are disabled, as the HIL
//   protocol supplies attitude information directly.
// - HIL Sensors mode. Synthetic sensors are configured that
//   supply data from the simulation.
//

// All GPS access should be through this pointer.
static GPS         *g_gps;

// flight modes convenience array
static AP_Int8 *plane_flight_modes = &g.plane_flight_mode1;

#if HIL_MODE == HIL_MODE_DISABLED

 #if CONFIG_ADC == ENABLED
static AP_ADC_ADS7844 adc;
 #endif

 #if CONFIG_IMU_TYPE == CONFIG_IMU_MPU6000
static AP_InertialSensor_MPU6000 ins;
#elif CONFIG_IMU_TYPE == CONFIG_IMU_OILPAN
static AP_InertialSensor_Oilpan ins(&adc);
#elif CONFIG_IMU_TYPE == CONFIG_IMU_SITL
static AP_InertialSensor_HIL ins;
#elif CONFIG_IMU_TYPE == CONFIG_IMU_PX4
static AP_InertialSensor_PX4 ins;
#elif CONFIG_IMU_TYPE == CONFIG_IMU_FLYMAPLE
AP_InertialSensor_Flymaple ins;
#elif CONFIG_IMU_TYPE == CONFIG_IMU_L3G4200D
AP_InertialSensor_L3G4200D ins;
#endif

 #if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
 // When building for SITL we use the HIL barometer and compass drivers
static AP_Baro_HIL barometer;
static AP_Compass_HIL compass;
static SITL sitl;
 #else
// Otherwise, instantiate a real barometer and compass driver
  #if CONFIG_BARO == AP_BARO_BMP085
static AP_Baro_BMP085 barometer;
  #elif CONFIG_BARO == AP_BARO_PX4
static AP_Baro_PX4 barometer;
  #elif CONFIG_BARO == AP_BARO_MS5611
   #if CONFIG_MS5611_SERIAL == AP_BARO_MS5611_SPI
static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::spi);
   #elif CONFIG_MS5611_SERIAL == AP_BARO_MS5611_I2C
static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::i2c);
   #else
    #error Unrecognized CONFIG_MS5611_SERIAL setting.
   #endif
  #endif

 #if CONFIG_HAL_BOARD == HAL_BOARD_PX4
static AP_Compass_PX4 compass;
 #else
static AP_Compass_HMC5843 compass;
 #endif
 #endif

// real GPS selection
 #if   GPS_PROTOCOL == GPS_PROTOCOL_AUTO
AP_GPS_Auto     g_gps_driver(&g_gps);

 #elif GPS_PROTOCOL == GPS_PROTOCOL_NMEA
AP_GPS_NMEA     g_gps_driver;

 #elif GPS_PROTOCOL == GPS_PROTOCOL_SIRF
AP_GPS_SIRF     g_gps_driver;

 #elif GPS_PROTOCOL == GPS_PROTOCOL_UBLOX
AP_GPS_UBLOX    g_gps_driver;

 #elif GPS_PROTOCOL == GPS_PROTOCOL_MTK
AP_GPS_MTK      g_gps_driver;

 #elif GPS_PROTOCOL == GPS_PROTOCOL_MTK19
AP_GPS_MTK19    g_gps_driver;

 #elif GPS_PROTOCOL == GPS_PROTOCOL_NONE
AP_GPS_None     g_gps_driver;

 #else
  #error Unrecognised GPS_PROTOCOL setting.
 #endif // GPS PROTOCOL

static AP_AHRS_DCM ahrs(ins, g_gps);

#elif HIL_MODE == HIL_MODE_SENSORS
// sensor emulators
static AP_ADC_HIL              adc;
static AP_Baro_HIL      barometer;
static AP_Compass_HIL          compass;
static AP_GPS_HIL              g_gps_driver;
static AP_InertialSensor_HIL   ins;
static AP_AHRS_DCM             ahrs(ins, g_gps);


 #if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
  // When building for SITL we use the HIL barometer and compass drivers
static SITL sitl;
#endif

#elif HIL_MODE == HIL_MODE_ATTITUDE
static AP_ADC_HIL              adc;
static AP_InertialSensor_HIL   ins;
static AP_AHRS_HIL             ahrs(ins, g_gps);
static AP_GPS_HIL              g_gps_driver;
static AP_Compass_HIL          compass;                  // never used
static AP_Baro_HIL      barometer;

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
 // When building for SITL we use the HIL barometer and compass drivers
static SITL sitl;
#endif

#else
 #error Unrecognised HIL_MODE setting.
#endif // HIL MODE

static AP_L1_Control L1_controller(ahrs);				// Plane
static AP_TECS TECS_controller(ahrs, aparm);			// Plane

// Attitude to servo controllers
static AP_RollController  rollController(ahrs, aparm);	// Plane
static AP_PitchController pitchController(ahrs, aparm);	// Plane
static AP_YawController   yawController(ahrs, aparm);	// Plane
static AP_SteerController steerController(ahrs);		// Plane


// Training mode
static bool training_manual_roll;						// Plane
static bool training_manual_pitch;						// Plane

// should throttle be pass-thru in guided?
static bool guided_throttle_passthru;					// Plane

////////////////////////////////////////////////////////////////////////////////
// GCS selection
////////////////////////////////////////////////////////////////////////////////
static const uint8_t num_gcs = MAVLINK_COMM_NUM_BUFFERS;
static GCS_MAVLINK gcs[MAVLINK_COMM_NUM_BUFFERS];

////////////////////////////////////////////////////////////////////////////////
// User variables
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////

/* Radio values
 *               Channel assignments
 *                       1   Ailerons
 *                       2   Elevator
 *                       3   Throttle
 *                       4   Rudder
 *                       5   Aux5
 *                       6   Aux6
 *                       7   Aux7
 *                       8   Aux8/Mode
 *               Each Aux channel can be configured to have any of the available auxiliary functions assigned to it.
 *               See libraries/RC_Channel/RC_Channel_aux.h for more information
 */

//Documentation of GLobals:
static union {
    struct {
        uint8_t home_is_set         : 1; // 0
        uint8_t simple_mode         : 2; // 1,2 // This is the state of simple mode : 0 = disabled ; 1 = SIMPLE ; 2 = SUPERSIMPLE

        uint8_t pre_arm_rc_check    : 1; // 3   // true if rc input pre-arm checks have been completed successfully
        uint8_t pre_arm_check       : 1; // 4   // true if all pre-arm checks (rc, accel calibration, gps lock) have been performed
        uint8_t auto_armed          : 1; // 5   // stops auto missions from beginning until throttle is raised
        uint8_t logging_started     : 1; // 6   // true if dataflash logging has started

        uint8_t do_flip             : 1; // 7   // Used to enable flip code
        uint8_t takeoff_complete    : 1; // 8
        uint8_t land_complete       : 1; // 9   // true if we have detected a landing

        uint8_t new_radio_frame     : 1; // 10      // Set true if we have new PWM data to act on from the Radio
        uint8_t CH7_flag            : 2; // 11,12   // ch7 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t CH8_flag            : 2; // 13,14   // ch8 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t usb_connected       : 1; // 15      // true if APM is powered from USB connection
        uint8_t yaw_stopped         : 1; // 16      // Used to manage the Yaw hold capabilities

        uint8_t disable_stab_rate_limit : 1; // 17  // disables limits rate request from the stability controller

        uint8_t rc_receiver_present : 1; // 18  // true if we have an rc receiver present (i.e. if we've ever received an update
    };
    uint32_t value;
} ap;

////////////////////////////////////////////////////////////////////////////////
// Radio
////////////////////////////////////////////////////////////////////////////////
// This is the state of the flight control system
// There are multiple states defined such as PLANE_MANUAL, FBW-A, PLANE_AUTO
static enum FlightMode plane_control_mode  = PLANE_INITIALISING;

// Used to maintain the state of the previous control switch position
// This is set to 254 when we need to re-read the switch
static uint8_t oldSwitchPosition = 254;
static RCMapper rcmap;

// board specific config
static AP_BoardConfig BoardConfig;

// receiver RSSI
static uint8_t receiver_rssi;

// This is used to enable the inverted flight feature
static bool inverted_flight     = false;	// Plane

static struct {								// Plane
    // These are trim values used for elevon control
    // For elevons radio_in[CH_ROLL] and radio_in[CH_PITCH] are
    // equivalent aileron and elevator, not left and right elevon
    uint16_t trim1;
    uint16_t trim2;
    // These are used in the calculation of elevon1_trim and elevon2_trim
    uint16_t ch1_temp;
    uint16_t ch2_temp;
} elevon = {								// Plane
	trim1 : 1500,
    trim2 : 1500,
    ch1_temp : 1500,
    ch2_temp : 1500
};											// END Plane

////////////////////////////////////////////////////////////////////////////////
// Failsafe
////////////////////////////////////////////////////////////////////////////////
// Plane failsafe
static struct {
    uint8_t rc_override_active:1; 	// A flag if GCS joystick control is in use
    uint8_t ch3_failsafe:1; 		// Used to track if the value on channel 3 (throtttle) has fallen below the failsafe threshold
    uint8_t saved_mode_set:1; 		// has the saved mode for failsafe been set?
    uint8_t low_battery:1; 			// flag to hold whether battery low voltage threshold has been breached
    enum FlightMode saved_mode;		// saved flight mode
    int16_t state;					// A tracking variable for type of failsafe active, Used for failsafe based on loss of RC signal or GCS signal
    uint8_t ch3_counter;			// number of low ch3 values
    uint32_t last_heartbeat_ms;             // the time when the last HEARTBEAT message arrived from a GCS - used for triggering gcs failsafe
    uint32_t ch3_timer_ms;			// A timer used to track how long we have been in a "short failsafe" condition due to loss of RC signal
    uint32_t last_valid_rc_ms;
    uint32_t last_radio_status_remrssi_ms;// last RADIO status packet
} plane_failsafe;

////////////////////////////////////////////////////////////////////////////////
// GPS variables
////////////////////////////////////////////////////////////////////////////////
// This is used to scale GPS values for EEPROM storage
// 10^7 times Decimal GPS means 1 == 1cm
// This approximation makes calculations integer and it's easy to read
static const float t7 = 10000000.0;
// We use atan2 and other trig techniques to calaculate angles
// A counter used to count down valid gps fixes to allow the gps estimate to settle
// before recording our home position (and executing a ground start if we booted with an air start)
static uint8_t ground_start_count      = 5;
// Used to compute a speed estimate from the first valid gps fixes to decide if we are
// on the ground or in the air.  Used to decide if a ground start is appropriate if we
// booted with an air start.
static int16_t ground_start_avg;		// Plane
// true if we have a position estimate from AHRS
static bool have_position;				// Plane

////////////////////////////////////////////////////////////////////////////////
// Location & Navigation
////////////////////////////////////////////////////////////////////////////////
// Plane Distance between plane and next waypoint.  
static uint32_t plane_wp_distance;								// Plane
// There may be two active commands in Auto mode.
// This indicates the active navigation command by index number
static uint8_t nav_command_index; 								// Plane
// This indicates the active non-navigation command by index number
static uint8_t non_nav_command_index;							// Plane
// This is the command type (eg navigate to waypoint) of the active navigation command
static uint8_t nav_command_ID          = NO_COMMAND;	// Plane
static uint8_t non_nav_command_ID      = NO_COMMAND;	// Plane

////////////////////////////////////////////////////////////////////////////////
// 3D Location vectors
////////////////////////////////////////////////////////////////////////////////
// home location is stored when we have a good GPS lock and arm the copter
// Can be reset each the copter is re-armed
static struct   Location home;
// Current location
static struct   Location current_loc;
// The location of the previous waypoint.  Used for track following and altitude ramp calculations
static struct   Location prev_WP;				// Plane
// The location of the current/active waypoint.  Used for altitude ramp, track following and loiter calculations.
static struct   Location next_WP;				// Plane
// The location of the active waypoint in Guided mode.
static struct   Location guided_WP;				// Plane
// The location structure information from the Nav command being processed
static struct   Location next_nav_command;		// Plane
// The location structure information from the Non-Nav command being processed
static struct   Location next_nonnav_command;	// Plane

////////////////////////////////////////////////////////////////////////////////
// Relay
////////////////////////////////////////////////////////////////////////////////
static AP_Relay relay;
// handle servo and relay events
static AP_ServoRelayEvents ServoRelayEvents(relay);

//Reference to the camera object (it uses the relay object inside it)
#if CAMERA == ENABLED
  static AP_Camera camera(&relay);
#endif

////////////////////////////////////////////////////////////////////////////////
// Battery Sensors
////////////////////////////////////////////////////////////////////////////////
static AP_BattMonitor battery;

// a pin for reading the receiver RSSI voltage. 
static AP_HAL::AnalogSource* rssi_analog_source;

// Input sources for battery voltage, battery current, board vcc
static AP_HAL::AnalogSource* board_vcc_analog_source;

////////////////////////////////////////////////////////////////////////////////
// flight mode specific
////////////////////////////////////////////////////////////////////////////////
// Altitude threshold to complete a takeoff command in autonomous modes.  Centimeters
static int32_t takeoff_altitude_cm;				// Plane
// Minimum pitch to hold during takeoff command execution.  Hundredths of a degree
static int16_t takeoff_pitch_cd;				// Plane
// true if we are in an auto-throttle mode, which means
// we need to run the speed/height controller
static bool auto_throttle_mode;					// Plane
// this controls throttle suppression in auto modes
static bool throttle_suppressed;				// Plane

////////////////////////////////////////////////////////////////////////////////
// Conditional command
////////////////////////////////////////////////////////////////////////////////
// A value used in condition commands (eg delay, change alt, etc.)
// For example in a change altitude command, it is the altitude to change to.
static int32_t condition_value;  // used in condition commands (eg delay, change alt, etc.)
// A starting value used to check the status of a conditional command.
// For example in a delay command the condition_start records that start time for the delay
static uint32_t condition_start;

////////////////////////////////////////////////////////////////////////////////
// IMU variables
////////////////////////////////////////////////////////////////////////////////
// Integration time (in seconds) for the gyros (DCM algorithm)
// Updated with the fast loop
static float G_Dt = 0.02;
// A value used in condition commands.  For example the rate at which to change altitude.
static int16_t condition_rate;

////////////////////////////////////////////////////////////////////////////////
// System Timers
////////////////////////////////////////////////////////////////////////////////
// Time in microseconds of main control loop
static uint32_t fast_loopTimer;
// Counter of main loop executions.  Used for performance monitoring and failsafe processing
static uint16_t mainLoop_count;
// Number of milliseconds used in last main loop cycle
static uint32_t delta_us_fast_loop;

// Camera/Antenna mount tracking and stabilisation stuff
// --------------------------------------
#if MOUNT == ENABLED
// current_loc uses the baro/gps soloution for altitude rather than gps only.
// mabe one could use current_loc for lat/lon too and eliminate g_gps alltogether?
static AP_Mount camera_mount(&current_loc, g_gps, ahrs, 0);
#endif

#if MOUNT2 == ENABLED
// current_loc uses the baro/gps soloution for altitude rather than gps only.
// mabe one could use current_loc for lat/lon too and eliminate g_gps alltogether?
static AP_Mount camera_mount2(&current_loc, g_gps, ahrs, 1);
#endif

////////////////////////////////////////////////////////////////////////////////
// Airspeed				// Plane
////////////////////////////////////////////////////////////////////////////////
// The calculated airspeed to use in FBW-B.  Also used in higher modes for insuring min ground speed is met.
// Also used for flap deployment criteria.  Centimeters per second.
static int32_t target_airspeed_cm;

// The difference between current and desired airspeed.  Used in the pitch controller.  Centimeters per second.
static float airspeed_error_cm;

// An amount that the airspeed should be increased in auto modes based on the user positioning the
// throttle stick in the top half of the range.  Centimeters per second.
static int16_t airspeed_nudge_cm;

// Similar to airspeed_nudge, but used when no airspeed sensor.
// 0-(plthr_max - plthr_cruise) : throttle nudge in Auto mode using top 1/2 of throttle stick travel
static int16_t throttle_nudge = 0;

////////////////////////////////////////////////////////////////////////////////
// Ground speed			// Plane
////////////////////////////////////////////////////////////////////////////////
// The amount current ground speed is below min ground speed.  Centimeters per second
static int32_t groundspeed_undershoot = 0;

////////////////////////////////////////////////////////////////////////////////
// Airspeed Sensors
////////////////////////////////////////////////////////////////////////////////
AP_Airspeed airspeed(aparm);

////////////////////////////////////////////////////////////////////////////////
// PLANE_ACRO Mode
////////////////////////////////////////////////////////////////////////////////

static struct {					// Plane
    bool locked_roll;
    bool locked_pitch;
    float locked_roll_err;
    int32_t locked_pitch_cd;
} acro_state;

////////////////////////////////////////////////////////////////////////////////
// PLANE_CRUISE controller state		// Plane
////////////////////////////////////////////////////////////////////////////////
static struct {
    bool locked_heading;
    int32_t locked_heading_cd;
    uint32_t lock_timer_ms;
} cruise_state;

////////////////////////////////////////////////////////////////////////////////
// ground steering controller state		// Plane
////////////////////////////////////////////////////////////////////////////////
static struct {
	// Direction held during phases of takeoff and landing centidegrees
	// A value of -1 indicates the course has not been set/is not in use
	// this is a 0..36000 value, or -1 for disabled
    int32_t hold_course_cd;

    // locked_course and locked_course_cd are used in stabilize mode 
    // when ground steering is active
    bool locked_course;
    float locked_course_err;
} steer_state = {
	hold_course_cd : -1,
};

////////////////////////////////////////////////////////////////////////////////
// Waypoint distances		// Plane
////////////////////////////////////////////////////////////////////////////////
// Distance between previous and next waypoint.  Meters
static uint32_t wp_totalDistance;

////////////////////////////////////////////////////////////////////////////////
// Circle Mode / Loiter control
////////////////////////////////////////////////////////////////////////////////
static struct {			// Plane
    // previous target bearing, used to update sum_cd
    int32_t old_target_bearing_cd;

    // Total desired rotation in a loiter.  Used for Loiter Turns commands. 
    int32_t total_cd;

    // total angle completed in the loiter so far
    int32_t sum_cd;

	// Direction for loiter. 1 for clockwise, -1 for counter-clockwise
    int8_t direction;

	// start time of the loiter.  Milliseconds.
    uint32_t start_time_ms;

	// The amount of time we should stay in a loiter for the Loiter Time command.  Milliseconds.
    uint32_t time_max_ms;
} loiter;

////////////////////////////////////////////////////////////////////////////////
// Altitude
////////////////////////////////////////////////////////////////////////////////
// Difference between current altitude and desired altitude.  Centimeters
static int32_t altitude_error_cm;			// Plane
// The current desired altitude.  Altitude is linearly ramped between waypoints.  Centimeters
static int32_t target_altitude_cm;			// Plane
// Altitude difference between previous and current waypoint.  Centimeters
static int32_t offset_altitude_cm;			// Plane

////////////////////////////////////////////////////////////////////////////////
// Navigation control variables
////////////////////////////////////////////////////////////////////////////////
// The instantaneous desired bank angle.  Hundredths of a degree
static int32_t nav_roll_cd;		// Plane

// The instantaneous desired pitch angle.  Hundredths of a degree
static int32_t nav_pitch_cd;	// Plane

////////////////////////////////////////////////////////////////////////////////
// Navigation Throttle control
////////////////////////////////////////////////////////////////////////////////
// The Commanded Throttle from the autopilot.
static int16_t nav_throttle;    // 0-1000 for throttle control
// This is a simple counter to track the amount of throttle used during flight
// This could be useful later in determining and debuging current usage and predicting battery life
static uint32_t throttle_integrator;
////////////////////////////////////////////////////////////////////////////////
// Performance monitoring
////////////////////////////////////////////////////////////////////////////////
// Timer used to accrue data and trigger recording of the performanc monitoring log message
static uint32_t perf_mon_timer;
// The maximum main loop execution time recorded in the current performance monitoring interval
static uint32_t perf_info_max_time = 0;
// The number of gps fixes recorded in the current performance monitoring interval
static uint8_t gps_fix_count = 0;

////////////////////////////////////////////////////////////////////////////////
// Arming/Disarming mangement class
////////////////////////////////////////////////////////////////////////////////
static AP_Arming arming(ahrs, barometer, ap.home_is_set, gcs_send_text_P);

////////////////////////////////////////////////////////////////////////////////
// Outback Challenge Failsafe Support
////////////////////////////////////////////////////////////////////////////////
#if OBC_FAILSAFE == ENABLED
APM_OBC obc;
#endif

// selected navigation controller
static AP_Navigation *nav_controller = &L1_controller;

// selected navigation controller
static AP_SpdHgtControl *SpdHgt_Controller = &TECS_controller;

// This is a convienience accessor for the IMU roll rates. It's currently the raw IMU rates
// and not the adjusted omega rates, but the name is stuck
static Vector3f omega;

////////////////////////////////////////////////////////////////////////////////
// Top-level logic
////////////////////////////////////////////////////////////////////////////////

// setup the var_info table
AP_Param param_loader(var_info, WP_START_BYTE);
/*
  PLANE scheduler table - all regular tasks are listed here, along with how
  often they should be called (in 20ms units) and the maximum time
  they are expected to take (in microseconds)
 */
static const AP_Scheduler::Task plane_scheduler_tasks[] PROGMEM = {
    { plane_read_radio,             1,    700 }, // 0
    { check_short_failsafe,   1,   1000 },
    { ahrs_update,            1,   6400 },
    { update_speed_height,    1,   1600 },
    { update_flight_mode,     1,   1400 },
    { plane_stabilize,        1,   3500 },
    { plane_set_servos,       1,   1600 },
    { plane_read_control_switch,    7,   1000 },
    { gcs_retry_deferred,     1,   1000 },
    { update_GPS_50Hz,        1,   2500 },
    { update_GPS_10Hz,        5,   2500 }, // 10
    { navigate,               5,   3000 },
    { update_compass,         5,   1200 },
    { read_airspeed,          5,   1200 },
    { update_alt,             5,   3400 },
    { calc_altitude_error,    5,   1000 },
    { plane_update_commands,        5,   5000 },
    { obc_fs_check,           5,   1000 },
    { gcs_check_input,             1,   1700 },
    { gcs_data_stream_send,   1,   3000 },
    { update_events,		  1,   1500 }, // 20
    { check_usb_mux,          5,    300 },
    { plane_read_battery,     5,   1000 },
    { compass_accumulate,     1,   1500 },
    { barometer_accumulate,   1,    900 },
    { update_notify,          1,    300 },
    { one_second_loop,       50,   1000 },
    { check_long_failsafe,   15,   1000 },
    { read_receiver_rssi,     5,   1000 },
    { airspeed_ratio_update, 50,   1000 }, // 30
    { update_mount,           1,   1500 },
    { log_perf_info,        500,   1000 },
    { compass_save,        3000,   2500 },
    { update_logging1,        5,   1700 },
    { update_logging2,        5,   1700 },
};

void setup() {
    cliSerial = hal.console;

    // Load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

    AP_Notify::flags.failsafe_battery = false;

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&plane_scheduler_tasks[0], sizeof(plane_scheduler_tasks)/sizeof(plane_scheduler_tasks[0]));
}

/*
  if the compass is enabled then try to accumulate a reading
 */
static void compass_accumulate(void)
{
    if (g.compass_enabled) {
        compass.accumulate();
    }
}

/*
  try to accumulate a baro reading
 */
static void barometer_accumulate(void)
{
    barometer.accumulate();
}

void loop()
{
    // wait for an INS sample
    if (!ins.wait_for_sample(1000)) {
        return;
    }
    uint32_t timer = micros();

    // check loop time
    perf_info_check_loop_time(timer - fast_loopTimer);

    // used by PI Loops
	delta_us_fast_loop  	= timer - fast_loopTimer;
    G_Dt                    = (float)(timer - fast_loopTimer) / 1000000.f;
    fast_loopTimer          = timer;

    // for mainloop failure monitoring
    mainLoop_count++;

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    uint32_t time_available = (timer + 20000) - micros();
    if (time_available > 19500) { time_available = 19500; }
    scheduler.run(time_available);
}

// END OF MAIN LOOP

//  update camera mount
static void update_mount()
{
#if MOUNT == ENABLED
    camera_mount.update_mount_position();
#endif
#if MOUNT2 == ENABLED
    camera_mount2.update_mount_position();
#endif
#if CAMERA == ENABLED
    camera.trigger_pic_cleanup();
#endif
}

// update AHRS system
static void ahrs_update()
{
#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before AHRS update
    gcs_check_input();
#endif

    ahrs.update();

    if (should_log(MASK_LOG_ATTITUDE_FAST))
        plane_Log_Write_Attitude();

    if (should_log(MASK_LOG_IMU))
        Log_Write_IMU();

    // calculate a scaled roll limit based on current pitch
    roll_limit_cd = g.roll_limit_cd * cosf(ahrs.pitch);
    pitch_limit_min_cd = aparm.pitch_limit_min_cd * fabsf(cosf(ahrs.roll));
}

/*
  update 50Hz speed/height controller
 */
static void update_speed_height(void)
{
    if (auto_throttle_mode) {
	    // Call TECS 50Hz update. Note that we call this regardless of
	    // throttle suppressed, as this needs to be running for
	    // takeoff detection
        SpdHgt_Controller->update_50hz(relative_altitude());
    }
}

/*
  read and update compass
 */
static void update_compass(void)
{
    if (g.compass_enabled && compass.read()) {
        ahrs.set_compass(&compass);
        compass.null_offsets();
        if (should_log(MASK_LOG_COMPASS)) {
            plane_Log_Write_Compass();
        }
    } else {
        ahrs.set_compass(NULL);
    }
}

/*
  do 10Hz logging
 */
static void update_logging1(void)
{
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST))
        plane_Log_Write_Attitude();

    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_IMU))
        Log_Write_IMU();
}

/*
  do 10Hz logging - part2
 */
static void update_logging2(void)
{
    if (should_log(MASK_LOG_CTUN))
        plane_Log_Write_Control_Tuning();
    
    if (should_log(MASK_LOG_NTUN))
        plane_Log_Write_Nav_Tuning();

    if (should_log(MASK_LOG_RCIN))
        DataFlash.Log_Write_RCIN();
    
    if (should_log(MASK_LOG_RCOUT))
        DataFlash.Log_Write_RCOUT();
}

/*
  check for OBC failsafe check
 */
static void obc_fs_check(void)
{
#if OBC_FAILSAFE == ENABLED
    // perform OBC failsafe checks
    obc.check(OBC_MODE(plane_control_mode),
              plane_failsafe.last_heartbeat_ms,
              g_gps ? g_gps->last_fix_time : 0);
#endif
}

static void one_second_loop()
{
    if (should_log(MASK_LOG_CURRENT))
        Log_Write_Current();

    // send a heartbeat
    gcs_send_message(MSG_HEARTBEAT);

    // make it possible to change control channel ordering at runtime
    set_control_channels();

    // make it possible to change orientation at runtime
    ahrs.set_orientation();

    // sync MAVLink system ID
    mavlink_system.sysid = g.sysid_this_mav;

    aux_servos_update_fn();
    enable_aux_servos();
	
    // update notify flags
    AP_Notify::flags.pre_arm_check = arming.pre_arm_checks(false);
    AP_Notify::flags.armed = arming.is_armed() || arming.arming_required() == AP_Arming::NO;
}

static void log_perf_info()
{
    if (scheduler.debug() != 0) {
        hal.console->printf_P(PSTR("perf_info_max_time=%lu\n"), (unsigned long)perf_info_max_time);
    }
    if (should_log(MASK_LOG_PM))
        plane_Log_Write_Performance();
    perf_info_max_time = 0;
    resetPerfData();
}

static void compass_save()
{
    if (g.compass_enabled) {
        compass.save_offsets();
    }
}

/*
  once a second update the airspeed calibration ratio
 */
static void airspeed_ratio_update(void)
{
    if (!airspeed.enabled() ||
        g_gps->status() < GPS::GPS_OK_FIX_3D ||
        g_gps->ground_speed_cm < 400) {
        // don't calibrate when not moving
        return;        
    }
    if (airspeed.get_airspeed() < aparm.airspeed_min && 
        g_gps->ground_speed_cm < (uint32_t)aparm.airspeed_min*100) {
        // don't calibrate when flying below the minimum airspeed. We
        // check both airspeed and ground speed to catch cases where
        // the airspeed ratio is way too low, which could lead to it
        // never coming up again
        return;
    }
    if (abs(ahrs.roll_sensor) > roll_limit_cd ||
        ahrs.pitch_sensor > aparm.pitch_limit_max_cd ||
        ahrs.pitch_sensor < pitch_limit_min_cd) {
        // don't calibrate when going beyond normal flight envelope
        return;
    }
    Vector3f vg = g_gps->velocity_vector();
    airspeed.update_calibration(vg);
    gcs_send_airspeed_calibration(vg);
}

/*
  read the GPS and update position
 */
static void update_GPS_50Hz(void)
{
    static uint32_t last_gps_reading;
    g_gps->update();
    if (g_gps->last_message_time_ms() != last_gps_reading) {
        last_gps_reading = g_gps->last_message_time_ms();
        if (should_log(MASK_LOG_GPS)) {
            Log_Write_GPS();
        }
    }
}

/*
  read update GPS position - 10Hz update
 */
static void update_GPS_10Hz(void)
{
    // get position from AHRS
    have_position = ahrs.get_projected_position(current_loc);

    if (g_gps->new_data && g_gps->status() >= GPS::GPS_OK_FIX_3D) {
        g_gps->new_data = false;

        // for performance
        // ---------------
        gps_fix_count++;

        if(ground_start_count > 1) {
            ground_start_count--;
            ground_start_avg += g_gps->ground_speed_cm;

        } else if (ground_start_count == 1) {
            // We countdown N number of good GPS fixes
            // so that the altitude is more accurate
            // -------------------------------------
            if (current_loc.lat == 0) {
                ground_start_count = 5;

            } else {
                plane_init_home();

                // set system clock for log timestamps
                hal.util->set_system_clock(g_gps->time_epoch_usec());

                if (g.compass_enabled) {
                    // Set compass declination automatically
                    compass.set_initial_location(g_gps->latitude, g_gps->longitude);
                }
                ground_start_count = 0;
            }
        }

        // see if we've breached the geo-fence
        geofence_check(false);

#if CAMERA == ENABLED
        if (camera.update_location(current_loc) == true) {
            plane_do_take_picture();
        }
#endif        

        if (!arming.is_armed() ||
            hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
            update_home();
        }
    }

    calc_gndspeed_undershoot();
}

/*
  main handling for PLANE_AUTO mode
 */
static void handle_auto_mode(void)
{
    switch(nav_command_ID) {
    case MAV_CMD_NAV_TAKEOFF:
        if (steer_state.hold_course_cd == -1) {
            // we don't yet have a heading to hold - just level
            // the wings until we get up enough speed to get a GPS heading
            nav_roll_cd = 0;
        } else {
            calc_nav_roll();
            // during takeoff use the level flight roll limit to
            // prevent large course corrections
            nav_roll_cd = constrain_int32(nav_roll_cd, -g.level_roll_limit*100UL, g.level_roll_limit*100UL);
        }
        
        if (airspeed.use()) {
            calc_nav_pitch();
            if (nav_pitch_cd < takeoff_pitch_cd)
                nav_pitch_cd = takeoff_pitch_cd;
        } else {
            nav_pitch_cd = (g_gps->ground_speed_cm / (float)g.airspeed_cruise_cm) * takeoff_pitch_cd;
            nav_pitch_cd = constrain_int32(nav_pitch_cd, 500, takeoff_pitch_cd);
        }
        
        // max throttle for takeoff
        channel_throttle->servo_out = aparm.plthr_max;
        break;

    case MAV_CMD_NAV_LAND:
        calc_nav_roll();
        
        if (ap.land_complete) {
            // during final approach constrain roll to the range
            // allowed for level flight
            nav_roll_cd = constrain_int32(nav_roll_cd, -g.level_roll_limit*100UL, g.level_roll_limit*100UL);
            
            // hold pitch constant in final approach
            nav_pitch_cd = g.land_pitch_cd;
        } else {
            calc_nav_pitch();
            if (!airspeed.use()) {
                // when not under airspeed control, don't allow
                // down pitch in landing
                nav_pitch_cd = constrain_int32(nav_pitch_cd, 0, nav_pitch_cd);
            }
        }
        calc_throttle();
        
        if (ap.land_complete) {
            // we are in the final stage of a landing - force
            // zero throttle
            channel_throttle->servo_out = 0;
        }
        break;
        
    default:
        // we are doing normal PLANE_AUTO flight, the special cases
        // are for takeoff and landing
        steer_state.hold_course_cd = -1;
        ap.land_complete = false;
        calc_nav_roll();
        calc_nav_pitch();
        calc_throttle();
        break;
    }
}

/*
  main flight mode dependent update code 
 */
static void update_flight_mode(void)
{
    enum FlightMode effective_mode = plane_control_mode;
    if (plane_control_mode == PLANE_AUTO && g.auto_fbw_steer) {
        effective_mode = PLANE_FLY_BY_WIRE_A;
    }

    if (effective_mode != PLANE_AUTO) {
        // hold_course is only used in takeoff and landing
        steer_state.hold_course_cd = -1;
    }

    switch (effective_mode) 
    {
    case PLANE_AUTO:
        handle_auto_mode();
        break;

    case PLANE_RTL:
    case PLANE_LOITER:
    case PLANE_GUIDED:
        calc_nav_roll();
        calc_nav_pitch();
        calc_throttle();
        break;
        
    case PLANE_TRAINING: {
        training_manual_roll = false;
        training_manual_pitch = false;
        
        // if the roll is past the set roll limit, then
        // we set target roll to the limit
        if (ahrs.roll_sensor >= roll_limit_cd) {
            nav_roll_cd = roll_limit_cd;
        } else if (ahrs.roll_sensor <= -roll_limit_cd) {
            nav_roll_cd = -roll_limit_cd;                
        } else {
            training_manual_roll = true;
            nav_roll_cd = 0;
        }
        
        // if the pitch is past the set pitch limits, then
        // we set target pitch to the limit
        if (ahrs.pitch_sensor >= aparm.pitch_limit_max_cd) {
            nav_pitch_cd = aparm.pitch_limit_max_cd;
        } else if (ahrs.pitch_sensor <= pitch_limit_min_cd) {
            nav_pitch_cd = pitch_limit_min_cd;
        } else {
            training_manual_pitch = true;
            nav_pitch_cd = 0;
        }
        if (inverted_flight) {
            nav_pitch_cd = -nav_pitch_cd;
        }
        break;
    }

    case PLANE_ACRO: {
        // handle locked/unlocked control
        if (acro_state.locked_roll) {
            nav_roll_cd = acro_state.locked_roll_err;
        } else {
            nav_roll_cd = ahrs.roll_sensor;
        }
        if (acro_state.locked_pitch) {
            nav_pitch_cd = acro_state.locked_pitch_cd;
        } else {
            nav_pitch_cd = ahrs.pitch_sensor;
        }
        break;
    }

    case PLANE_FLY_BY_WIRE_A: {
        // set nav_roll and nav_pitch using sticks
        nav_roll_cd  = channel_roll->norm_input() * roll_limit_cd;
        nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
        float pitch_input = channel_pitch->norm_input();
        if (pitch_input > 0) {
            nav_pitch_cd = pitch_input * aparm.pitch_limit_max_cd;
        } else {
            nav_pitch_cd = -(pitch_input * pitch_limit_min_cd);
        }
        nav_pitch_cd = constrain_int32(nav_pitch_cd, pitch_limit_min_cd, aparm.pitch_limit_max_cd.get());
        if (inverted_flight) {
            nav_pitch_cd = -nav_pitch_cd;
        }
        if (plane_failsafe.ch3_failsafe && g.short_fs_action == 2) {
            // FBWA failsafe glide
            nav_roll_cd = 0;
            nav_pitch_cd = 0;
        }
        break;
    }

    case PLANE_FLY_BY_WIRE_B:
        // Thanks to Yury MonZon for the altitude limit code!
        nav_roll_cd = channel_roll->norm_input() * roll_limit_cd;
        update_fbwb_speed_height();
        break;
        
    case PLANE_CRUISE:
        /*
          in PLANE_CRUISE mode we use the navigation code to control
          roll when heading is locked. Heading becomes unlocked on
          any aileron or rudder input
        */
        if ((channel_roll->control_in != 0 ||
             channel_rudder->control_in != 0)) {                
            cruise_state.locked_heading = false;
            cruise_state.lock_timer_ms = 0;
        }                 
        
        if (!cruise_state.locked_heading) {
            nav_roll_cd = channel_roll->norm_input() * roll_limit_cd;
        } else {
            calc_nav_roll();
        }
        update_fbwb_speed_height();
        break;
        
    case PLANE_STABILIZE:
        nav_roll_cd        = 0;
        nav_pitch_cd       = 0;
        // throttle is passthrough
        break;
        
    case PLANE_CIRCLE:
        // we have no GPS installed and have lost radio contact
        // or we just want to fly around in a gentle circle w/o GPS,
        // holding altitude at the altitude we set when we
        // switched into the mode
        nav_roll_cd  = roll_limit_cd / 3;
        calc_nav_pitch();
        calc_throttle();
        break;

    case PLANE_MANUAL:
        // servo_out is for Sim control only
        // ---------------------------------
        channel_roll->servo_out = channel_roll->pwm_to_angle();
        channel_pitch->servo_out = channel_pitch->pwm_to_angle();
        channel_rudder->servo_out = channel_rudder->pwm_to_angle();
        break;
        //roll: -13788.000,  pitch: -13698.000,   thr: 0.000, rud: -13742.000
        
    case PLANE_INITIALISING:
        // handled elsewhere
        break;
    }
}

static void update_navigation()
{
    // plane_wp_distance is in ACTUAL meters, not the *100 meters we get from the GPS
    // ------------------------------------------------------------------------

    // distance and bearing calcs only
    switch(plane_control_mode) {
    case PLANE_AUTO:
        plane_verify_commands();
        break;
            
    case PLANE_LOITER:
    case PLANE_RTL:
    case PLANE_GUIDED:
        // allow loiter direction to be changed in flight
        if (g.loiter_radius < 0) {
            loiter.direction = -1;
        } else {
            loiter.direction = 1;
        }
        update_loiter();
        break;

    case PLANE_CRUISE:
        update_cruise();
        break;

    case PLANE_MANUAL:
    case PLANE_STABILIZE:
    case PLANE_TRAINING:
    case PLANE_INITIALISING:
    case PLANE_ACRO:
    case PLANE_FLY_BY_WIRE_A:
    case PLANE_FLY_BY_WIRE_B:
    case PLANE_CIRCLE:
        // nothing to do
        break;
    }
}

static void update_alt()
{
    if (barometer.healthy) {
        // alt_MSL centimeters (centimeters)
        current_loc.alt = (1 - g.altitude_mix) * g_gps->altitude_cm;
        current_loc.alt += g.altitude_mix * (plane_read_barometer() + home.alt);
    } else if (g_gps->status() >= GPS::GPS_OK_FIX_3D) {
        // alt_MSL centimeters (centimeters)
        current_loc.alt = g_gps->altitude_cm;
    }

    geofence_check(true);

    // Update the speed & height controller states
    if (auto_throttle_mode && !throttle_suppressed) {
        AP_SpdHgtControl::FlightStage flight_stage = AP_SpdHgtControl::FLIGHT_NORMAL;
        
        if (plane_control_mode==PLANE_AUTO) {
            if (ap.takeoff_complete == false) {
                flight_stage = AP_SpdHgtControl::FLIGHT_TAKEOFF;
            } else if (nav_command_ID == MAV_CMD_NAV_LAND && ap.land_complete == true) {
                flight_stage = AP_SpdHgtControl::FLIGHT_LAND_FINAL;
            } else if (nav_command_ID == MAV_CMD_NAV_LAND) {
                flight_stage = AP_SpdHgtControl::FLIGHT_LAND_APPROACH; 
            }
        }

        SpdHgt_Controller->update_pitch_throttle(target_altitude_cm - home.alt + (int32_t(g.alt_offset)*100), 
                                                 target_airspeed_cm,
                                                 flight_stage,
                                                 takeoff_pitch_cd,
                                                 throttle_nudge,
                                                 relative_altitude());
        if (should_log(MASK_LOG_TECS)) {
            Log_Write_TECS_Tuning();
        }
    }

    // tell AHRS the airspeed to true airspeed ratio
    airspeed.set_EAS2TAS(barometer.get_EAS2TAS());
}

AP_HAL_MAIN();
