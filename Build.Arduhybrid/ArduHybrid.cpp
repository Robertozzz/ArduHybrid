#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/ArduHybrid.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduHybrid V1.0"
/*
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
/*
 *  ArduCopter Version 3.0
 *  Creator:        Jason Short
 *  Lead Developer: Randy Mackay
 *  Lead Tester:    Marco Robustini 
 *  Based on code and ideas from the Arducopter team: Leonard Hall, Andrew Tridgell, Robert Lefebvre, Pat Hickey, Michael Oborne, Jani Hirvinen, 
                                                      Olivier Adler, Kevin Hester, Arthur Benemann, Jonathan Challinger, John Arne Birkeland,
                                                      Jean-Louis Naudin, Mike Smith, and more
 *  Thanks to:	Chris Anderson, Jordi Munoz, Jason Short, Doug Weibel, Jose Julio
 *
 *  Special Thanks to contributors (in alphabetical order by first name):
 *
 *  Adam M Rivera		:Auto Compass Declination
 *  Amilcar Lucas		:Camera mount library
 *  Andrew Tridgell		:General development, Mavlink Support
 *  Angel Fernandez		:Alpha testing
 *  AndreasAntonopoulous:GeoFence
 *  Arthur Benemann     :DroidPlanner GCS
 *  Benjamin Pelletier  :Libraries
 *  Bill King           :Single Copter
 *  Christof Schmid		:Alpha testing
 *  Craig Elder         :Release Management, Support
 *  Dani Saez           :V Octo Support
 *  Doug Weibel			:DCM, Libraries, Control law advice
 *  Gregory Fletcher	:Camera mount orientation math
 *  Guntars				:Arming safety suggestion
 *  HappyKillmore		:Mavlink GCS
 *  Hein Hollander      :Octo Support, Heli Testing
 *  Igor van Airde      :Control Law optimization
 *  Jack Dunkle			:Alpha testing
 *  James Goppert		:Mavlink Support
 *  Jani Hiriven		:Testing feedback
 *  Jean-Louis Naudin   :Auto Landing
 *  John Arne Birkeland	:PPM Encoder
 *  Jose Julio			:Stabilization Control laws, MPU6k driver
 *  Julian Oes          :Pixhawk
 *  Jonathan Challinger :Inertial Navigation, CompassMot, Spin-When-Armed
 *  Kevin Hester        :Andropilot GCS
 *  Max Levine			:Tri Support, Graphics
 *  Leonard Hall 		:Flight Dynamics, Throttle, Loiter and Navigation Controllers
 *  Marco Robustini		:Lead tester
 *  Michael Oborne		:Mission Planner GCS
 *  Mike Smith			:Pixhawk driver, coding support
 *  Olivier Adler       :PPM Encoder, piezo buzzer
 *  Pat Hickey          :Hardware Abstraaction Layer (HAL)
 *  Robert Lefebvre		:Heli Support, Copter LEDs
 *  Roberto Navoni      :Library testing, Porting to VRBrain
 *  Sandro Benigno      :Camera support, MinimOSD
 *  ..and many more.
 *
 *  Code commit statistics can be found here: https://github.com/diydrones/ardupilot/graphs/contributors
 *  Wiki: http://copter.ardupilot.com/
 *  Requires modified version of Arduino, which can be found here: http://ardupilot.com/downloads/?category=6
 *
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
#include <AP_GPS_Glitch.h>      // GPS glitch protection library
#include <DataFlash.h>          // ArduPilot Mega Flash Memory Library
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>        	// ArduPilot barometer library
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Curve.h>           // Curve used to linearlise throttle pwm to thrust
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_AHRS.h>         	// ArduPilot Mega DCM Library
#include <APM_PI.h>             // PI library
#include <AC_PID.h>             // PID library
#include <RC_Channel.h>         // RC Channel Library
#include <AP_Motors.h>          // AP Motors library
#include <AP_RangeFinder.h>     // Range finder library
#include <AP_OpticalFlow.h>     // Optical Flow library
#include <Filter.h>             // Filter library
#include <AP_Buffer.h>          // APM FIFO Buffer
#include <AP_Relay.h>           // APM relay
#include <AP_ServoRelayEvents.h>
#include <AP_Camera.h>          // Photo or video camera
#include <AP_Mount.h>           // Camera/Antenna mount
#include <AP_Airspeed.h>        // needed for AHRS build
#include <AP_Vehicle.h>         // needed for AHRS build
#include <AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
#include <AC_WPNav.h>     		// ArduCopter waypoint navigation library
#include <AP_Declination.h>     // ArduPilot Mega Declination Helper Library
#include <AC_Fence.h>           // Arducopter Fence library
#include <SITL.h>               // software in the loop support
#include <AP_Scheduler.h>       // main loop scheduler
#include <AP_RCMapper.h>        // RC input mapping library
#include <AP_Notify.h>          // Notify library
#include <AP_BattMonitor.h>     // Battery monitor library
#include <AP_BoardConfig.h>     // board configuration library
#if SPRAYER == ENABLED
#include <AC_Sprayer.h>         // crop sprayer library
#endif
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
#include "config_channels.h"

// Local modules
#include "Parameters.h"
#include "GCS.h"
#include <APM_OBC.h>			// Plane
#include <APM_Control.h>		// Plane

#line 1 "autogenerated"
    void setup() ;
 static void compass_accumulate(void) ;
 static void barometer_accumulate(void) ;
  static void perf_update(void) ;
  void loop() ;
 static void fast_loop() ;
 static void throttle_loop() ;
 static void update_mount() ;
 static void update_batt_compass(void) ;
 static void ten_hz_logging_loop() ;
 static void fifty_hz_logging_loop() ;
 static void three_hz_loop() ;
 static void one_hz_loop() ;
 static void update_optical_flow(void) ;
 static void update_GPS(void) ;
 bool set_yaw_mode(uint8_t new_yaw_mode) ;
 void update_yaw_mode(void) ;
 uint8_t get_wp_yaw_mode(bool rtl) ;
 bool set_roll_pitch_mode(uint8_t new_roll_pitch_mode) ;
 void exit_roll_pitch_mode(uint8_t old_roll_pitch_mode) ;
 void update_roll_pitch_mode(void) ;
  static void init_simple_bearing() ;
 void update_simple_mode(void) ;
 void update_super_simple_bearing(bool force_update) ;
 bool throttle_mode_manual(uint8_t thr_mode) ;
 bool set_throttle_mode( uint8_t new_throttle_mode ) ;
 void update_throttle_mode(void) ;
 static void set_target_alt_for_reporting(float alt_cm) ;
 static float get_target_alt_for_reporting() ;
  static void read_AHRS(void) ;
  static void update_trig(void);
 static void update_altitude() ;
  static void tuning();
 static void ahrs_update() ;
 static void update_speed_height(void) ;
 static void update_compass(void) ;
 static void update_logging1(void) ;
 static void update_logging2(void) ;
 static void obc_fs_check(void) ;
  static void one_second_loop() ;
  static void log_perf_info() ;
  static void compass_save() ;
 static void airspeed_ratio_update(void) ;
 static void update_GPS_50Hz(void) ;
 static void update_GPS_10Hz(void) ;
 static void handle_auto_mode(void) ;
 static void update_flight_mode(void) ;
  static void update_navigation() ;
  static void update_alt() ;
   void set_home_is_set(bool b) ;
 void set_auto_armed(bool b) ;
 void set_simple_mode(uint8_t b) ;
 static void set_failsafe_radio(bool b) ;
 void set_failsafe_battery(bool b) ;
 static void set_failsafe_gps(bool b) ;
 static void set_failsafe_gcs(bool b) ;
 void set_takeoff_complete(bool b) ;
 void set_land_complete(bool b) ;
  void set_pre_arm_check(bool b) ;
  void set_pre_arm_rc_check(bool b) ;
  static void reset_roll_pitch_in_filters(int16_t roll_in, int16_t pitch_in) ;
 static void get_pilot_desired_lean_angles(int16_t roll_in, int16_t pitch_in, int16_t &roll_out, int16_t &pitch_out) ;
  static void get_stabilize_roll(int32_t target_angle) ;
  static void get_stabilize_pitch(int32_t target_angle) ;
  static void get_stabilize_yaw(int32_t target_angle) ;
 static void get_acro_level_rates() ;
 static void get_roll_rate_stabilized_bf(int32_t stick_angle) ;
 static void get_pitch_rate_stabilized_bf(int32_t stick_angle) ;
 static void get_yaw_rate_stabilized_bf(int32_t stick_angle) ;
 static void get_roll_rate_stabilized_ef(int32_t stick_angle) ;
 static void get_pitch_rate_stabilized_ef(int32_t stick_angle) ;
 static void get_yaw_rate_stabilized_ef(int32_t stick_angle) ;
 void set_roll_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) ;
 void set_pitch_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) ;
 void set_yaw_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) ;
 void update_rate_contoller_targets() ;
 void run_rate_controllers() ;
 static int16_t get_rate_roll(int32_t target_rate) ;
  static int16_t get_rate_pitch(int32_t target_rate) ;
  static int16_t get_rate_yaw(int32_t target_rate) ;
 static int32_t get_of_roll(int32_t input_roll) ;
  static int32_t get_of_pitch(int32_t input_pitch) ;
 static void get_circle_yaw() ;
 static void get_look_at_yaw() ;
  static void get_look_ahead_yaw(int16_t pilot_yaw) ;
 static void update_throttle_cruise(int16_t throttle) ;
 static int16_t get_angle_boost(int16_t throttle) ;
 static int16_t get_angle_boost(int16_t throttle) ;
 void set_throttle_out( int16_t throttle_out, bool apply_angle_boost ) ;
 void set_throttle_accel_target( int16_t desired_acceleration ) ;
 void throttle_accel_deactivate() ;
 static void set_throttle_takeoff() ;
 static int16_t get_throttle_accel(int16_t z_target_accel) ;
 static int16_t get_pilot_desired_throttle(int16_t throttle_control) ;
 static int16_t get_pilot_desired_climb_rate(int16_t throttle_control) ;
 static int32_t get_initial_alt_hold( int32_t alt_cm, int16_t climb_rate_cms) ;
 static void get_throttle_rate(float z_target_speed) ;
 static void get_throttle_althold(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate) ;
 static void get_throttle_althold_with_slew(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate) ;
 static void get_throttle_rate_stabilized(int16_t target_rate) ;
 static float get_speed_scaler(void) ;
 static bool stick_mixing_enabled(void) ;
 static void stabilize_roll(float speed_scaler) ;
 static void stabilize_pitch(float speed_scaler) ;
 static void stick_mix_channel(RC_Channel *channel) ;
 static void stabilize_stick_mixing_direct() ;
 static void stabilize_stick_mixing_fbw() ;
 static void stabilize_yaw(float speed_scaler) ;
 static void stabilize_training(float speed_scaler) ;
 static void stabilize_acro(float speed_scaler) ;
 static void plane_stabilize() ;
   static void calc_throttle() ;
 static void calc_nav_yaw_coordinated(float speed_scaler) ;
 static void calc_nav_yaw_course(void) ;
 static void calc_nav_yaw_ground(void) ;
   static void calc_nav_pitch() ;
   static void calc_nav_roll() ;
 static void throttle_slew_limit(int16_t last_throttle) ;
 static bool auto_takeoff_check(void) ;
 static bool is_flying(void) ;
 static bool suppress_throttle(void) ;
 static void channel_output_mixer(uint8_t mixing_type, int16_t &chan1_out, int16_t &chan2_out) ;
 void throttle_plane_to_copter();
  static void plane_set_servos(void) ;
  static void demo_servos(uint8_t i)  ;
 static void get_throttle_land() ;
 static void reset_land_detector() ;
 static bool update_land_detector() ;
 static void get_throttle_surface_tracking(int16_t target_rate) ;
 static void reset_I_all(void) ;
  static void reset_rate_I() ;
  static void reset_optflow_I(void) ;
  static void reset_throttle_I(void) ;
  static void set_accel_throttle_I_from_pilot_throttle(int16_t pilot_throttle) ;
  static void gcs_send_heartbeat(void) ;
  static void gcs_send_deferred(void) ;
  static NOINLINE void send_heartbeat(mavlink_channel_t chan) ;
  static NOINLINE void plane_send_heartbeat(mavlink_channel_t chan) ;
  static NOINLINE void send_attitude(mavlink_channel_t chan) ;
  static NOINLINE void plane_send_attitude(mavlink_channel_t chan) ;
 static NOINLINE void send_limits_status(mavlink_channel_t chan) ;
 static NOINLINE void send_fence_status(mavlink_channel_t chan) ;
   static NOINLINE void send_extended_status1(mavlink_channel_t chan, uint16_t packet_drops) ;
  static NOINLINE void plane_send_extended_status1(mavlink_channel_t chan) ;
  static void NOINLINE send_location(mavlink_channel_t chan) ;
  static void NOINLINE plane_send_location(mavlink_channel_t chan) ;
  static void NOINLINE send_nav_controller_output(mavlink_channel_t chan) ;
  static void NOINLINE plane_send_nav_controller_output(mavlink_channel_t chan) ;
  static void NOINLINE send_ahrs(mavlink_channel_t chan) ;
  static void NOINLINE plane_send_ahrs(mavlink_channel_t chan) ;
 static void NOINLINE send_simstate(mavlink_channel_t chan) ;
  static void NOINLINE plane_send_simstate(mavlink_channel_t chan) ;
  static void NOINLINE send_hwstatus(mavlink_channel_t chan) ;
  static void NOINLINE send_gps_raw(mavlink_channel_t chan) ;
  static void NOINLINE plane_send_gps_raw(mavlink_channel_t chan) ;
  static void NOINLINE send_system_time(mavlink_channel_t chan) ;
 static void NOINLINE send_servo_out(mavlink_channel_t chan) ;
  static void NOINLINE plane_send_servo_out(mavlink_channel_t chan) ;
  static void NOINLINE send_radio_in(mavlink_channel_t chan) ;
  static void NOINLINE plane_send_radio_in(mavlink_channel_t chan) ;
  static void NOINLINE send_radio_out(mavlink_channel_t chan) ;
  static void NOINLINE plane_send_radio_out(mavlink_channel_t chan) ;
  static void NOINLINE send_vfr_hud(mavlink_channel_t chan) ;
  static void NOINLINE plane_send_vfr_hud(mavlink_channel_t chan) ;
  static void NOINLINE send_raw_imu1(mavlink_channel_t chan) ;
  static void NOINLINE send_raw_imu2(mavlink_channel_t chan) ;
  static void NOINLINE send_raw_imu3(mavlink_channel_t chan) ;
  static void NOINLINE plane_send_raw_imu1(mavlink_channel_t chan) ;
  static void NOINLINE plane_send_raw_imu2(mavlink_channel_t chan) ;
  static void NOINLINE plane_send_raw_imu3(mavlink_channel_t chan) ;
  static void NOINLINE send_wind(mavlink_channel_t chan) ;
  static void NOINLINE send_current_waypoint(mavlink_channel_t chan) ;
  static void NOINLINE send_statustext(mavlink_channel_t chan) ;
 static bool telemetry_delayed(mavlink_channel_t chan) ;
 static bool mavlink_try_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops) ;
  static bool plane_mavlink_try_send_message(mavlink_channel_t chan, enum ap_message id) ;
 static void mavlink_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops) ;
  static void plane_mavlink_send_message(mavlink_channel_t chan, enum ap_message id) ;
  void mavlink_send_text(mavlink_channel_t chan, gcs_severity severity, const char *str) ;
  void plane_mavlink_send_text(mavlink_channel_t chan, gcs_severity severity, const char *str) ;
 static void mavlink_delay_cb() ;
 static void gcs_send_message(enum ap_message id) ;
 static void gcs_data_stream_send(void) ;
 static void gcs_check_input(void) ;
  static void gcs_send_text_P(gcs_severity severity, const prog_char_t *str) ;
 static void gcs_send_airspeed_calibration(const Vector3f &vg) ;
 static void gcs_retry_deferred(void) ;
  static bool print_log_menu(void) ;
  static void do_erase_logs(void) ;
 static void Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float rate_min, float rate_max, float new_gain_rp, float new_gain_rd, float new_gain_sp) ;
 static void Log_Write_AutoTuneDetails(int16_t angle_cd, float rate_cds) ;
 static void Log_Write_Current() ;
 static void Log_Write_Optflow() ;
 static void Log_Write_Nav_Tuning() ;
 static void plane_Log_Write_Nav_Tuning() ;
 static void Log_Write_Control_Tuning() ;
 static void plane_Log_Write_Control_Tuning() ;
 static void Log_Write_Compass() ;
 static void plane_Log_Write_Compass() ;
 static void Log_Write_Performance() ;
 static void plane_Log_Write_Performance() ;
 static void Log_Write_Cmd(uint8_t num, const struct Location *wp) ;
 static void Log_Write_Attitude() ;
 static void plane_Log_Write_Attitude(void) ;
 static void Log_Write_Mode(uint8_t mode) ;
 static void plane_Log_Write_Mode(uint8_t mode) ;
 static void Log_Write_Startup() ;
 static void Log_Write_Camera() ;
 static void Log_Write_TECS_Tuning(void) ;
  static void Log_Arm_Disarm() ;
  static void Log_Write_GPS(void) ;
  static void Log_Write_IMU()  ;
  static void Log_Write_Baro(void) ;
 static void Log_Write_Airspeed(void) ;
 static void Log_Write_Event(uint8_t id) ;
 static void Log_Write_Data(uint8_t id, int16_t value) ;
 static void Log_Write_Data(uint8_t id, uint16_t value) ;
 static void Log_Write_Data(uint8_t id, int32_t value) ;
 static void Log_Write_Data(uint8_t id, uint32_t value) ;
 static void Log_Write_Data(uint8_t id, float value) ;
 static void Log_Write_Error(uint8_t sub_system, uint8_t error_code) ;
 static void Log_Read(uint16_t log_num, uint16_t start_page, uint16_t end_page) ;
 static void start_logging()  ;
 static void Log_Write_Startup() ;
 static void Log_Write_Cmd(uint8_t num, const struct Location *wp) ;
 static void Log_Write_Mode(uint8_t mode) ;
 static void plane_Log_Write_Mode(uint8_t mode) ;
  static void Log_Write_IMU() ;
 static void Log_Write_GPS() ;
 static void Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float rate_min, float rate_max, float new_gain_rp, float new_gain_rd, float new_gain_sp) ;
 static void Log_Write_AutoTuneDetails(int16_t angle_cd, float rate_cds) ;
 static void Log_Write_Current() ;
 static void Log_Write_Compass() ;
 static void Log_Write_Attitude() ;
 static void plane_Log_Write_Compass() ;
 static void plane_Log_Write_Attitude() ;
 static void Log_Write_Data(uint8_t id, int16_t value);
 static void Log_Write_Data(uint8_t id, uint16_t value);
 static void Log_Write_Data(uint8_t id, int32_t value);
 static void Log_Write_Data(uint8_t id, uint32_t value);
 static void Log_Write_Data(uint8_t id, float value);
 static void Log_Write_Event(uint8_t id);
 static void Log_Write_Optflow() ;
 static void Log_Write_Nav_Tuning() ;
 static void plane_Log_Write_Nav_Tuning() ;
  static void Log_Write_Control_Tuning() ;
 static void plane_Log_Write_Control_Tuning() ;
 static void Log_Write_TECS_Tuning() ;
  static void Log_Write_Performance() ;
 static void plane_Log_Write_Performance() ;
 static void Log_Write_Camera() ;
 static void Log_Write_Error(uint8_t sub_system, uint8_t error_code) ;
 static void Log_Write_Airspeed(void) ;
 static void Log_Write_Baro(void) ;
  static void load_parameters(void) ;
 void userhook_init() ;
 void userhook_FastLoop() ;
 void userhook_50Hz() ;
 void userhook_MediumLoop() ;
 void userhook_SlowLoop() ;
 void userhook_SuperSlowLoop() ;
 static void auto_tune_initialise() ;
 static void auto_tune_intra_test_gains() ;
 static void auto_tune_restore_orig_gains() ;
 static void auto_tune_load_tuned_gains() ;
 static void auto_tune_load_test_gains() ;
 static bool auto_tune_start() ;
 static void auto_tune_stop() ;
 static void auto_tune_save_tuning_gains_and_reset() ;
 void auto_tune_update_gcs(uint8_t message_id) ;
 static void get_autotune_roll_pitch_controller(int16_t pilot_roll_angle, int16_t pilot_pitch_angle, int16_t pilot_yaw_command) ;
  void add_altitude_data(unsigned long xl, long y) ;
  static void init_commands() ;
  static void plane_init_commands() ;
 static struct Location get_cmd_with_index(int i) ;
 static struct Location plane_get_cmd_with_index(int16_t i) ;
 static void set_cmd_with_index(struct Location temp, int i) ;
  static void plane_set_cmd_with_index(struct Location temp, int16_t i) ;
  static int32_t get_RTL_alt() ;
 static void init_home() ;
  static void plane_init_home() ;
 static void process_nav_command() ;
 static void process_cond_command() ;
 static void process_now_command() ;
  static void handle_process_nav_cmd() ;
  static void handle_process_condition_command() ;
  static void handle_process_do_command() ;
  static void handle_no_commands() ;
 static bool verify_nav_command() ;
 static bool verify_cond_command() ;
 static void do_RTL(void) ;
 static void do_takeoff() ;
 static void do_nav_wp() ;
 static void do_land(const struct Location *cmd) ;
 static void do_loiter_unlimited() ;
 static void do_circle() ;
 static void do_loiter_time() ;
  static void plane_do_RTL(void) ;
  static void plane_do_takeoff() ;
  static void plane_do_nav_wp() ;
  static void plane_do_land() ;
  static void loiter_set_direction_wp(const struct Location *nav_command) ;
  static void plane_do_loiter_unlimited() ;
  static void do_loiter_turns() ;
  static void plane_do_loiter_time() ;
 static bool verify_takeoff() ;
 static bool verify_land() ;
 static bool verify_nav_wp() ;
  static bool verify_loiter_unlimited() ;
 static bool verify_loiter_time() ;
 static bool verify_circle() ;
 static bool verify_RTL() ;
  static bool plane_verify_takeoff() ;
 static bool plane_verify_land() ;
  static bool plane_verify_nav_wp() ;
  static bool plane_verify_loiter_unlim() ;
  static bool plane_verify_loiter_time() ;
  static bool verify_loiter_turns() ;
  static bool plane_verify_RTL() ;
  static void do_wait_delay() ;
  static void do_change_alt() ;
  static void do_within_distance() ;
  static void do_yaw() ;
  static void plane_do_wait_delay() ;
  static void plane_do_change_alt() ;
  static void plane_do_within_distance() ;
  static bool verify_wait_delay() ;
  static bool verify_change_alt() ;
  static bool verify_within_distance() ;
 static bool verify_yaw() ;
  static bool plane_verify_wait_delay() ;
  static bool plane_verify_change_alt() ;
  static bool plane_verify_within_distance() ;
 static void do_guided(const struct Location *cmd) ;
  static void do_change_speed() ;
  static void do_jump() ;
  static void do_set_home() ;
 static void do_roi() ;
 static void do_take_picture() ;
  static void do_loiter_at_location() ;
  static void plane_do_jump() ;
  static void plane_do_change_speed() ;
  static void plane_do_set_home() ;
 static void plane_do_take_picture() ;
 static void change_command(uint8_t cmd_index) ;
  void plane_change_command(uint8_t cmd_index) ;
 static void update_commands() ;
   static void plane_update_commands(void) ;
 static void execute_nav_command(void) ;
 static void verify_commands(void) ;
  static void plane_verify_commands(void) ;
  static void process_next_command() ;
  static void process_non_nav_command() ;
 static int16_t find_next_nav_index(int16_t search_index) ;
  static void exit_mission() ;
  static void delay(uint32_t ms) ;
  static void mavlink_delay(uint32_t ms) ;
  static uint32_t millis() ;
  static uint32_t micros() ;
  static void pinMode(uint8_t pin, uint8_t output) ;
  static void digitalWrite(uint8_t pin, uint8_t out) ;
  static uint8_t digitalRead(uint8_t pin) ;
 static void read_control_switch() ;
  static void plane_read_control_switch() ;
  static uint8_t readSwitch(void);
  static void reset_control_switch() ;
 static uint8_t read_3pos_switch(int16_t radio_in);
 static void read_aux_switches() ;
 static void init_aux_switches() ;
 static void do_aux_switch_function(int8_t ch_function, uint8_t ch_flag) ;
 static void save_trim() ;
 static void auto_trim() ;
 void crash_check() ;
 static void get_roll_pitch_drift() ;
 static void get_yaw_drift() ;
 static void failsafe_radio_on_event() ;
 static void failsafe_radio_off_event() ;
  static void failsafe_battery_event(void) ;
 static void failsafe_gps_check() ;
 static void failsafe_gps_off_event(void) ;
 static void failsafe_gcs_check() ;
 static void failsafe_gcs_off_event(void) ;
  static void failsafe_short_on_event(enum failsafe_state fstype) ;
  static void failsafe_long_on_event(enum failsafe_state fstype) ;
  static void failsafe_short_off_event() ;
  void low_battery_event(void) ;
  static void update_events() ;
 void failsafe_enable() ;
 void failsafe_disable() ;
 void failsafe_check() ;
 void fence_check() ;
 static void fence_send_mavlink_status(mavlink_channel_t chan) ;
  void init_flip() ;
  void roll_flip() ;
 static Vector2l get_fence_point_with_index(unsigned i) ;
 static void set_fence_point_with_index(Vector2l &point, unsigned i) ;
 static void geofence_load(void) ;
 static bool geofence_enabled(void) ;
 static bool geofence_check_minalt(void) ;
 static bool geofence_check_maxalt(void) ;
 static void geofence_check(bool altitude_check_only) ;
 static bool geofence_stickmixing(void) ;
 static void geofence_send_status(mavlink_channel_t chan) ;
 bool geofence_breached(void) ;
  static void geofence_check(bool altitude_check_only) ;
 static bool geofence_stickmixing(void) ;
 static bool geofence_enabled(void) ;
 static void heli_init() ;
 static int16_t get_pilot_desired_collective(int16_t control_in) ;
 static void check_dynamic_flight(void) ;
 static void heli_integrated_swash_controller(int32_t target_roll_rate, int32_t target_pitch_rate) ;
  static int16_t get_heli_rate_yaw(int32_t target_rate) ;
 static void heli_update_landing_swash() ;
 static void heli_update_rotor_speed_targets() ;
 static void read_inertia() ;
 static void read_inertial_altitude() ;
 static void update_notify() ;
 static void arm_motors_check() ;
 static void auto_disarm_check() ;
 static void init_arm_motors() ;
 static void pre_arm_checks(bool display_failure) ;
 static void pre_arm_rc_checks() ;
 static bool pre_arm_gps_checks(bool display_failure) ;
 static bool arm_checks(bool display_failure) ;
 static void init_disarm_motors() ;
 static void set_servos_4() ;
 static void servo_write(uint8_t ch, uint16_t pwm) ;
 static void run_nav_updates(void) ;
 static void calc_position();
 static void calc_distance_and_bearing() ;
 static void run_autopilot() ;
 static bool set_nav_mode(uint8_t new_nav_mode) ;
 static void update_nav_mode() ;
 static void reset_nav_params(void) ;
 static int32_t get_yaw_slew(int32_t current_yaw, int32_t desired_yaw, int16_t deg_per_sec) ;
 static void circle_set_center(const Vector3f current_position, float heading_in_radians) ;
 static void update_circle() ;
 void perf_info_reset() ;
 void perf_info_check_loop_time(uint32_t time_in_micros) ;
 uint16_t perf_info_get_num_loops() ;
 uint32_t perf_info_get_max_time() ;
 uint16_t perf_info_get_num_long_running() ;
 static void set_nav_controller(void) ;
 static void loiter_angle_reset(void) ;
 static void loiter_angle_update(void) ;
 static void navigate() ;
   static void calc_airspeed_errors() ;
  static void calc_gndspeed_undershoot() ;
  static void calc_altitude_error() ;
  static void update_loiter() ;
 static void update_cruise() ;
 static void update_fbwb_speed_height(void) ;
  static void setup_glide_slope(void) ;
 static float relative_altitude(void) ;
 static int32_t relative_altitude_abs_cm(void) ;
 Vector3f pv_latlon_to_vector(int32_t lat, int32_t lon, int32_t alt) ;
 Vector3f pv_location_to_vector(Location loc) ;
 int32_t pv_get_lat(const Vector3f pos_vec) ;
 int32_t pv_get_lon(const Vector3f &pos_vec) ;
 float pv_get_horizontal_distance_cm(const Vector3f &origin, const Vector3f &destination) ;
 float pv_get_bearing_cd(const Vector3f &origin, const Vector3f &destination) ;
  static void set_control_channels(void) ;
  static void default_dead_zones() ;
  static void init_rc_in() ;
 static void init_rc_out() ;
 static void rudder_arm_check()  ;
 void output_min() ;
 static void read_radio() ;
  static void plane_read_radio() ;
  static void control_failsafe(uint16_t pwm) ;
 static void set_throttle_and_failsafe(uint16_t throttle_pwm) ;
 void aux_servos_update_fn() ;
  static void trim_control_surfaces() ;
  static void trim_radio() ;
  static bool throttle_failsafe_level(void) ;
 static bool get_rally_point_with_index(unsigned i, RallyLocation &ret) ;
 static bool set_rally_point_with_index(unsigned i, const RallyLocation &rallyLoc) ;
 static bool find_best_rally_point(const Location &myloc, const Location &homeloc, RallyLocation &ret)  ;
 static Location rally_location_to_location(const RallyLocation &r_loc, const Location &homeloc)  ;
 static Location rally_find_best_location(const Location &myloc, const Location &homeloc) ;
 static void init_sonar(void) ;
  static void init_barometer(bool full_calibration) ;
 static int32_t read_barometer(void) ;
  static int32_t plane_read_barometer(void) ;
 static void read_airspeed(void) ;
  static void zero_airspeed(void) ;
 static int16_t read_sonar(void) ;
   static void init_compass() ;
  static void init_optflow() ;
 static void read_battery(void) ;
 static void plane_read_battery(void) ;
 void read_receiver_rssi(void) ;
  static int32_t adjusted_altitude_cm(void) ;
 static void display_compassmot_info(Vector3f& motor_impact, Vector3f& motor_compensation) ;
  static void report_batt_monitor() ;
  static void report_sonar() ;
  static void report_frame() ;
  static void report_radio() ;
  static void report_ins() ;
  static void report_compass() ;
  static void report_flight_modes() ;
  static void plane_report_flight_modes() ;
  void report_optflow()  ;
  static void print_radio_values() ;
  static void print_switch(uint8_t p, uint8_t m, bool b) ;
  static void plane_print_switch(uint8_t p, uint8_t m) ;
  static void print_done() ;
  static void zero_eeprom(void) ;
  static void print_accel_offsets_and_scaling(void) ;
  static void print_gyro_offsets(void) ;
  static void print_blanks(int16_t num) ;
  static void print_divider(void) ;
  static void print_enabled(bool b) ;
  static void init_esc()  ;
  static void report_version() ;
  static void report_tuning() ;
  static int8_t radio_input_switch(void) ;
  static void init_ardupilot() ;
 static void startup_ground() ;
 static bool set_mode(uint8_t mode) ;
  static void plane_set_mode(enum FlightMode mode) ;
 static bool GPS_ok() ;
 static bool mode_requires_GPS(uint8_t mode) ;
 static bool manual_flight_mode(uint8_t mode) ;
 static void update_auto_armed() ;
  static void check_long_failsafe() ;
  static void check_short_failsafe() ;
  static void resetPerfData(void) ;
  static void print_comma(void) ;
 static bool should_log(uint32_t mask) ;
 static uint32_t map_baudrate(int8_t rate, uint32_t default_baud) ;
  static void check_usb_mux(void) ;
 uint16_t board_voltage(void) ;
  static void print_hit_enter() ;
#line 160 "/d/Copter/GIT/ArduHybrid/ArduHybrid/ArduHybrid.pde"
bool isplane = true;
bool oldisplane = isplane;
bool transit;
uint16_t vtolservo1;
unsigned long transittimer;

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
static void gcs_send_text_fmt(const prog_char_t *fmt, ...);
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
static const AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_100HZ;

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
static GPS_Glitch   gps_glitch(g_gps);

// flight modes convenience array
static AP_Int8 *flight_modes = &g.flight_mode1;
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
// SONAR selection
////////////////////////////////////////////////////////////////////////////////
ModeFilterInt16_Size3 sonar_mode_filter(1);
#if CONFIG_SONAR == ENABLED
static AP_HAL::AnalogSource *sonar_analog_source;
static AP_RangeFinder_MaxsonarXL *sonar;
#endif

////////////////////////////////////////////////////////////////////////////////
// User variables
////////////////////////////////////////////////////////////////////////////////
#ifdef USERHOOK_VARIABLES
 #include USERHOOK_VARIABLES
#endif

////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////

/* Radio values
 *               Channel assignments
 *                       1	Ailerons (rudder if no ailerons)
 *                       2	Elevator
 *                       3	Throttle
 *                       4	Rudder (if we have ailerons)
 *                       5	Mode - 3 position switch
 *                       6  User assignable
 *                       7	trainer switch - sets throttle nominal (toggle switch), sets accels to Level (hold > 1 second)
 *                       8	TBD
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
// There are multiple states defined such as STABILIZE, ACRO,
static int8_t control_mode = STABILIZE;
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
static struct {
    uint8_t rc_override_active  : 1; // 0   // true if rc control are overwritten by ground station
    uint8_t radio               : 1; // 1   // A status flag for the radio failsafe
    uint8_t battery             : 1; // 2   // A status flag for the battery failsafe
    uint8_t gps                 : 1; // 3   // A status flag for the gps failsafe
    uint8_t gcs                 : 1; // 4   // A status flag for the ground station failsafe
    int8_t radio_counter;                  // number of iterations with throttle below throttle_fs_value
    uint32_t last_heartbeat_ms;             // the time when the last HEARTBEAT message arrived from a GCS - used for triggering gcs failsafe
} failsafe;

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
// We need to scale the longitude up to make these calcs work
// to account for decreasing distance between lines of longitude away from the equator
static float scaleLongUp = 1;
// Sometimes we need to remove the scaling for distance calcs
static float scaleLongDown = 1;
// A counter used to count down valid gps fixes to allow the gps estimate to settle
// before recording our home position (and executing a ground start if we booted with an air start)
static uint8_t ground_start_count      = 10;
// Used to compute a speed estimate from the first valid gps fixes to decide if we are
// on the ground or in the air.  Used to decide if a ground start is appropriate if we
// booted with an air start.
static int16_t ground_start_avg;		// Plane
// true if we have a position estimate from AHRS
static bool have_position;				// Plane

////////////////////////////////////////////////////////////////////////////////
// Location & Navigation
////////////////////////////////////////////////////////////////////////////////
// This is the angle from the copter to the next waypoint in centi-degrees
static int32_t wp_bearing;
// The original bearing to the next waypoint.  used to point the nose of the copter at the next waypoint
static int32_t original_wp_bearing;
// The location of home in relation to the copter in centi-degrees
static int32_t home_bearing;
// distance between plane and home in cm
static int32_t home_distance;
// distance between plane and next waypoint in cm.
static uint32_t wp_distance;
// Plane Distance between plane and next waypoint.  
static uint32_t plane_wp_distance;								// Plane
// navigation mode - options include NAV_NONE, NAV_LOITER, NAV_CIRCLE, NAV_WP
static uint8_t nav_mode;
// Register containing the index of the current navigation command in the mission script
static int16_t command_nav_index;
// There may be two active commands in Auto mode.
// This indicates the active navigation command by index number
static uint8_t nav_command_index; 								// Plane
// This indicates the active non-navigation command by index number
static uint8_t non_nav_command_index;							// Plane
// Register containing the index of the previous navigation command in the mission script
// Used to manage the execution of conditional commands
static uint8_t prev_nav_index;
// Register containing the index of the current conditional command in the mission script
static uint8_t command_cond_index;
// Used to track the required WP navigation information
// options include
// NAV_ALTITUDE - have we reached the desired altitude?
// NAV_LOCATION - have we reached the desired location?
// NAV_DELAY    - have we waited at the waypoint the desired time?
static float lon_error, lat_error;      // Used to report how many cm we are from the next waypoint or loiter target position
static int16_t control_roll;            // desired roll angle of copter in centi-degrees
static int16_t control_pitch;           // desired pitch angle of copter in centi-degrees
static uint8_t rtl_state;               // records state of rtl (initial climb, returning home, etc)
static uint8_t land_state;              // records state of land (flying to location, descending)
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
// Holds the current loaded command from the EEPROM for navigation
static struct   Location command_nav_queue;
// Holds the current loaded command from the EEPROM for conditional scripts
static struct   Location command_cond_queue;

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
// An additional throttle added to keep the copter at the same altitude when banking
static int16_t angle_boost;
// counter to verify landings
static uint16_t land_detector;
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
// Loiter timer - Records how long we have been in loiter
static uint32_t rtl_loiter_start_time;


// Used to exit the roll and pitch auto trim function
static uint8_t auto_trim_counter;

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
// SIMPLE Mode
////////////////////////////////////////////////////////////////////////////////
// Used to track the orientation of the copter for Simple mode. This value is reset at each arming
// or in SuperSimple mode when the copter leaves a 20m radius from home.
static float simple_cos_yaw = 1.0;
static float simple_sin_yaw;
static int32_t super_simple_last_bearing;
static float super_simple_cos_yaw = 1.0;
static float super_simple_sin_yaw;

// Stores initial bearing when armed - initial simple bearing is modified in super simple mode so not suitable
static int32_t initial_armed_bearing;

////////////////////////////////////////////////////////////////////////////////
// Rate contoller targets
////////////////////////////////////////////////////////////////////////////////
static uint8_t rate_targets_frame = EARTH_FRAME;    // indicates whether rate targets provided in earth or body frame
static int32_t roll_rate_target_ef;
static int32_t pitch_rate_target_ef;
static int32_t yaw_rate_target_ef;
static int32_t roll_rate_target_bf;     // body frame roll rate target
static int32_t pitch_rate_target_bf;    // body frame pitch rate target
static int32_t yaw_rate_target_bf;      // body frame yaw rate target

////////////////////////////////////////////////////////////////////////////////
// Throttle variables
////////////////////////////////////////////////////////////////////////////////
static int16_t throttle_accel_target_ef;    // earth frame throttle acceleration target
static bool throttle_accel_controller_active;   // true when accel based throttle controller is active, false when higher level throttle controllers are providing throttle output directly
static float throttle_avg;                  // g.throttle_cruise as a float
static int16_t desired_climb_rate;          // pilot desired climb rate - for logging purposes only
static float target_alt_for_reporting;      // target altitude in cm for reporting (logs and ground station)

////////////////////////////////////////////////////////////////////////////////
// ACRO Mode
////////////////////////////////////////////////////////////////////////////////
// Used to control Axis lock
static int32_t acro_roll;                   // desired roll angle while sport mode
static int32_t acro_roll_rate;              // desired roll rate while in acro mode
static int32_t acro_pitch;                  // desired pitch angle while sport mode
static int32_t acro_pitch_rate;             // desired pitch rate while acro mode
static int32_t acro_yaw_rate;               // desired yaw rate while acro mode
static float acro_level_mix;                // scales back roll, pitch and yaw inversely proportional to input from pilot

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

// Filters
#if FRAME_CONFIG == HELI_FRAME
//static LowPassFilterFloat rate_roll_filter;    // Rate Roll filter
//static LowPassFilterFloat rate_pitch_filter;   // Rate Pitch filter
#endif // HELI_FRAME

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
Vector3f circle_center;     // circle position expressed in cm from home location.  x = lat, y = lon
// angle from the circle center to the copter's desired location.  Incremented at circle_rate / second
static float circle_angle;
// the total angle (in radians) travelled
static float circle_angle_total;
// deg : how many times to circle as specified by mission command
static uint8_t circle_desired_rotations;
static float circle_angular_acceleration;       // circle mode's angular acceleration
static float circle_angular_velocity;           // circle mode's angular velocity
static float circle_angular_velocity_max;       // circle mode's max angular velocity
// How long we should stay in Loiter Mode for mission scripting (time in seconds)
static uint16_t loiter_time_max;
// How long have we been loitering - The start time in millis
static uint32_t loiter_time;

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
// CH7 and CH8 save waypoint control
////////////////////////////////////////////////////////////////////////////////
// This register tracks the current Mission Command index when writing
// a mission using Ch7 or Ch8 aux switches in flight
static int8_t aux_switch_wp_index;

////////////////////////////////////////////////////////////////////////////////
// Altitude
////////////////////////////////////////////////////////////////////////////////
// The (throttle) controller desired altitude in cm
static float controller_desired_alt;
// The cm we are off in altitude from next_WP.alt – Positive value means we are below the WP
static int32_t altitude_error;
// The cm/s we are moving up or down based on filtered data - Positive = UP
static int16_t climb_rate;
// The altitude as reported by Sonar in cm – Values are 20 to 700 generally.
static int16_t sonar_alt;
static uint8_t sonar_alt_health;   // true if we can trust the altitude from the sonar
static float target_sonar_alt;      // desired altitude in cm above the ground
// The altitude as reported by Baro in cm – Values can be quite high
static int32_t baro_alt;

// Difference between current altitude and desired altitude.  Centimeters
static int32_t altitude_error_cm;			// Plane
// The current desired altitude.  Altitude is linearly ramped between waypoints.  Centimeters
static int32_t target_altitude_cm;			// Plane
// Altitude difference between previous and current waypoint.  Centimeters
static int32_t offset_altitude_cm;			// Plane

////////////////////////////////////////////////////////////////////////////////
// flight modes
////////////////////////////////////////////////////////////////////////////////
// Flight modes are combinations of Roll/Pitch, Yaw and Throttle control modes
// Each Flight mode is a unique combination of these modes
//
// The current desired control scheme for Yaw
static uint8_t yaw_mode = STABILIZE_YAW;
// The current desired control scheme for roll and pitch / navigation
static uint8_t roll_pitch_mode = STABILIZE_RP;
// The current desired control scheme for altitude hold
static uint8_t throttle_mode = STABILIZE_THR;

////////////////////////////////////////////////////////////////////////////////
// Navigation control variables
////////////////////////////////////////////////////////////////////////////////
// The instantaneous desired bank angle.  Hundredths of a degree
static int32_t nav_roll_cd;		// Plane

// The instantaneous desired pitch angle.  Hundredths of a degree
static int32_t nav_pitch_cd;	// Plane

////////////////////////////////////////////////////////////////////////////////
// Navigation Roll/Pitch functions
////////////////////////////////////////////////////////////////////////////////
// The Commanded ROll from the autopilot based on optical flow sensor.
static int32_t of_roll;
// The Commanded pitch from the autopilot based on optical flow sensor. negative Pitch means go forward.
static int32_t of_pitch;

////////////////////////////////////////////////////////////////////////////////
// Navigation Throttle control
////////////////////////////////////////////////////////////////////////////////
// The Commanded Throttle from the autopilot.
static int16_t nav_throttle;    // 0-1000 for throttle control
// This is a simple counter to track the amount of throttle used during flight
// This could be useful later in determining and debuging current usage and predicting battery life
static uint32_t throttle_integrator;

////////////////////////////////////////////////////////////////////////////////
// Navigation Yaw control
////////////////////////////////////////////////////////////////////////////////
// The Commanded Yaw from the autopilot.
static int32_t control_yaw;
// Yaw will point at this location if yaw_mode is set to YAW_LOOK_AT_LOCATION
static Vector3f yaw_look_at_WP;
// bearing from current location to the yaw_look_at_WP
static int32_t yaw_look_at_WP_bearing;
// yaw used for YAW_LOOK_AT_HEADING yaw_mode
static int32_t yaw_look_at_heading;
// Deg/s we should turn
static int16_t yaw_look_at_heading_slew;

////////////////////////////////////////////////////////////////////////////////
// Inertial Navigation
////////////////////////////////////////////////////////////////////////////////
static AP_InertialNav inertial_nav(&ahrs, &barometer, g_gps, gps_glitch);

////////////////////////////////////////////////////////////////////////////////
// Waypoint navigation object
// To-Do: move inertial nav up or other navigation variables down here
////////////////////////////////////////////////////////////////////////////////
static AC_WPNav wp_nav(&inertial_nav, &ahrs, &g.pi_loiter_lat, &g.pi_loiter_lon, &g.pid_loiter_rate_lat, &g.pid_loiter_rate_lon);

////////////////////////////////////////////////////////////////////////////////
// Performance monitoring
////////////////////////////////////////////////////////////////////////////////
// Timer used to accrue data and trigger recording of the performanc monitoring log message
static uint32_t perf_mon_timer;
static int16_t pmTest1;
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

////////////////////////////////////////////////////////////////////////////////
// AC_Fence library to reduce fly-aways
////////////////////////////////////////////////////////////////////////////////
#if AC_FENCE == ENABLED
AC_Fence    fence(&inertial_nav);
#endif

////////////////////////////////////////////////////////////////////////////////
// Crop Sprayer
////////////////////////////////////////////////////////////////////////////////
#if SPRAYER == ENABLED
static AC_Sprayer sprayer(&inertial_nav);
#endif

////////////////////////////////////////////////////////////////////////////////
// EPM Cargo Griper
////////////////////////////////////////////////////////////////////////////////
#if EPM_ENABLED == ENABLED
static AP_EPM epm;
#endif

////////////////////////////////////////////////////////////////////////////////
// function definitions to keep compiler from complaining about undeclared functions
////////////////////////////////////////////////////////////////////////////////
void get_throttle_althold(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate);
static void pre_arm_checks(bool display_failure);

////////////////////////////////////////////////////////////////////////////////
// Optical flow sensor
////////////////////////////////////////////////////////////////////////////////
 #if OPTFLOW == ENABLED
static AP_OpticalFlow_ADNS3080 optflow;
 #endif
////////////////////////////////////////////////////////////////////////////////
// Motor Output
////////////////////////////////////////////////////////////////////////////////
#if FRAME_CONFIG == QUAD_FRAME
 #define MOTOR_CLASS AP_MotorsQuad
#elif FRAME_CONFIG == TRI_FRAME
 #define MOTOR_CLASS AP_MotorsTri
#elif FRAME_CONFIG == HEXA_FRAME
 #define MOTOR_CLASS AP_MotorsHexa
#elif FRAME_CONFIG == Y6_FRAME
 #define MOTOR_CLASS AP_MotorsY6
#elif FRAME_CONFIG == OCTA_FRAME
 #define MOTOR_CLASS AP_MotorsOcta
#elif FRAME_CONFIG == OCTA_QUAD_FRAME
 #define MOTOR_CLASS AP_MotorsOctaQuad
#elif FRAME_CONFIG == HELI_FRAME
 #define MOTOR_CLASS AP_MotorsHeli
#elif FRAME_CONFIG == SINGLE_FRAME
 #define MOTOR_CLASS AP_MotorsSingle
#else
 #error Unrecognised frame type
#endif

#if FRAME_CONFIG == HELI_FRAME  // helicopter constructor requires more arguments
static MOTOR_CLASS motors(&g.rc_1, &g.rc_2, &g.rc_3, &g.rc_4, &g.rc_7, &g.rc_8, &g.heli_servo_1, &g.heli_servo_2, &g.heli_servo_3, &g.heli_servo_4);
#elif FRAME_CONFIG == TRI_FRAME  // tri constructor requires additional rc_7 argument to allow tail servo reversing
static MOTOR_CLASS motors(&g.rc_1, &g.rc_2, &g.rc_3, &g.rc_4, &g.rc_7);
#elif FRAME_CONFIG == SINGLE_FRAME  // single constructor requires extra servos for flaps
static MOTOR_CLASS motors(&g.rc_1, &g.rc_2, &g.rc_3, &g.rc_4, &g.single_servo_1, &g.single_servo_2, &g.single_servo_3, &g.single_servo_4);
#else
static MOTOR_CLASS motors(&g.rc_1, &g.rc_2, &g.rc_3, &g.rc_4);
#endif

////////////////////////////////////////////////////////////////////////////////
// PIDs
////////////////////////////////////////////////////////////////////////////////

// This is a convienience accessor for the IMU roll rates. It's currently the raw IMU rates
// and not the adjusted omega rates, but the name is stuck
static Vector3f omega;

// This is used to hold radio tuning values for in-flight CH6 tuning
float tuning_value;
////////////////////////////////////////////////////////////////////////////////
// Orientation
////////////////////////////////////////////////////////////////////////////////
// Convienience accessors for commonly used trig functions. These values are generated
// by the DCM through a few simple equations. They are used throughout the code where cos and sin
// would normally be used.
// The cos values are defaulted to 1 to get a decent initial value for a level state
static float cos_roll_x         = 1.0;
static float cos_pitch_x        = 1.0;
static float cos_yaw            = 1.0;
static float sin_yaw;
static float sin_roll;
static float sin_pitch;

////////////////////////////////////////////////////////////////////////////////
// Top-level logic
////////////////////////////////////////////////////////////////////////////////

// setup the var_info table
AP_Param param_loader(var_info, WP_START_BYTE);
/*
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 10ms units) and the maximum time they are expected to take (in
  microseconds)
 */
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
    { throttle_loop,         2,     450 },
    { update_GPS,            2,     900 },
    { update_nav_mode,       1,     400 },
    { update_batt_compass,  10,     720 },
    { read_aux_switches,    10,      50 },
    { arm_motors_check,     10,      10 },
    { auto_trim,            10,     140 },
    { update_altitude,      10,    1000 },
    { run_nav_updates,      10,     800 },
    { three_hz_loop,        33,      90 },
    { compass_accumulate,    2,     420 },
    { barometer_accumulate,  2,     250 },
#if FRAME_CONFIG == HELI_FRAME
    { check_dynamic_flight,  2,     100 },
#endif
    { update_notify,         2,     100 },
    { one_hz_loop,         100,     420 },
    { crash_check,          10,      20 },
    { gcs_check_input,	     2,     550 },
    { gcs_send_heartbeat,  100,     150 },
    { gcs_send_deferred,     2,     720 },
    { gcs_data_stream_send,  2,     950 },
    { update_mount,          2,     450 },
#if LOGGING_ENABLED == ENABLED
    { ten_hz_logging_loop,  10,     300 },
#endif
    { fifty_hz_logging_loop, 2,     220 },
    { perf_update,        1000,     200 },
    { read_receiver_rssi,   10,      50 },
#ifdef USERHOOK_FASTLOOP
    { userhook_FastLoop,     1,    100  },
#endif
#ifdef USERHOOK_50HZLOOP
    { userhook_50Hz,         2,    100  },
#endif
#ifdef USERHOOK_MEDIUMLOOP
    { userhook_MediumLoop,   10,    100 },
#endif
#ifdef USERHOOK_SLOWLOOP
    { userhook_SlowLoop,     30,    100 },
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    { userhook_SuperSlowLoop,100,   100 },
#endif
};

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
#if LOGGING_ENABLED == ENABLED
    { update_logging1,        5,   1700 },
    { update_logging2,        5,   1700 },
#endif
};



void setup() {

		if (!isplane){
			AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_100HZ;
		}
		if (isplane){
			AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_50HZ;
		}
		
    cliSerial = hal.console;

    // Load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

    AP_Notify::flags.failsafe_battery = false;

    init_ardupilot();

    // initialise the main loop scheduler
		if (!isplane){
			scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
		}
		if (isplane){
			scheduler.init(&plane_scheduler_tasks[0], sizeof(plane_scheduler_tasks)/sizeof(plane_scheduler_tasks[0]));
		}
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

static void perf_update(void)
{
    if (g.log_bitmask & MASK_LOG_PM)
        Log_Write_Performance();
    if (scheduler.debug()) {
        cliSerial->printf_P(PSTR("PERF: %u/%u %lu\n"),
                            (unsigned)perf_info_get_num_long_running(),
                            (unsigned)perf_info_get_num_loops(),
                            (unsigned long)perf_info_get_max_time());
    }
    perf_info_reset();
    pmTest1 = 0;
}

void loop()
{ 
    uint16_t vtolservo = hal.rcin->read(4);
			
 
 if(vtolservo<1500 && isplane == false && transit == 0){
	gcs_send_text_fmt(PSTR("vtolservo %d"),vtolservo);
	transit=1;
	gcs_send_text_fmt(PSTR("CHANGE < 1500  change to transit"));
	RC_Channel_aux::set_radio(RC_Channel_aux::k_hybridservo, g.hybridservo_transition);
	transittimer = millis();
	}

	
 if(vtolservo<1500 && isplane == false && transit == 1){
	if (millis() > (transittimer + g.transitionspeed)){
	set_control_channels();
	gcs_send_text_fmt(PSTR("CHANGE > 1500  change to plane"));
	transit = 0;isplane=true;
	RC_Channel_aux::set_radio(RC_Channel_aux::k_hybridservo, g.hybridservo_plane);
	}
	}
 
 if(vtolservo>1500 && (isplane == true || transit == 1)){
	init_rc_in();
	plane_gcs_send_text_fmt(PSTR("vtolservo %d"),vtolservo);
	isplane=false;transit=0;
	plane_gcs_send_text_fmt(PSTR("CHANGE > 1500 change to copter"));
	RC_Channel_aux::set_radio(RC_Channel_aux::k_hybridservo, g.hybridservo_copter);
	}
  

	if (isplane != oldisplane){
		if (!isplane){
			AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_100HZ;
			scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
		}
		if (isplane){
			AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_50HZ;
			scheduler.init(&plane_scheduler_tasks[0], sizeof(plane_scheduler_tasks)/sizeof(plane_scheduler_tasks[0]));
		}
		oldisplane = isplane;
	}

    // wait for an INS sample
    if (!ins.wait_for_sample(1000)) {
        Log_Write_Error(ERROR_SUBSYSTEM_MAIN, ERROR_CODE_MAIN_INS_DELAY);
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

	if (!isplane){
    // Execute the fast loop
    fast_loop();
	}
	
    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
if (!isplane){
	uint32_t time_available = (timer + 10000) - micros();
    scheduler.run(time_available - 300);
}
	
if (isplane){
    uint32_t time_available = (timer + 20000) - micros();
    if (time_available > 19500) { time_available = 19500; }
    scheduler.run(time_available);
}
}

// END OF MAIN LOOP

// Main loop - 100hz
static void fast_loop()
{

    // IMU DCM Algorithm
    // --------------------
    read_AHRS();

    // reads all of the necessary trig functions for cameras, throttle, etc.
    // --------------------------------------------------------------------
    update_trig();

#if FLIP == ENABLED
	// Acrobatic control
    if (ap.do_flip) {
        if(abs(g.rc_1.control_in) < 4000) {
            // calling roll_flip will override the desired roll rate and throttle output
            roll_flip();
        }else{
            // force an exit from the loop if we are not hands off sticks.
            ap.do_flip = false;
            Log_Write_Event(DATA_EXIT_FLIP);
        }
    }
#endif
	
    // run low level rate controllers that only require IMU data
    run_rate_controllers();

    // write out the servo PWM values
    // ------------------------------
    set_servos_4();

    // Inertial Nav
    // --------------------
    read_inertia();

    // optical flow
    // --------------------
#if OPTFLOW == ENABLED
    if(g.optflow_enabled) {
        update_optical_flow();
    }
#endif  // OPTFLOW == ENABLED

    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    read_control_switch();

    // custom code/exceptions for flight modes
    // ---------------------------------------
    update_yaw_mode();
    update_roll_pitch_mode();

    // update targets to rate controllers
    update_rate_contoller_targets();
}

// throttle_loop - should be run at 50 hz
// ---------------------------
static void throttle_loop()
{
    // get altitude and climb rate from inertial lib
    read_inertial_altitude();

    // Update the throttle ouput
    // -------------------------
    update_throttle_mode();

    // check if we've landed
    update_land_detector();

    // check auto_armed status
    update_auto_armed();

#if FRAME_CONFIG == HELI_FRAME
    // update rotor speed
    heli_update_rotor_speed_targets();

    // update trad heli swash plate movement
    heli_update_landing_swash();
#endif
}

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

// update_batt_compass - read battery and compass
// should be called at 10hz
static void update_batt_compass(void)
{
    // read battery before compass because it may be used for motor interference compensation
    read_battery();

#if HIL_MODE != HIL_MODE_ATTITUDE  // don't execute in HIL mode
    if(g.compass_enabled) {
        if (compass.read()) {
            compass.null_offsets();
        }
        // log compass information
        if (g.log_bitmask & MASK_LOG_COMPASS) {
            Log_Write_Compass();
        }
    }
#endif

    // record throttle output
    throttle_integrator += g.rc_3.servo_out;
}

#if LOGGING_ENABLED == ENABLED
// ten_hz_logging_loop
// should be run at 10hz
static void ten_hz_logging_loop()
{
    if (g.log_bitmask & MASK_LOG_ATTITUDE_MED) {
        Log_Write_Attitude();
    }
    if (g.log_bitmask & MASK_LOG_RCIN) {
        DataFlash.Log_Write_RCIN();
    }
    if (g.log_bitmask & MASK_LOG_RCOUT) {
        DataFlash.Log_Write_RCOUT();
    }
}
#endif

// fifty_hz_logging_loop
// should be run at 50hz
static void fifty_hz_logging_loop()
{
#if HIL_MODE != HIL_MODE_DISABLED
    // HIL for a copter needs very fast update of the servo values
    gcs_send_message(MSG_RADIO_OUT);
#endif
#if LOGGING_ENABLED == ENABLED
#if HIL_MODE == HIL_MODE_DISABLED
    if (g.log_bitmask & MASK_LOG_ATTITUDE_FAST) {
        Log_Write_Attitude();
    }

    if (g.log_bitmask & MASK_LOG_IMU) {
        DataFlash.Log_Write_IMU(ins);
    }
#endif
#endif
}

// three_hz_loop - 3.3hz loop
static void three_hz_loop()
{
    // check if we've lost contact with the ground station
    failsafe_gcs_check();

#if AC_FENCE == ENABLED
    // check if we have breached a fence
    fence_check();
#endif // AC_FENCE_ENABLED

#if SPRAYER == ENABLED
    sprayer.update();
#endif

    update_events();

    if(g.radio_tuning > 0)
        tuning();
}

// one_hz_loop - runs at 1Hz
static void one_hz_loop()
{
    if (g.log_bitmask != 0) {
        Log_Write_Data(DATA_AP_STATE, ap.value);
    }

    // pass latest alt hold kP value to navigation controller
    wp_nav.set_althold_kP(g.pi_alt_hold.kP());

    // update latest lean angle to navigation controller
    wp_nav.set_lean_angle_max(g.angle_max);

    // log battery info to the dataflash
    if (g.log_bitmask & MASK_LOG_CURRENT) {
        Log_Write_Current();
    }

    // perform pre-arm checks & display failures every 30 seconds
    static uint8_t pre_arm_display_counter = 15;
    pre_arm_display_counter++;
    if (pre_arm_display_counter >= 30) {
        pre_arm_checks(true);
        pre_arm_display_counter = 0;
    }else{
        pre_arm_checks(false);
    }

    // auto disarm checks
    auto_disarm_check();

    if (!motors.armed()) {
        // make it possible to change ahrs orientation at runtime during initial config
        ahrs.set_orientation();

        // check the user hasn't updated the frame orientation
        motors.set_frame_orientation(g.frame_orientation);
    }

    // update assigned functions and enable auxiliar servos
    aux_servos_update_fn();
    enable_aux_servos();

#if MOUNT == ENABLED
    camera_mount.update_mount_type();
#endif

#if MOUNT2 == ENABLED
    camera_mount2.update_mount_type();
#endif

    check_usb_mux();
}

// called at 100hz but data from sensor only arrives at 20 Hz
#if OPTFLOW == ENABLED
static void update_optical_flow(void)
{
    static uint32_t last_of_update = 0;
    static uint8_t of_log_counter = 0;

    // if new data has arrived, process it
    if( optflow.last_update != last_of_update ) {
        last_of_update = optflow.last_update;
        optflow.update_position(ahrs.roll, ahrs.pitch, sin_yaw, cos_yaw, current_loc.alt);      // updates internal lon and lat with estimation based on optical flow

        // write to log at 5hz
        of_log_counter++;
        if( of_log_counter >= 4 ) {
            of_log_counter = 0;
            if (g.log_bitmask & MASK_LOG_OPTFLOW) {
                Log_Write_Optflow();
            }
        }
    }
}
#endif  // OPTFLOW == ENABLED

// called at 50hz
static void update_GPS(void)
{
    static uint32_t last_gps_reading;           // time of last gps message
    static uint8_t ground_start_count = 10;     // counter used to grab at least 10 reads before commiting the Home location
    bool report_gps_glitch;

    g_gps->update();

    // logging and glitch protection run after every gps message
    if (g_gps->last_message_time_ms() != last_gps_reading) {
        last_gps_reading = g_gps->last_message_time_ms();

        // log GPS message
        if (g.log_bitmask & MASK_LOG_GPS) {
            DataFlash.Log_Write_GPS(g_gps, current_loc.alt);
        }

        // run glitch protection and update AP_Notify if home has been initialised
        if (ap.home_is_set) {
            gps_glitch.check_position();
            report_gps_glitch = (gps_glitch.glitching() && !ap.usb_connected);
            if (AP_Notify::flags.gps_glitching != report_gps_glitch) {
                if (gps_glitch.glitching()) {
                    Log_Write_Error(ERROR_SUBSYSTEM_GPS, ERROR_CODE_GPS_GLITCH);
                }else{
                    Log_Write_Error(ERROR_SUBSYSTEM_GPS, ERROR_CODE_ERROR_RESOLVED);
                }
                AP_Notify::flags.gps_glitching = report_gps_glitch;
            }
        }
    }

    // checks to initialise home and take location based pictures
    if (g_gps->new_data && g_gps->status() >= GPS::GPS_OK_FIX_3D) {
        // clear new data flag
        g_gps->new_data = false;

        // check if we can initialise home yet
        if (!ap.home_is_set) {
            // if we have a 3d lock and valid location
            if(g_gps->status() >= GPS::GPS_OK_FIX_3D && g_gps->latitude != 0) {
                if( ground_start_count > 0 ) {
                    ground_start_count--;
                }else{
                    // after 10 successful reads store home location
                    // ap.home_is_set will be true so this will only happen once
                    ground_start_count = 0;
                    init_home();

                    // set system clock for log timestamps
                    hal.util->set_system_clock(g_gps->time_epoch_usec());

                    if (g.compass_enabled) {
                        // Set compass declination automatically
                        compass.set_initial_location(g_gps->latitude, g_gps->longitude);
                    }
                }
            }else{
                // start again if we lose 3d lock
                ground_start_count = 10;
            }
        }

#if CAMERA == ENABLED
        if (camera.update_location(current_loc) == true) {
            do_take_picture();
        }
#endif
    }

    // check for loss of gps
    failsafe_gps_check();
}

// set_yaw_mode - update yaw mode and initialise any variables required
bool set_yaw_mode(uint8_t new_yaw_mode)
{
    // boolean to ensure proper initialisation of throttle modes
    bool yaw_initialised = false;

    // return immediately if no change
    if( new_yaw_mode == yaw_mode ) {
        return true;
    }

    switch( new_yaw_mode ) {
        case YAW_HOLD:
            yaw_initialised = true;
            break;
        case YAW_ACRO:
            yaw_initialised = true;
            acro_yaw_rate = 0;
            break;
        case YAW_LOOK_AT_NEXT_WP:
            if( ap.home_is_set ) {
                yaw_initialised = true;
            }
            break;
        case YAW_LOOK_AT_LOCATION:
            if( ap.home_is_set ) {
                // update bearing - assumes yaw_look_at_WP has been intialised before set_yaw_mode was called
                yaw_look_at_WP_bearing = pv_get_bearing_cd(inertial_nav.get_position(), yaw_look_at_WP);
                yaw_initialised = true;
            }
            break;
        case YAW_CIRCLE:
            if( ap.home_is_set ) {
                // set yaw to point to center of circle
                yaw_look_at_WP = circle_center;
                // initialise bearing to current heading
                yaw_look_at_WP_bearing = ahrs.yaw_sensor;
                yaw_initialised = true;
            }
            break;
        case YAW_LOOK_AT_HEADING:
            yaw_initialised = true;
            break;
        case YAW_LOOK_AT_HOME:
            if( ap.home_is_set ) {
                yaw_initialised = true;
            }
            break;
        case YAW_LOOK_AHEAD:
            if( ap.home_is_set ) {
                yaw_initialised = true;
            }
            break;
#if DRIFT == ENABLED
        case YAW_DRIFT:
            yaw_initialised = true;
            break;
#endif
        case YAW_RESETTOARMEDYAW:
            control_yaw = ahrs.yaw_sensor; // store current yaw so we can start rotating back to correct one
            yaw_initialised = true;
            break;
    }

    // if initialisation has been successful update the yaw mode
    if( yaw_initialised ) {
        yaw_mode = new_yaw_mode;
    }

    // return success or failure
    return yaw_initialised;
}

// update_yaw_mode - run high level yaw controllers
// 100hz update rate
void update_yaw_mode(void)
{
    int16_t pilot_yaw = g.rc_4.control_in;

    // do not process pilot's yaw input during radio failsafe
    if (failsafe.radio) {
        pilot_yaw = 0;
    }

    switch(yaw_mode) {

    case YAW_HOLD:
        // if we are landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }
        // heading hold at heading held in control_yaw but allow input from pilot
        get_yaw_rate_stabilized_ef(pilot_yaw);
        break;

    case YAW_ACRO:
        // pilot controlled yaw using rate controller
        get_yaw_rate_stabilized_bf(pilot_yaw);
        break;

    case YAW_LOOK_AT_NEXT_WP:
        // if we are landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }else{
            // point towards next waypoint (no pilot input accepted)
            // we don't use wp_bearing because we don't want the copter to turn too much during flight
            control_yaw = get_yaw_slew(control_yaw, original_wp_bearing, AUTO_YAW_SLEW_RATE);
        }
        get_stabilize_yaw(control_yaw);

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if (pilot_yaw != 0) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

    case YAW_LOOK_AT_LOCATION:
        // if we are landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }
        // point towards a location held in yaw_look_at_WP
        get_look_at_yaw();

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if (pilot_yaw != 0) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

    case YAW_CIRCLE:
        // if we are landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }
        // points toward the center of the circle or does a panorama
        get_circle_yaw();

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if (pilot_yaw != 0) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

    case YAW_LOOK_AT_HOME:
        // if we are landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }else{
            // keep heading always pointing at home with no pilot input allowed
            control_yaw = get_yaw_slew(control_yaw, home_bearing, AUTO_YAW_SLEW_RATE);
        }
        get_stabilize_yaw(control_yaw);

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if (pilot_yaw != 0) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

    case YAW_LOOK_AT_HEADING:
        // if we are landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }else{
            // keep heading pointing in the direction held in yaw_look_at_heading with no pilot input allowed
            control_yaw = get_yaw_slew(control_yaw, yaw_look_at_heading, yaw_look_at_heading_slew);
        }
        get_stabilize_yaw(control_yaw);
        break;

	case YAW_LOOK_AHEAD:
        // if we are landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }
		// Commanded Yaw to automatically look ahead.
        get_look_ahead_yaw(pilot_yaw);
        break;

#if DRIFT == ENABLED
    case YAW_DRIFT:
        // if we have landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }
        get_yaw_drift();
        break;
#endif

    case YAW_RESETTOARMEDYAW:
        // if we are landed reset yaw target to current heading
        if (ap.land_complete) {
            control_yaw = ahrs.yaw_sensor;
        }else{
            // changes yaw to be same as when quad was armed
            control_yaw = get_yaw_slew(control_yaw, initial_armed_bearing, AUTO_YAW_SLEW_RATE);
        }
        get_stabilize_yaw(control_yaw);

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if (pilot_yaw != 0) {
            set_yaw_mode(YAW_HOLD);
        }

        break;
    }
}

// get yaw mode based on WP_YAW_BEHAVIOR parameter
// set rtl parameter to true if this is during an RTL
uint8_t get_wp_yaw_mode(bool rtl)
{
    switch (g.wp_yaw_behavior) {
        case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP:
            return YAW_LOOK_AT_NEXT_WP;
            break;

        case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL:
            if( rtl ) {
                return YAW_HOLD;
            }else{
                return YAW_LOOK_AT_NEXT_WP;
            }
            break;

        case WP_YAW_BEHAVIOR_LOOK_AHEAD:
            return YAW_LOOK_AHEAD;
            break;

        default:
            return YAW_HOLD;
            break;
    }
}

// set_roll_pitch_mode - update roll/pitch mode and initialise any variables as required
bool set_roll_pitch_mode(uint8_t new_roll_pitch_mode)
{
    // boolean to ensure proper initialisation of throttle modes
    bool roll_pitch_initialised = false;

    // return immediately if no change
    if( new_roll_pitch_mode == roll_pitch_mode ) {
        return true;
    }

    switch( new_roll_pitch_mode ) {
        case ROLL_PITCH_STABLE:
            reset_roll_pitch_in_filters(g.rc_1.control_in, g.rc_2.control_in);
            roll_pitch_initialised = true;
            break;
        case ROLL_PITCH_ACRO:
            // reset acro level rates
            acro_roll_rate = 0;
            acro_pitch_rate = 0;
            roll_pitch_initialised = true;
            break;
        case ROLL_PITCH_STABLE_OF:
#if DRIFT == ENABLED
        case ROLL_PITCH_DRIFT:
            reset_roll_pitch_in_filters(g.rc_1.control_in, g.rc_2.control_in);
            roll_pitch_initialised = true;
            break;
#endif
        case ROLL_PITCH_AUTO:
        case ROLL_PITCH_LOITER:
        case ROLL_PITCH_SPORT:
            roll_pitch_initialised = true;
            break;

#if AUTOTUNE == ENABLED
        case ROLL_PITCH_AUTOTUNE:
            // only enter autotune mode from stabilized roll-pitch mode when armed and flying
            if (roll_pitch_mode == ROLL_PITCH_STABLE && motors.armed() && !ap.land_complete) {
                reset_roll_pitch_in_filters(g.rc_1.control_in, g.rc_2.control_in);
                // auto_tune_start returns true if it wants the roll-pitch mode changed to autotune
                roll_pitch_initialised = auto_tune_start();
            }
            break;
#endif
    }

    // if initialisation has been successful update the yaw mode
    if( roll_pitch_initialised ) {
        exit_roll_pitch_mode(roll_pitch_mode);
        roll_pitch_mode = new_roll_pitch_mode;
    }

    // return success or failure
    return roll_pitch_initialised;
}

// exit_roll_pitch_mode - peforms any code required when exiting the current roll-pitch mode
void exit_roll_pitch_mode(uint8_t old_roll_pitch_mode)
{
#if AUTOTUNE == ENABLED
    if (old_roll_pitch_mode == ROLL_PITCH_AUTOTUNE) {
        auto_tune_stop();
    }
#endif
}


// update_roll_pitch_mode - run high level roll and pitch controllers
// 100hz update rate
void update_roll_pitch_mode(void)
{
    switch(roll_pitch_mode) {
    case ROLL_PITCH_ACRO:
        // copy user input for reporting purposes
        control_roll            = g.rc_1.control_in;
        control_pitch           = g.rc_2.control_in;

#if FRAME_CONFIG == HELI_FRAME
        // ACRO does not get SIMPLE mode ability
        if (motors.has_flybar()) {
            g.rc_1.servo_out = g.rc_1.control_in;
            g.rc_2.servo_out = g.rc_2.control_in;
        }else{
            acro_level_mix = constrain_float(1-max(max(abs(g.rc_1.control_in), abs(g.rc_2.control_in)), abs(g.rc_4.control_in))/4500.0, 0, 1)*cos_pitch_x;
            get_roll_rate_stabilized_bf(g.rc_1.control_in);
            get_pitch_rate_stabilized_bf(g.rc_2.control_in);
            get_acro_level_rates();
        }
#else  // !HELI_FRAME
        acro_level_mix = constrain_float(1-max(max(abs(g.rc_1.control_in), abs(g.rc_2.control_in)), abs(g.rc_4.control_in))/4500.0, 0, 1)*cos_pitch_x;
        get_roll_rate_stabilized_bf(g.rc_1.control_in);
        get_pitch_rate_stabilized_bf(g.rc_2.control_in);
        get_acro_level_rates();
#endif  // HELI_FRAME
        break;

    case ROLL_PITCH_STABLE:
        // apply SIMPLE mode transform
        update_simple_mode();

        if(failsafe.radio) {
            // don't allow copter to fly away during failsafe
            get_pilot_desired_lean_angles(0.0f, 0.0f, control_roll, control_pitch);
        } else {
            // convert pilot input to lean angles
            get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, control_roll, control_pitch);
        }

        // pass desired roll, pitch to stabilize attitude controllers
        get_stabilize_roll(control_roll);
        get_stabilize_pitch(control_pitch);

        break;

    case ROLL_PITCH_AUTO:
        // copy latest output from nav controller to stabilize controller
        control_roll = wp_nav.get_desired_roll();
        control_pitch = wp_nav.get_desired_pitch();
        get_stabilize_roll(control_roll);
        get_stabilize_pitch(control_pitch);
        break;

    case ROLL_PITCH_STABLE_OF:
        // apply SIMPLE mode transform
        update_simple_mode();

        if(failsafe.radio) {
            // don't allow copter to fly away during failsafe
            get_pilot_desired_lean_angles(0.0f, 0.0f, control_roll, control_pitch);
        } else {
            // convert pilot input to lean angles
            get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, control_roll, control_pitch);
        }

        // mix in user control with optical flow
        control_roll = get_of_roll(control_roll);
        control_pitch = get_of_pitch(control_pitch);

        // call stabilize controller
        get_stabilize_roll(control_roll);
        get_stabilize_pitch(control_pitch);
        break;

#if DRIFT == ENABLED
    case ROLL_PITCH_DRIFT:
        get_roll_pitch_drift();
        break;
#endif

    case ROLL_PITCH_LOITER:
        // apply SIMPLE mode transform
        update_simple_mode();

        if(failsafe.radio) {
            // don't allow loiter target to move during failsafe
            wp_nav.move_loiter_target(0.0f, 0.0f, 0.01f);
        } else {
            // update loiter target from user controls
            wp_nav.move_loiter_target(g.rc_1.control_in, g.rc_2.control_in, 0.01f);
        }

        // copy latest output from nav controller to stabilize controller
        control_roll = wp_nav.get_desired_roll();
        control_pitch = wp_nav.get_desired_pitch();

        get_stabilize_roll(control_roll);
        get_stabilize_pitch(control_pitch);
        break;

    case ROLL_PITCH_SPORT:
        // apply SIMPLE mode transform
        update_simple_mode();

        // copy user input for reporting purposes
        control_roll = g.rc_1.control_in;
        control_pitch = g.rc_2.control_in;
        get_roll_rate_stabilized_ef(g.rc_1.control_in);
        get_pitch_rate_stabilized_ef(g.rc_2.control_in);
        break;

#if AUTOTUNE == ENABLED
    case ROLL_PITCH_AUTOTUNE:
        // apply SIMPLE mode transform
        if(ap.simple_mode && ap.new_radio_frame) {
            update_simple_mode();
        }

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, control_roll, control_pitch);

        // pass desired roll, pitch to stabilize attitude controllers
        get_stabilize_roll(control_roll);
        get_stabilize_pitch(control_pitch);

        // copy user input for reporting purposes
        get_autotune_roll_pitch_controller(g.rc_1.control_in, g.rc_2.control_in, g.rc_4.control_in);
        break;
#endif
    }

	#if FRAME_CONFIG != HELI_FRAME
    if(g.rc_3.control_in == 0 && control_mode <= ACRO) {
        reset_rate_I();
    }
	#endif //HELI_FRAME

    if(ap.new_radio_frame) {
        // clear new radio frame info
        ap.new_radio_frame = false;
    }
}

static void
init_simple_bearing()
{
    // capture current cos_yaw and sin_yaw values
    simple_cos_yaw = cos_yaw;
    simple_sin_yaw = sin_yaw;

    // initialise super simple heading (i.e. heading towards home) to be 180 deg from simple mode heading
    super_simple_last_bearing = wrap_360_cd(ahrs.yaw_sensor+18000);
    super_simple_cos_yaw = simple_cos_yaw;
    super_simple_sin_yaw = simple_sin_yaw;

    // log the simple bearing to dataflash
    if (g.log_bitmask != 0) {
        Log_Write_Data(DATA_INIT_SIMPLE_BEARING, ahrs.yaw_sensor);
    }
}

// update_simple_mode - rotates pilot input if we are in simple mode
void update_simple_mode(void)
{
    float rollx, pitchx;

    // exit immediately if no new radio frame or not in simple mode
    if (ap.simple_mode == 0 || !ap.new_radio_frame) {
        return;
    }

    if (ap.simple_mode == 1) {
        // rotate roll, pitch input by -initial simple heading (i.e. north facing)
        rollx = g.rc_1.control_in*simple_cos_yaw - g.rc_2.control_in*simple_sin_yaw;
        pitchx = g.rc_1.control_in*simple_sin_yaw + g.rc_2.control_in*simple_cos_yaw;
    }else{
        // rotate roll, pitch input by -super simple heading (reverse of heading to home)
        rollx = g.rc_1.control_in*super_simple_cos_yaw - g.rc_2.control_in*super_simple_sin_yaw;
        pitchx = g.rc_1.control_in*super_simple_sin_yaw + g.rc_2.control_in*super_simple_cos_yaw;
    }

    // rotate roll, pitch input from north facing to vehicle's perspective
    g.rc_1.control_in = rollx*cos_yaw + pitchx*sin_yaw;
    g.rc_2.control_in = -rollx*sin_yaw + pitchx*cos_yaw;
}

// update_super_simple_bearing - adjusts simple bearing based on location
// should be called after home_bearing has been updated
void update_super_simple_bearing(bool force_update)
{
    // check if we are in super simple mode and at least 10m from home
    if(force_update || (ap.simple_mode == 2 && home_distance > SUPER_SIMPLE_RADIUS)) {
        // check the bearing to home has changed by at least 5 degrees
        if (labs(super_simple_last_bearing - home_bearing) > 500) {
            super_simple_last_bearing = home_bearing;
            float angle_rad = radians((super_simple_last_bearing+18000)/100);
            super_simple_cos_yaw = cosf(angle_rad);
            super_simple_sin_yaw = sinf(angle_rad);
        }
    }
}

// throttle_mode_manual - returns true if the throttle is directly controlled by the pilot
bool throttle_mode_manual(uint8_t thr_mode)
{
    return (thr_mode == THROTTLE_MANUAL || thr_mode == THROTTLE_MANUAL_TILT_COMPENSATED || thr_mode == THROTTLE_MANUAL_HELI);
}

// set_throttle_mode - sets the throttle mode and initialises any variables as required
bool set_throttle_mode( uint8_t new_throttle_mode )
{
    // boolean to ensure proper initialisation of throttle modes
    bool throttle_initialised = false;

    // return immediately if no change
    if( new_throttle_mode == throttle_mode ) {
        return true;
    }

    // initialise any variables required for the new throttle mode
    switch(new_throttle_mode) {
        case THROTTLE_MANUAL:
        case THROTTLE_MANUAL_TILT_COMPENSATED:
            throttle_accel_deactivate();                // this controller does not use accel based throttle controller
            altitude_error = 0;                         // clear altitude error reported to GCS
            throttle_initialised = true;
            break;

        case THROTTLE_HOLD:
        case THROTTLE_AUTO:
            controller_desired_alt = get_initial_alt_hold(current_loc.alt, climb_rate);     // reset controller desired altitude to current altitude
            wp_nav.set_desired_alt(controller_desired_alt);                                 // same as above but for loiter controller
            if (throttle_mode_manual(throttle_mode)) {  // reset the alt hold I terms if previous throttle mode was manual
                reset_throttle_I();
                set_accel_throttle_I_from_pilot_throttle(get_pilot_desired_throttle(g.rc_3.control_in));
            }
            throttle_initialised = true;
            break;

        case THROTTLE_LAND:
            reset_land_detector();  // initialise land detector
            controller_desired_alt = get_initial_alt_hold(current_loc.alt, climb_rate);   // reset controller desired altitude to current altitude
            throttle_initialised = true;
            break;

#if FRAME_CONFIG == HELI_FRAME
        case THROTTLE_MANUAL_HELI:
            throttle_accel_deactivate();                // this controller does not use accel based throttle controller
            altitude_error = 0;                         // clear altitude error reported to GCS
            throttle_initialised = true;
            break;
#endif
    }

    // update the throttle mode
    if( throttle_initialised ) {
        throttle_mode = new_throttle_mode;

        // reset some variables used for logging
        desired_climb_rate = 0;
        nav_throttle = 0;
    }

    // return success or failure
    return throttle_initialised;
}

// update_throttle_mode - run high level throttle controllers
// 50 hz update rate
void update_throttle_mode(void)
{
    int16_t pilot_climb_rate;
    int16_t pilot_throttle_scaled;

    if(ap.do_flip)     // this is pretty bad but needed to flip in AP modes.
        return;

#if FRAME_CONFIG != HELI_FRAME
    // do not run throttle controllers if motors disarmed
    if( !motors.armed() ) {
        set_throttle_out(0, false);
        throttle_accel_deactivate();    // do not allow the accel based throttle to override our command
        set_target_alt_for_reporting(0);
        return;
    }
#endif // FRAME_CONFIG != HELI_FRAME

    switch(throttle_mode) {

    case THROTTLE_MANUAL:
        // completely manual throttle
        if(g.rc_3.control_in <= 0){
            set_throttle_out(0, false);
        }else{
            // send pilot's output directly to motors
            pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);
            set_throttle_out(pilot_throttle_scaled, false);

            // update estimate of throttle cruise
			#if FRAME_CONFIG == HELI_FRAME
            update_throttle_cruise(motors.get_collective_out());
			#else
			update_throttle_cruise(pilot_throttle_scaled);
            #endif  //HELI_FRAME

            // check if we've taken off yet
            if (!ap.takeoff_complete && motors.armed()) {
                if (pilot_throttle_scaled > g.throttle_cruise) {
                    // we must be in the air by now
                    set_takeoff_complete(true);
                }
            }
        }
        set_target_alt_for_reporting(0);
        break;

    case THROTTLE_MANUAL_TILT_COMPENSATED:
        // manual throttle but with angle boost
        if (g.rc_3.control_in <= 0) {
            set_throttle_out(0, false); // no need for angle boost with zero throttle
        }else{
            pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);
            set_throttle_out(pilot_throttle_scaled, true);

            // update estimate of throttle cruise
            #if FRAME_CONFIG == HELI_FRAME
            update_throttle_cruise(motors.get_collective_out());
			#else
			update_throttle_cruise(pilot_throttle_scaled);
            #endif  //HELI_FRAME

            if (!ap.takeoff_complete && motors.armed()) {
                if (pilot_throttle_scaled > g.throttle_cruise) {
                    // we must be in the air by now
                    set_takeoff_complete(true);
                }
            }
        }
        set_target_alt_for_reporting(0);
        break;

    case THROTTLE_HOLD:
        if(ap.auto_armed) {
            // alt hold plus pilot input of climb rate
            pilot_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);

            // special handling if we have landed
            if (ap.land_complete) {
                if (pilot_climb_rate > 0) {
                    // indicate we are taking off
                    set_land_complete(false);
                    // clear i term when we're taking off
                    set_throttle_takeoff();
                }else{
                    // move throttle to minimum to keep us on the ground
                    set_throttle_out(0, false);
                    // deactivate accel based throttle controller (it will be automatically re-enabled when alt-hold controller next runs)
                    throttle_accel_deactivate();
                }
            }
            // check land_complete flag again in case it was changed above
            if (!ap.land_complete) {
                if( sonar_alt_health >= SONAR_ALT_HEALTH_MAX ) {
                    // if sonar is ok, use surface tracking
                    get_throttle_surface_tracking(pilot_climb_rate);    // this function calls set_target_alt_for_reporting for us
                }else{
                    // if no sonar fall back stabilize rate controller
                    get_throttle_rate_stabilized(pilot_climb_rate);     // this function calls set_target_alt_for_reporting for us
                }
            }
        }else{
            // pilot's throttle must be at zero so keep motors off
            set_throttle_out(0, false);
            // deactivate accel based throttle controller
            throttle_accel_deactivate();
            set_target_alt_for_reporting(0);
        }
        break;

    case THROTTLE_AUTO:
        // auto pilot altitude controller with target altitude held in wp_nav.get_desired_alt()
        if(ap.auto_armed) {
            // special handling if we are just taking off
            if (ap.land_complete) {
                // tell motors to do a slow start.
                motors.slow_start(true);
            }
            get_throttle_althold_with_slew(wp_nav.get_desired_alt(), -wp_nav.get_descent_velocity(), wp_nav.get_climb_velocity());
            set_target_alt_for_reporting(wp_nav.get_desired_alt()); // To-Do: return get_destination_alt if we are flying to a waypoint
        }else{
            // pilot's throttle must be at zero so keep motors off
            set_throttle_out(0, false);
            // deactivate accel based throttle controller
            throttle_accel_deactivate();
            set_target_alt_for_reporting(0);
        }
        break;

    case THROTTLE_LAND:
        // landing throttle controller
        get_throttle_land();
        set_target_alt_for_reporting(0);
        break;

#if FRAME_CONFIG == HELI_FRAME
    case THROTTLE_MANUAL_HELI:
        // trad heli manual throttle controller
        // send pilot's output directly to swash plate
        pilot_throttle_scaled = get_pilot_desired_collective(g.rc_3.control_in);
        set_throttle_out(pilot_throttle_scaled, false);

        // update estimate of throttle cruise
        update_throttle_cruise(motors.get_collective_out());
        set_target_alt_for_reporting(0);
        break;
#endif // HELI_FRAME

    }
}

// set_target_alt_for_reporting - set target altitude in cm for reporting purposes (logs and gcs)
static void set_target_alt_for_reporting(float alt_cm)
{
    target_alt_for_reporting = alt_cm;
}

// get_target_alt_for_reporting - returns target altitude in cm for reporting purposes (logs and gcs)
static float get_target_alt_for_reporting()
{
    return target_alt_for_reporting;
}

static void read_AHRS(void)
{
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before ahrs update
    gcs_check_input();
#endif

    ahrs.update();
    omega = ins.get_gyro();
}

static void update_trig(void){
    Vector2f yawvector;
    const Matrix3f &temp   = ahrs.get_dcm_matrix();

    yawvector.x     = temp.a.x;     // sin
    yawvector.y     = temp.b.x;         // cos
    yawvector.normalize();

    cos_pitch_x     = safe_sqrt(1 - (temp.c.x * temp.c.x));     // level = 1
    cos_roll_x      = temp.c.z / cos_pitch_x;                       // level = 1

    cos_pitch_x     = constrain_float(cos_pitch_x, 0, 1.0);
    // this relies on constrain_float() of infinity doing the right thing,
    // which it does do in avr-libc
    cos_roll_x      = constrain_float(cos_roll_x, -1.0, 1.0);

    sin_yaw         = constrain_float(yawvector.y, -1.0, 1.0);
    cos_yaw         = constrain_float(yawvector.x, -1.0, 1.0);

    // added to convert earth frame to body frame for rate controllers
    sin_pitch       = -temp.c.x;
    sin_roll        = temp.c.y / cos_pitch_x;

    // update wp_nav controller with trig values
    wp_nav.set_cos_sin_yaw(cos_yaw, sin_yaw, cos_pitch_x);

    //flat:
    // 0 ° = cos_yaw:  1.00, sin_yaw:  0.00,
    // 90° = cos_yaw:  0.00, sin_yaw:  1.00,
    // 180 = cos_yaw: -1.00, sin_yaw:  0.00,
    // 270 = cos_yaw:  0.00, sin_yaw: -1.00,
}

// read baro and sonar altitude at 20hz
static void update_altitude()
{
#if HIL_MODE == HIL_MODE_ATTITUDE
    // we are in the SIM, fake out the baro and Sonar
    baro_alt                = g_gps->altitude_cm;

    if(g.sonar_enabled) {
        sonar_alt           = baro_alt;
    }
#else
    // read in baro altitude
    baro_alt            = read_barometer();

    // read in sonar altitude
    sonar_alt           = read_sonar();
#endif  // HIL_MODE == HIL_MODE_ATTITUDE

    // write altitude info to dataflash logs
    if (g.log_bitmask & MASK_LOG_CTUN) {
        Log_Write_Control_Tuning();
    }
}

static void tuning(){

    // exit immediately when radio failsafe is invoked so tuning values are not set to zero
    if (failsafe.radio || failsafe.radio_counter != 0) {
        return;
    }

    tuning_value = (float)g.rc_6.control_in / 1000.0f;
    g.rc_6.set_range(g.radio_tuning_low,g.radio_tuning_high);                   // 0 to 1

    switch(g.radio_tuning) {

    // Roll, Pitch tuning
    case CH6_STABILIZE_ROLL_PITCH_KP:
        g.pi_stabilize_roll.kP(tuning_value);
        g.pi_stabilize_pitch.kP(tuning_value);
        break;

    case CH6_RATE_ROLL_PITCH_KP:
        g.pid_rate_roll.kP(tuning_value);
        g.pid_rate_pitch.kP(tuning_value);
        break;

    case CH6_RATE_ROLL_PITCH_KI:
        g.pid_rate_roll.kI(tuning_value);
        g.pid_rate_pitch.kI(tuning_value);
        break;

    case CH6_RATE_ROLL_PITCH_KD:
        g.pid_rate_roll.kD(tuning_value);
        g.pid_rate_pitch.kD(tuning_value);
        break;

    // Yaw tuning
    case CH6_STABILIZE_YAW_KP:
        g.pi_stabilize_yaw.kP(tuning_value);
        break;

    case CH6_YAW_RATE_KP:
        g.pid_rate_yaw.kP(tuning_value);
        break;

    case CH6_YAW_RATE_KD:
        g.pid_rate_yaw.kD(tuning_value);
        break;

    // Altitude and throttle tuning
    case CH6_ALTITUDE_HOLD_KP:
        g.pi_alt_hold.kP(tuning_value);
        break;

    case CH6_THROTTLE_RATE_KP:
        g.pid_throttle_rate.kP(tuning_value);
        break;

    case CH6_THROTTLE_RATE_KD:
        g.pid_throttle_rate.kD(tuning_value);
        break;

    case CH6_THROTTLE_ACCEL_KP:
        g.pid_throttle_accel.kP(tuning_value);
        break;

    case CH6_THROTTLE_ACCEL_KI:
        g.pid_throttle_accel.kI(tuning_value);
        break;

    case CH6_THROTTLE_ACCEL_KD:
        g.pid_throttle_accel.kD(tuning_value);
        break;

    // Loiter and navigation tuning
    case CH6_LOITER_POSITION_KP:
        g.pi_loiter_lat.kP(tuning_value);
        g.pi_loiter_lon.kP(tuning_value);
        break;

    case CH6_LOITER_RATE_KP:
        g.pid_loiter_rate_lon.kP(tuning_value);
        g.pid_loiter_rate_lat.kP(tuning_value);
        break;

    case CH6_LOITER_RATE_KI:
        g.pid_loiter_rate_lon.kI(tuning_value);
        g.pid_loiter_rate_lat.kI(tuning_value);
        break;

    case CH6_LOITER_RATE_KD:
        g.pid_loiter_rate_lon.kD(tuning_value);
        g.pid_loiter_rate_lat.kD(tuning_value);
        break;

    case CH6_WP_SPEED:
        // set waypoint navigation horizontal speed to 0 ~ 1000 cm/s
        wp_nav.set_horizontal_velocity(g.rc_6.control_in);
        break;

    // Acro roll pitch gain
    case CH6_ACRO_RP_KP:
        g.acro_rp_p = tuning_value;
        break;

    // Acro yaw gain
    case CH6_ACRO_YAW_KP:
        g.acro_yaw_p = tuning_value;
        break;

    case CH6_RELAY:
        if (g.rc_6.control_in > 525) relay.on(0);
        if (g.rc_6.control_in < 475) relay.off(0);
        break;

#if FRAME_CONFIG == HELI_FRAME
    case CH6_HELI_EXTERNAL_GYRO:
        motors.ext_gyro_gain(g.rc_6.control_in);
        break;
#endif

    case CH6_OPTFLOW_KP:
        g.pid_optflow_roll.kP(tuning_value);
        g.pid_optflow_pitch.kP(tuning_value);
        break;

    case CH6_OPTFLOW_KI:
        g.pid_optflow_roll.kI(tuning_value);
        g.pid_optflow_pitch.kI(tuning_value);
        break;

    case CH6_OPTFLOW_KD:
        g.pid_optflow_roll.kD(tuning_value);
        g.pid_optflow_pitch.kD(tuning_value);
        break;

#if HIL_MODE != HIL_MODE_ATTITUDE                                       // do not allow modifying _kp or _kp_yaw gains in HIL mode
    case CH6_AHRS_YAW_KP:
        ahrs._kp_yaw.set(tuning_value);
        break;

    case CH6_AHRS_KP:
        ahrs._kp.set(tuning_value);
        break;
#endif

    case CH6_INAV_TC:
        // To-Do: allowing tuning TC for xy and z separately
        inertial_nav.set_time_constant_xy(tuning_value);
        inertial_nav.set_time_constant_z(tuning_value);
        break;

    case CH6_DECLINATION:
        // set declination to +-20degrees
        compass.set_declination(ToRad((2.0f * g.rc_6.control_in - g.radio_tuning_high)/100.0f), false);     // 2nd parameter is false because we do not want to save to eeprom because this would have a performance impact
        break;

    case CH6_CIRCLE_RATE:
        // set circle rate
        g.circle_rate.set(g.rc_6.control_in/25-20);     // allow approximately 45 degree turn rate in either direction
        break;

    case CH6_SONAR_GAIN:
        // set sonar gain
        g.sonar_gain.set(tuning_value);
        break;

    case CH6_LOIT_SPEED:
        // set max loiter speed to 0 ~ 1000 cm/s
        wp_nav.set_loiter_velocity(g.rc_6.control_in);
        break;
    }
}

// update AHRS system
static void ahrs_update()
{
#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before AHRS update
    gcs_check_input();
#endif

    ahrs.update();

#if LOGGING_ENABLED == ENABLED
    if (should_log(MASK_LOG_ATTITUDE_FAST))
        plane_Log_Write_Attitude();

    if (should_log(MASK_LOG_IMU))
        Log_Write_IMU();
#endif
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
#if LOGGING_ENABLED
        if (should_log(MASK_LOG_COMPASS)) {
            plane_Log_Write_Compass();
        }
#endif
    } else {
        ahrs.set_compass(NULL);
    }
}

#if LOGGING_ENABLED == ENABLED
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
#endif

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
#if LOGGING_ENABLED
    if (should_log(MASK_LOG_CURRENT))
        Log_Write_Current();
#endif
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
#if LOGGING_ENABLED == ENABLED
    if (should_log(MASK_LOG_PM))
        plane_Log_Write_Performance();
#endif
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
#if LOGGING_ENABLED == ENABLED
        if (should_log(MASK_LOG_GPS)) {
            Log_Write_GPS();
        }
#endif
    }
}

/*
  read update GPS position - 10Hz update
 */
static void update_GPS_10Hz(void)
{
    // get position from AHRS
    have_position = ahrs.get_projected_position(current_loc);

//    if (g_gps->new_data && g_gps->status() >= GPS::GPS_OK_FIX_3D) {
//        g_gps->new_data = false;

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
                ground_start_count = 10;

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
 
 
 //           update_home(); WORKAROUND FOR PIXHAWK BUILD FAIL
		home.lng        = g_gps->longitude;                                 // Lon * 10**7
		home.lat        = g_gps->latitude;                                  // Lat * 10**7
		home.alt        = max(g_gps->altitude_cm, 0);
		barometer.update_calibration();
	// END WORKAROUND
	
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
#if LOGGING_ENABLED
        if (should_log(MASK_LOG_TECS)) {
            Log_Write_TECS_Tuning();
        }
#endif
    }

    // tell AHRS the airspeed to true airspeed ratio
    airspeed.set_EAS2TAS(barometer.get_EAS2TAS());
}

AP_HAL_MAIN();
#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/AP_State.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


void set_home_is_set(bool b)
{
    // if no change, exit immediately
    if( ap.home_is_set == b )
        return;

    ap.home_is_set 	= b;
    if(b) {
        Log_Write_Event(DATA_SET_HOME);
    }
}

// ---------------------------------------------
void set_auto_armed(bool b)
{
    // if no change, exit immediately
    if( ap.auto_armed == b )
        return;

    ap.auto_armed = b;
    if(b){
        Log_Write_Event(DATA_AUTO_ARMED);
    }
}

// ---------------------------------------------
void set_simple_mode(uint8_t b)
{
    if(ap.simple_mode != b){
        if(b == 0){
            Log_Write_Event(DATA_SET_SIMPLE_OFF);
        }else if(b == 1){
            Log_Write_Event(DATA_SET_SIMPLE_ON);
        }else{
            // initialise super simple heading
            update_super_simple_bearing(true);
            Log_Write_Event(DATA_SET_SUPERSIMPLE_ON);
        }
        ap.simple_mode = b;
    }
}

// ---------------------------------------------
static void set_failsafe_radio(bool b)
{
    // only act on changes
    // -------------------
    if(failsafe.radio != b) {

        // store the value so we don't trip the gate twice
        // -----------------------------------------------
        failsafe.radio = b;

        if (failsafe.radio == false) {
            // We've regained radio contact
            // ----------------------------
            failsafe_radio_off_event();
        }else{
            // We've lost radio contact
            // ------------------------
            failsafe_radio_on_event();
        }

        // update AP_Notify
        AP_Notify::flags.failsafe_radio = b;
    }
}


// ---------------------------------------------
void set_failsafe_battery(bool b)
{
    failsafe.battery = b;
    AP_Notify::flags.failsafe_battery = b;
}


// ---------------------------------------------
static void set_failsafe_gps(bool b)
{
    failsafe.gps = b;

    // update AP_Notify
    AP_Notify::flags.failsafe_gps = b;
}

// ---------------------------------------------
static void set_failsafe_gcs(bool b)
{
    failsafe.gcs = b;
}

// ---------------------------------------------
void set_takeoff_complete(bool b)
{
    // if no change, exit immediately
    if( ap.takeoff_complete == b )
        return;

    if(b){
        Log_Write_Event(DATA_TAKEOFF);
    }
    ap.takeoff_complete = b;
}

// ---------------------------------------------
void set_land_complete(bool b)
{
    // if no change, exit immediately
    if( ap.land_complete == b )
        return;

    if(b){
        Log_Write_Event(DATA_LAND_COMPLETE);
    }else{
        Log_Write_Event(DATA_NOT_LANDED);
    }
    ap.land_complete = b;
}

// ---------------------------------------------

void set_pre_arm_check(bool b)
{
    if(ap.pre_arm_check != b) {
        ap.pre_arm_check = b;
        AP_Notify::flags.pre_arm_check = b;
    }
}

void set_pre_arm_rc_check(bool b)
{
    if(ap.pre_arm_rc_check != b) {
        ap.pre_arm_rc_check = b;
    }
}

#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/Attitude.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// local variables
float roll_in_filtered;     // roll-in in filtered with RC_FEEL_RP parameter
float pitch_in_filtered;    // pitch-in filtered with RC_FEEL_RP parameter

static void reset_roll_pitch_in_filters(int16_t roll_in, int16_t pitch_in)
{
    roll_in_filtered = constrain_int16(roll_in, -ROLL_PITCH_INPUT_MAX, ROLL_PITCH_INPUT_MAX);
    pitch_in_filtered = constrain_int16(pitch_in, -ROLL_PITCH_INPUT_MAX, ROLL_PITCH_INPUT_MAX);
}

// get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in centi-degrees
static void get_pilot_desired_lean_angles(int16_t roll_in, int16_t pitch_in, int16_t &roll_out, int16_t &pitch_out)
{
    static float _scaler = 1.0;
    static int16_t _angle_max = 0;

    // range check the input
    roll_in = constrain_int16(roll_in, -ROLL_PITCH_INPUT_MAX, ROLL_PITCH_INPUT_MAX);
    pitch_in = constrain_int16(pitch_in, -ROLL_PITCH_INPUT_MAX, ROLL_PITCH_INPUT_MAX);

    // filter input for feel
    if (g.rc_feel_rp >= RC_FEEL_RP_VERY_CRISP) {
        // no filtering required
        roll_in_filtered = roll_in;
        pitch_in_filtered = pitch_in;
    }else{
        float filter_gain;
        if (g.rc_feel_rp >= RC_FEEL_RP_CRISP) {
            filter_gain = 0.5;
        } else if(g.rc_feel_rp >= RC_FEEL_RP_MEDIUM) {
            filter_gain = 0.3;
        } else if(g.rc_feel_rp >= RC_FEEL_RP_SOFT) {
            filter_gain = 0.05;
        } else {
            // must be RC_FEEL_RP_VERY_SOFT
            filter_gain = 0.02;
        }
        roll_in_filtered = roll_in_filtered * (1.0 - filter_gain) + (float)roll_in * filter_gain;
        pitch_in_filtered = pitch_in_filtered * (1.0 - filter_gain) + (float)pitch_in * filter_gain;
    }

    // return filtered roll if no scaling required
    if (g.angle_max == ROLL_PITCH_INPUT_MAX) {
        roll_out = (int16_t)roll_in_filtered;
        pitch_out = (int16_t)pitch_in_filtered;
        return;
    }

    // check if angle_max has been updated and redo scaler
    if (g.angle_max != _angle_max) {
        _angle_max = g.angle_max;
        _scaler = (float)g.angle_max/(float)ROLL_PITCH_INPUT_MAX;
    }

    // convert pilot input to lean angle
    roll_out = (int16_t)(roll_in_filtered * _scaler);
    pitch_out = (int16_t)(pitch_in_filtered * _scaler);
}

static void
get_stabilize_roll(int32_t target_angle)
{
    // angle error
    target_angle = wrap_180_cd(target_angle - ahrs.roll_sensor);

    // convert to desired rate
    int32_t target_rate = g.pi_stabilize_roll.kP() * target_angle;

    // constrain the target rate
    if (!ap.disable_stab_rate_limit) {
        target_rate = constrain_int32(target_rate, -g.angle_rate_max, g.angle_rate_max);
    }

    // set targets for rate controller
    set_roll_rate_target(target_rate, EARTH_FRAME);
}

static void
get_stabilize_pitch(int32_t target_angle)
{
    // angle error
    target_angle            = wrap_180_cd(target_angle - ahrs.pitch_sensor);

    // convert to desired rate
    int32_t target_rate = g.pi_stabilize_pitch.kP() * target_angle;

    // constrain the target rate
    if (!ap.disable_stab_rate_limit) {
        target_rate = constrain_int32(target_rate, -g.angle_rate_max, g.angle_rate_max);
    }

    // set targets for rate controller
    set_pitch_rate_target(target_rate, EARTH_FRAME);
}

static void
get_stabilize_yaw(int32_t target_angle)
{
    int32_t target_rate;
    int32_t angle_error;

    // angle error
    angle_error = wrap_180_cd(target_angle - ahrs.yaw_sensor);

    // limit the error we're feeding to the PID
    angle_error = constrain_int32(angle_error, -4500, 4500);

    // convert angle error to desired Rate:
    target_rate = g.pi_stabilize_yaw.kP() * angle_error;

    // do not use rate controllers for helicotpers with external gyros
#if FRAME_CONFIG == HELI_FRAME
    if(motors.tail_type() == AP_MOTORS_HELI_TAILTYPE_SERVO_EXTGYRO) {
        g.rc_4.servo_out = constrain_int32(target_rate, -4500, 4500);
    }
#endif

    // set targets for rate controller
    set_yaw_rate_target(target_rate, EARTH_FRAME);
}

// get_acro_level_rates - calculate earth frame rate corrections to pull the copter back to level while in ACRO mode
static void
get_acro_level_rates()
{
    // zero earth frame leveling if trainer is disabled
    if (g.acro_trainer == ACRO_TRAINER_DISABLED) {
        set_roll_rate_target(0, BODY_EARTH_FRAME);
        set_pitch_rate_target(0, BODY_EARTH_FRAME);
        set_yaw_rate_target(0, BODY_EARTH_FRAME);
        return;
    }

    // Calculate trainer mode earth frame rate command for roll
    int32_t roll_angle = wrap_180_cd(ahrs.roll_sensor);
    int32_t target_rate = 0;

    if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
        if (roll_angle > g.angle_max){
            target_rate =  g.pi_stabilize_roll.get_p(g.angle_max-roll_angle);
        }else if (roll_angle < -g.angle_max) {
            target_rate =  g.pi_stabilize_roll.get_p(-g.angle_max-roll_angle);
        }
    }
    roll_angle   = constrain_int32(roll_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE);
    target_rate -= roll_angle * g.acro_balance_roll;

    // add earth frame targets for roll rate controller
    set_roll_rate_target(target_rate, BODY_EARTH_FRAME);

    // Calculate trainer mode earth frame rate command for pitch
    int32_t pitch_angle = wrap_180_cd(ahrs.pitch_sensor);
    target_rate = 0;

    if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
        if (pitch_angle > g.angle_max){
            target_rate =  g.pi_stabilize_pitch.get_p(g.angle_max-pitch_angle);
        }else if (pitch_angle < -g.angle_max) {
            target_rate =  g.pi_stabilize_pitch.get_p(-g.angle_max-pitch_angle);
        }
    }
    pitch_angle  = constrain_int32(pitch_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE);
    target_rate -= pitch_angle * g.acro_balance_pitch;

    // add earth frame targets for pitch rate controller
    set_pitch_rate_target(target_rate, BODY_EARTH_FRAME);

    // add earth frame targets for yaw rate controller
    set_yaw_rate_target(0, BODY_EARTH_FRAME);
}

// Roll with rate input and stabilized in the body frame
static void
get_roll_rate_stabilized_bf(int32_t stick_angle)
{
    static float angle_error = 0;

    // convert the input to the desired body frame roll rate
    int32_t rate_request = stick_angle * g.acro_rp_p;

    if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
        rate_request += acro_roll_rate;
    }else{        
        // Scale pitch leveling by stick input
        acro_roll_rate = (float)acro_roll_rate*acro_level_mix;

        // Calculate rate limit to prevent change of rate through inverted
        int32_t rate_limit = labs(labs(rate_request)-labs(acro_roll_rate));

        rate_request += acro_roll_rate;
        rate_request = constrain_int32(rate_request, -rate_limit, rate_limit);
    }

    // add automatic correction
    int32_t rate_correction = g.pi_stabilize_roll.get_p(angle_error);

    // set body frame targets for rate controller
    set_roll_rate_target(rate_request+rate_correction, BODY_FRAME);

    // Calculate integrated body frame rate error
    angle_error += (rate_request - (omega.x * DEGX100)) * G_Dt;

    // don't let angle error grow too large
    angle_error = constrain_float(angle_error, -MAX_ROLL_OVERSHOOT, MAX_ROLL_OVERSHOOT);

#if FRAME_CONFIG == HELI_FRAME
    if (!motors.motor_runup_complete()) {
           angle_error = 0;
    }
#else
    if (!motors.armed() || g.rc_3.servo_out == 0) {
        angle_error = 0;
    }
#endif // HELI_FRAME
}

// Pitch with rate input and stabilized in the body frame
static void
get_pitch_rate_stabilized_bf(int32_t stick_angle)
{
    static float angle_error = 0;

    // convert the input to the desired body frame pitch rate
    int32_t rate_request = stick_angle * g.acro_rp_p;

    if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
        rate_request += acro_pitch_rate;
    }else{
        // Scale pitch leveling by stick input
        acro_pitch_rate = (float)acro_pitch_rate*acro_level_mix;
        
        // Calculate rate limit to prevent change of rate through inverted
        int32_t rate_limit = labs(labs(rate_request)-labs(acro_pitch_rate));
        
        rate_request += acro_pitch_rate;
        rate_request = constrain_int32(rate_request, -rate_limit, rate_limit);
    }

    // add automatic correction
    int32_t rate_correction = g.pi_stabilize_pitch.get_p(angle_error);

    // set body frame targets for rate controller
    set_pitch_rate_target(rate_request+rate_correction, BODY_FRAME);

    // Calculate integrated body frame rate error
    angle_error += (rate_request - (omega.y * DEGX100)) * G_Dt;

    // don't let angle error grow too large
    angle_error = constrain_float(angle_error, -MAX_PITCH_OVERSHOOT, MAX_PITCH_OVERSHOOT);

#if FRAME_CONFIG == HELI_FRAME
    if (!motors.motor_runup_complete()) {
           angle_error = 0;
    }
#else
    if (!motors.armed() || g.rc_3.servo_out == 0) {
        angle_error = 0;
    }
#endif // HELI_FRAME
}

// Yaw with rate input and stabilized in the body frame
static void
get_yaw_rate_stabilized_bf(int32_t stick_angle)
{
    static float angle_error = 0;

    // convert the input to the desired body frame yaw rate
    int32_t rate_request = stick_angle * g.acro_yaw_p;

    if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
        rate_request += acro_yaw_rate;
    }else{
        // Scale pitch leveling by stick input
        acro_yaw_rate = (float)acro_yaw_rate*acro_level_mix;

        // Calculate rate limit to prevent change of rate through inverted
        int32_t rate_limit = labs(labs(rate_request)-labs(acro_yaw_rate));

        rate_request += acro_yaw_rate;
        rate_request = constrain_int32(rate_request, -rate_limit, rate_limit);
    }

    // add automatic correction
    int32_t rate_correction = g.pi_stabilize_yaw.get_p(angle_error);

    // set body frame targets for rate controller
    set_yaw_rate_target(rate_request+rate_correction, BODY_FRAME);

    // Calculate integrated body frame rate error
    angle_error += (rate_request - (omega.z * DEGX100)) * G_Dt;

    // don't let angle error grow too large
    angle_error = constrain_float(angle_error, -MAX_YAW_OVERSHOOT, MAX_YAW_OVERSHOOT);

#if FRAME_CONFIG == HELI_FRAME
    if (!motors.motor_runup_complete()) {
           angle_error = 0;
    }
#else
    if (!motors.armed() || g.rc_3.servo_out == 0) {
        angle_error = 0;
    }
#endif // HELI_FRAME
}

// Roll with rate input and stabilized in the earth frame
static void
get_roll_rate_stabilized_ef(int32_t stick_angle)
{
    int32_t angle_error = 0;

    // convert the input to the desired roll rate
    int32_t target_rate = stick_angle * g.acro_rp_p - (acro_roll * g.acro_balance_roll);

    // convert the input to the desired roll rate
    acro_roll += target_rate * G_Dt;
    acro_roll = wrap_180_cd(acro_roll);

    // ensure that we don't reach gimbal lock
    if (labs(acro_roll) > g.angle_max) {
        acro_roll  = constrain_int32(acro_roll, -g.angle_max, g.angle_max);
        angle_error = wrap_180_cd(acro_roll - ahrs.roll_sensor);
    } else {
        // angle error with maximum of +- max_angle_overshoot
        angle_error = wrap_180_cd(acro_roll - ahrs.roll_sensor);
        angle_error  = constrain_int32(angle_error, -MAX_ROLL_OVERSHOOT, MAX_ROLL_OVERSHOOT);
    }

#if FRAME_CONFIG == HELI_FRAME
    if (!motors.motor_runup_complete()) {
        angle_error = 0;
    }
#else      
    // reset target angle to current angle if motors not spinning
    if (!motors.armed() || g.rc_3.servo_out == 0) {
        angle_error = 0;
    }
#endif // HELI_FRAME

    // update acro_roll to be within max_angle_overshoot of our current heading
    acro_roll = wrap_180_cd(angle_error + ahrs.roll_sensor);

    // set earth frame targets for rate controller
  set_roll_rate_target(g.pi_stabilize_roll.get_p(angle_error) + target_rate, EARTH_FRAME);
}

// Pitch with rate input and stabilized in the earth frame
static void
get_pitch_rate_stabilized_ef(int32_t stick_angle)
{
    int32_t angle_error = 0;

    // convert the input to the desired pitch rate
    int32_t target_rate = stick_angle * g.acro_rp_p - (acro_pitch * g.acro_balance_pitch);

    // convert the input to the desired pitch rate
    acro_pitch += target_rate * G_Dt;
    acro_pitch = wrap_180_cd(acro_pitch);

    // ensure that we don't reach gimbal lock
    if (labs(acro_pitch) > g.angle_max) {
        acro_pitch  = constrain_int32(acro_pitch, -g.angle_max, g.angle_max);
        angle_error = wrap_180_cd(acro_pitch - ahrs.pitch_sensor);
    } else {
        // angle error with maximum of +- max_angle_overshoot
        angle_error = wrap_180_cd(acro_pitch - ahrs.pitch_sensor);
        angle_error  = constrain_int32(angle_error, -MAX_PITCH_OVERSHOOT, MAX_PITCH_OVERSHOOT);
    }

#if FRAME_CONFIG == HELI_FRAME
    if (!motors.motor_runup_complete()) {
        angle_error = 0;
    }
#else       
    // reset target angle to current angle if motors not spinning
    if (!motors.armed() || g.rc_3.servo_out == 0) {
        angle_error = 0;
    }
#endif // HELI_FRAME

    // update acro_pitch to be within max_angle_overshoot of our current heading
    acro_pitch = wrap_180_cd(angle_error + ahrs.pitch_sensor);

    // set earth frame targets for rate controller
    set_pitch_rate_target(g.pi_stabilize_pitch.get_p(angle_error) + target_rate, EARTH_FRAME);
}

// Yaw with rate input and stabilized in the earth frame
static void
get_yaw_rate_stabilized_ef(int32_t stick_angle)
{

    int32_t angle_error = 0;

    // convert the input to the desired yaw rate
    int32_t target_rate = stick_angle * g.acro_yaw_p;

    // convert the input to the desired yaw rate
    control_yaw += target_rate * G_Dt;
    control_yaw = wrap_360_cd(control_yaw);

    // calculate difference between desired heading and current heading
    angle_error = wrap_180_cd(control_yaw - ahrs.yaw_sensor);

    // limit the maximum overshoot
    angle_error	= constrain_int32(angle_error, -MAX_YAW_OVERSHOOT, MAX_YAW_OVERSHOOT);

#if FRAME_CONFIG == HELI_FRAME
    if (!motors.motor_runup_complete()) {
    	angle_error = 0;
    }
#else   
    // reset target angle to current heading if motors not spinning
    if (!motors.armed() || g.rc_3.servo_out == 0) {
    	angle_error = 0;
    }
#endif // HELI_FRAME

    // update control_yaw to be within max_angle_overshoot of our current heading
    control_yaw = wrap_360_cd(angle_error + ahrs.yaw_sensor);

    // set earth frame targets for rate controller
	set_yaw_rate_target(g.pi_stabilize_yaw.get_p(angle_error)+target_rate, EARTH_FRAME);
}

// set_roll_rate_target - to be called by upper controllers to set roll rate targets in the earth frame
void set_roll_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) {
    rate_targets_frame = earth_or_body_frame;
    if( earth_or_body_frame == BODY_FRAME ) {
        roll_rate_target_bf = desired_rate;
    }else{
        roll_rate_target_ef = desired_rate;
    }
}

// set_pitch_rate_target - to be called by upper controllers to set pitch rate targets in the earth frame
void set_pitch_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) {
    rate_targets_frame = earth_or_body_frame;
    if( earth_or_body_frame == BODY_FRAME ) {
        pitch_rate_target_bf = desired_rate;
    }else{
        pitch_rate_target_ef = desired_rate;
    }
}

// set_yaw_rate_target - to be called by upper controllers to set yaw rate targets in the earth frame
void set_yaw_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) {
    rate_targets_frame = earth_or_body_frame;
    if( earth_or_body_frame == BODY_FRAME ) {
        yaw_rate_target_bf = desired_rate;
    }else{
        yaw_rate_target_ef = desired_rate;
    }
}

// update_rate_contoller_targets - converts earth frame rates to body frame rates for rate controllers
void
update_rate_contoller_targets()
{
    if( rate_targets_frame == EARTH_FRAME ) {
        // convert earth frame rates to body frame rates
        roll_rate_target_bf     = roll_rate_target_ef - sin_pitch * yaw_rate_target_ef;
        pitch_rate_target_bf    = cos_roll_x  * pitch_rate_target_ef + sin_roll * cos_pitch_x * yaw_rate_target_ef;
        yaw_rate_target_bf      = cos_pitch_x * cos_roll_x * yaw_rate_target_ef - sin_roll * pitch_rate_target_ef;
    }else if( rate_targets_frame == BODY_EARTH_FRAME ) {
        // add converted earth frame rates to body frame rates
        acro_roll_rate = roll_rate_target_ef - sin_pitch * yaw_rate_target_ef;
        acro_pitch_rate = cos_roll_x  * pitch_rate_target_ef + sin_roll * cos_pitch_x * yaw_rate_target_ef;
        acro_yaw_rate = cos_pitch_x * cos_roll_x * yaw_rate_target_ef - sin_roll * pitch_rate_target_ef;
    }
}

// run roll, pitch and yaw rate controllers and send output to motors
// targets for these controllers comes from stabilize controllers
void
run_rate_controllers()
{
#if FRAME_CONFIG == HELI_FRAME
    // convert desired roll and pitch rate to roll and pitch swash angles
    heli_integrated_swash_controller(roll_rate_target_bf, pitch_rate_target_bf);
    // helicopters only use rate controllers for yaw and only when not using an external gyro
    if(motors.tail_type() != AP_MOTORS_HELI_TAILTYPE_SERVO_EXTGYRO) {
        g.rc_4.servo_out = get_heli_rate_yaw(yaw_rate_target_bf);
    }else{
        // do not use rate controllers for helicotpers with external gyros
        g.rc_4.servo_out = constrain_int32(yaw_rate_target_bf, -4500, 4500);
    }
#else
    // call rate controllers
    g.rc_1.servo_out = get_rate_roll(roll_rate_target_bf);
    g.rc_2.servo_out = get_rate_pitch(pitch_rate_target_bf);
    g.rc_4.servo_out = get_rate_yaw(yaw_rate_target_bf);
#endif

    // run throttle controller if accel based throttle controller is enabled and active (active means it has been given a target)
    if( throttle_accel_controller_active ) {
        set_throttle_out(get_throttle_accel(throttle_accel_target_ef), true);
    }
}

#if FRAME_CONFIG != HELI_FRAME
static int16_t
get_rate_roll(int32_t target_rate)
{
    int32_t p,i,d;                  // used to capture pid values for logging
    int32_t current_rate;           // this iteration's rate
    int32_t rate_error;             // simply target_rate - current_rate
    int32_t output;                 // output from pid controller

    // get current rate
    current_rate    = (omega.x * DEGX100);

    // call pid controller
    rate_error  = target_rate - current_rate;
    p           = g.pid_rate_roll.get_p(rate_error);

    // get i term
    i = g.pid_rate_roll.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!motors.limit.roll_pitch || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = g.pid_rate_roll.get_i(rate_error, G_Dt);
    }

    d = g.pid_rate_roll.get_d(rate_error, G_Dt);
    output = p + i + d;

    // constrain output
    output = constrain_int32(output, -5000, 5000);

    // output control
    return output;
}

static int16_t
get_rate_pitch(int32_t target_rate)
{
    int32_t p,i,d;                                                                      // used to capture pid values for logging
    int32_t current_rate;                                                       // this iteration's rate
    int32_t rate_error;                                                                 // simply target_rate - current_rate
    int32_t output;                                                                     // output from pid controller

    // get current rate
    current_rate    = (omega.y * DEGX100);

    // call pid controller
    rate_error      = target_rate - current_rate;
    p               = g.pid_rate_pitch.get_p(rate_error);

    // get i term
    i = g.pid_rate_pitch.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!motors.limit.roll_pitch || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = g.pid_rate_pitch.get_i(rate_error, G_Dt);
    }

    d = g.pid_rate_pitch.get_d(rate_error, G_Dt);
    output = p + i + d;

    // constrain output
    output = constrain_int32(output, -5000, 5000);

    // output control
    return output;
}

static int16_t
get_rate_yaw(int32_t target_rate)
{
    int32_t p,i,d;                                                                      // used to capture pid values for logging
    int32_t rate_error;
    int32_t output;

    // rate control
    rate_error              = target_rate - (omega.z * DEGX100);

    // separately calculate p, i, d values for logging
    p = g.pid_rate_yaw.get_p(rate_error);

    // get i term
    i = g.pid_rate_yaw.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!motors.limit.yaw || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = g.pid_rate_yaw.get_i(rate_error, G_Dt);
    }

    // get d value
    d = g.pid_rate_yaw.get_d(rate_error, G_Dt);

    output  = p+i+d;
    output = constrain_int32(output, -4500, 4500);

    // constrain output
    return output;
}
#endif // !HELI_FRAME

// calculate modified roll/pitch depending upon optical flow calculated position
static int32_t
get_of_roll(int32_t input_roll)
{
#if OPTFLOW == ENABLED
    static float tot_x_cm = 0;      // total distance from target
    static uint32_t last_of_roll_update = 0;
    int32_t new_roll = 0;
    int32_t p,i,d;

    // check if new optflow data available
    if( optflow.last_update != last_of_roll_update) {
        last_of_roll_update = optflow.last_update;

        // add new distance moved
        tot_x_cm += optflow.x_cm;

        // only stop roll if caller isn't modifying roll
        if( input_roll == 0 && current_loc.alt < 1500) {
            p = g.pid_optflow_roll.get_p(-tot_x_cm);
            i = g.pid_optflow_roll.get_i(-tot_x_cm,1.0f);              // we could use the last update time to calculate the time change
            d = g.pid_optflow_roll.get_d(-tot_x_cm,1.0f);
            new_roll = p+i+d;
        }else{
            g.pid_optflow_roll.reset_I();
            tot_x_cm = 0;
            p = 0;              // for logging
            i = 0;
            d = 0;
        }
        // limit amount of change and maximum angle
        of_roll = constrain_int32(new_roll, (of_roll-20), (of_roll+20));
    }

    // limit max angle
    of_roll = constrain_int32(of_roll, -1000, 1000);

    return input_roll+of_roll;
#else
    return input_roll;
#endif
}

static int32_t
get_of_pitch(int32_t input_pitch)
{
#if OPTFLOW == ENABLED
    static float tot_y_cm = 0;  // total distance from target
    static uint32_t last_of_pitch_update = 0;
    int32_t new_pitch = 0;
    int32_t p,i,d;

    // check if new optflow data available
    if( optflow.last_update != last_of_pitch_update ) {
        last_of_pitch_update = optflow.last_update;

        // add new distance moved
        tot_y_cm += optflow.y_cm;

        // only stop roll if caller isn't modifying pitch
        if( input_pitch == 0 && current_loc.alt < 1500 ) {
            p = g.pid_optflow_pitch.get_p(tot_y_cm);
            i = g.pid_optflow_pitch.get_i(tot_y_cm, 1.0f);              // we could use the last update time to calculate the time change
            d = g.pid_optflow_pitch.get_d(tot_y_cm, 1.0f);
            new_pitch = p + i + d;
        }else{
            tot_y_cm = 0;
            g.pid_optflow_pitch.reset_I();
            p = 0;              // for logging
            i = 0;
            d = 0;
        }

        // limit amount of change
        of_pitch = constrain_int32(new_pitch, (of_pitch-20), (of_pitch+20));
    }

    // limit max angle
    of_pitch = constrain_int32(of_pitch, -1000, 1000);

    return input_pitch+of_pitch;
#else
    return input_pitch;
#endif
}

/*************************************************************
 * yaw controllers
 *************************************************************/

 // get_look_at_yaw - updates bearing to look at center of circle or do a panorama
// should be called at 100hz
static void get_circle_yaw()
{
    static uint8_t look_at_yaw_counter = 0;     // used to reduce update rate to 10hz

    // if circle radius is zero do panorama
    if( g.circle_radius == 0 ) {
        // slew yaw towards circle angle
        control_yaw = get_yaw_slew(control_yaw, ToDeg(circle_angle)*100, AUTO_YAW_SLEW_RATE);
    }else{
        look_at_yaw_counter++;
        if( look_at_yaw_counter >= 10 ) {
            look_at_yaw_counter = 0;
            yaw_look_at_WP_bearing = pv_get_bearing_cd(inertial_nav.get_position(), yaw_look_at_WP);
        }
        // slew yaw
        control_yaw = get_yaw_slew(control_yaw, yaw_look_at_WP_bearing, AUTO_YAW_SLEW_RATE);
    }

    // call stabilize yaw controller
    get_stabilize_yaw(control_yaw);
}

// get_look_at_yaw - updates bearing to location held in look_at_yaw_WP and calls stabilize yaw controller
// should be called at 100hz
static void get_look_at_yaw()
{
    static uint8_t look_at_yaw_counter = 0;     // used to reduce update rate to 10hz

    look_at_yaw_counter++;
    if( look_at_yaw_counter >= 10 ) {
        look_at_yaw_counter = 0;
        yaw_look_at_WP_bearing = pv_get_bearing_cd(inertial_nav.get_position(), yaw_look_at_WP);
    }

    // slew yaw and call stabilize controller
    control_yaw = get_yaw_slew(control_yaw, yaw_look_at_WP_bearing, AUTO_YAW_SLEW_RATE);
    get_stabilize_yaw(control_yaw);
}

static void get_look_ahead_yaw(int16_t pilot_yaw)
{
    // Commanded Yaw to automatically look ahead.
    if (g_gps->fix && g_gps->ground_speed_cm > YAW_LOOK_AHEAD_MIN_SPEED) {
        control_yaw = get_yaw_slew(control_yaw, g_gps->ground_course_cd, AUTO_YAW_SLEW_RATE);
        get_stabilize_yaw(wrap_360_cd(control_yaw + pilot_yaw));   // Allow pilot to "skid" around corners up to 45 degrees
    }else{
        control_yaw += pilot_yaw * g.acro_yaw_p * G_Dt;
        control_yaw = wrap_360_cd(control_yaw);
        get_stabilize_yaw(control_yaw);
    }
}

/*************************************************************
 *  throttle control
 ****************************************************************/

// update_throttle_cruise - update throttle cruise if necessary
static void update_throttle_cruise(int16_t throttle)
{
    // ensure throttle_avg has been initialised
    if( throttle_avg == 0 ) {
        throttle_avg = g.throttle_cruise;
    }
    // calc average throttle if we are in a level hover
    if (throttle > g.throttle_min && abs(climb_rate) < 60 && labs(ahrs.roll_sensor) < 500 && labs(ahrs.pitch_sensor) < 500) {
        throttle_avg = throttle_avg * 0.99f + (float)throttle * 0.01f;
        g.throttle_cruise = throttle_avg;
    }
}

#if FRAME_CONFIG == HELI_FRAME
// get_angle_boost - returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1000
// for traditional helicopters
static int16_t get_angle_boost(int16_t throttle)
{
    float angle_boost_factor = cos_pitch_x * cos_roll_x;
    angle_boost_factor = 1.0f - constrain_float(angle_boost_factor, .5f, 1.0f);
    int16_t throttle_above_mid = max(throttle - motors.get_collective_mid(),0);

    // to allow logging of angle boost
    angle_boost = throttle_above_mid*angle_boost_factor;

    return throttle + angle_boost;
}
#else   // all multicopters
// get_angle_boost - returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1000
static int16_t get_angle_boost(int16_t throttle)
{
    float temp = cos_pitch_x * cos_roll_x;
    int16_t throttle_out;

    temp = constrain_float(temp, 0.5f, 1.0f);

    // reduce throttle if we go inverted
    temp = constrain_float(9000-max(labs(ahrs.roll_sensor),labs(ahrs.pitch_sensor)), 0, 3000) / (3000 * temp);

    // apply scale and constrain throttle
    throttle_out = constrain_float((float)(throttle-g.throttle_min) * temp + g.throttle_min, g.throttle_min, 1000);

    // to allow logging of angle boost
    angle_boost = throttle_out - throttle;

    return throttle_out;
}
#endif // FRAME_CONFIG == HELI_FRAME

 // set_throttle_out - to be called by upper throttle controllers when they wish to provide throttle output directly to motors
 // provide 0 to cut motors
void set_throttle_out( int16_t throttle_out, bool apply_angle_boost )
{
    if( apply_angle_boost ) {
        g.rc_3.servo_out = get_angle_boost(throttle_out);
    }else{
        g.rc_3.servo_out = throttle_out;
        // clear angle_boost for logging purposes
        angle_boost = 0;
    }

    // update compass with throttle value
    compass.set_throttle((float)g.rc_3.servo_out/1000.0f);
}

// set_throttle_accel_target - to be called by upper throttle controllers to set desired vertical acceleration in earth frame
void set_throttle_accel_target( int16_t desired_acceleration )
{
    throttle_accel_target_ef = desired_acceleration;
    throttle_accel_controller_active = true;
}

// disable_throttle_accel - disables the accel based throttle controller
// it will be re-enasbled on the next set_throttle_accel_target
// required when we wish to set motors to zero when pilot inputs zero throttle
void throttle_accel_deactivate()
{
    throttle_accel_controller_active = false;
}

// set_throttle_takeoff - allows parents to tell throttle controller we are taking off so I terms can be cleared
static void
set_throttle_takeoff()
{
    // set alt target
    controller_desired_alt = current_loc.alt + ALT_HOLD_TAKEOFF_JUMP;

    // clear i term from acceleration controller
    if (g.pid_throttle_accel.get_integrator() < 0) {
        g.pid_throttle_accel.reset_I();
    }
    // tell motors to do a slow start
    motors.slow_start(true);
}

// get_throttle_accel - accelerometer based throttle controller
// returns an actual throttle output (0 ~ 1000) to be sent to the motors
static int16_t
get_throttle_accel(int16_t z_target_accel)
{
    static float z_accel_error = 0;     // The acceleration error in cm.
    static uint32_t last_call_ms = 0;   // the last time this controller was called
    int32_t p,i,d;                      // used to capture pid values for logging
    int16_t output;
    float z_accel_meas;
    uint32_t now = millis();

    // Calculate Earth Frame Z acceleration
    z_accel_meas = -(ahrs.get_accel_ef().z + GRAVITY_MSS) * 100;

    // reset target altitude if this controller has just been engaged
    if( now - last_call_ms > 100 ) {
        // Reset Filter
        z_accel_error = 0;
    } else {
        // calculate accel error and Filter with fc = 2 Hz
        z_accel_error = z_accel_error + 0.11164f * (constrain_float(z_target_accel - z_accel_meas, -32000, 32000) - z_accel_error);
    }
    last_call_ms = now;

    // separately calculate p, i, d values for logging
    p = g.pid_throttle_accel.get_p(z_accel_error);

    // get i term
    i = g.pid_throttle_accel.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if ((!motors.limit.throttle_lower && !motors.limit.throttle_upper) || (i>0&&z_accel_error<0) || (i<0&&z_accel_error>0)) {
        i = g.pid_throttle_accel.get_i(z_accel_error, .01f);
    }

    d = g.pid_throttle_accel.get_d(z_accel_error, .01f);

    output =  constrain_float(p+i+d+g.throttle_cruise, g.throttle_min, g.throttle_max);

    return output;
}

// get_pilot_desired_throttle - transform pilot's throttle input to make cruise throttle mid stick
// used only for manual throttle modes
// returns throttle output 0 to 1000
#define THROTTLE_IN_MIDDLE 500          // the throttle mid point
static int16_t get_pilot_desired_throttle(int16_t throttle_control)
{
    int16_t throttle_out;

    // exit immediately in the simple cases
    if( throttle_control == 0 || g.throttle_mid == 500) {
        return throttle_control;
    }

    // ensure reasonable throttle values
    throttle_control = constrain_int16(throttle_control,0,1000);
    g.throttle_mid = constrain_int16(g.throttle_mid,300,700);

    // check throttle is above, below or in the deadband
    if (throttle_control < THROTTLE_IN_MIDDLE) {
        // below the deadband
        throttle_out = g.throttle_min + ((float)(throttle_control-g.throttle_min))*((float)(g.throttle_mid - g.throttle_min))/((float)(500-g.throttle_min));
    }else if(throttle_control > THROTTLE_IN_MIDDLE) {
        // above the deadband
        throttle_out = g.throttle_mid + ((float)(throttle_control-500))*(float)(1000-g.throttle_mid)/500.0f;
    }else{
        // must be in the deadband
        throttle_out = g.throttle_mid;
    }

    return throttle_out;
}

// get_pilot_desired_climb_rate - transform pilot's throttle input to
// climb rate in cm/s.  we use radio_in instead of control_in to get the full range
// without any deadzone at the bottom
#define THROTTLE_IN_DEADBAND_TOP (THROTTLE_IN_MIDDLE+THROTTLE_IN_DEADBAND)  // top of the deadband
#define THROTTLE_IN_DEADBAND_BOTTOM (THROTTLE_IN_MIDDLE-THROTTLE_IN_DEADBAND)  // bottom of the deadband
static int16_t get_pilot_desired_climb_rate(int16_t throttle_control)
{
    int16_t desired_rate = 0;

    // throttle failsafe check
    if( failsafe.radio ) {
        return 0;
    }

    // ensure a reasonable throttle value
    throttle_control = constrain_int16(throttle_control,0,1000);

    // check throttle is above, below or in the deadband
    if (throttle_control < THROTTLE_IN_DEADBAND_BOTTOM) {
        // below the deadband
        desired_rate = (int32_t)g.pilot_velocity_z_max * (throttle_control-THROTTLE_IN_DEADBAND_BOTTOM) / (THROTTLE_IN_MIDDLE - THROTTLE_IN_DEADBAND);
    }else if (throttle_control > THROTTLE_IN_DEADBAND_TOP) {
        // above the deadband
        desired_rate = (int32_t)g.pilot_velocity_z_max * (throttle_control-THROTTLE_IN_DEADBAND_TOP) / (THROTTLE_IN_MIDDLE - THROTTLE_IN_DEADBAND);
    }else{
        // must be in the deadband
        desired_rate = 0;
    }

    // desired climb rate for logging
    desired_climb_rate = desired_rate;

    return desired_rate;
}

// get_initial_alt_hold - get new target altitude based on current altitude and climb rate
static int32_t
get_initial_alt_hold( int32_t alt_cm, int16_t climb_rate_cms)
{
    int32_t target_alt;
    int32_t linear_distance;      // half the distace we swap between linear and sqrt and the distace we offset sqrt.
    int32_t linear_velocity;      // the velocity we swap between linear and sqrt.

    linear_velocity = ALT_HOLD_ACCEL_MAX/g.pi_alt_hold.kP();

    if (abs(climb_rate_cms) < linear_velocity) {
        target_alt = alt_cm + climb_rate_cms/g.pi_alt_hold.kP();
    } else {
        linear_distance = ALT_HOLD_ACCEL_MAX/(2*g.pi_alt_hold.kP()*g.pi_alt_hold.kP());
        if (climb_rate_cms > 0){
            target_alt = alt_cm + linear_distance + (int32_t)climb_rate_cms*(int32_t)climb_rate_cms/(2*ALT_HOLD_ACCEL_MAX);
        } else {
            target_alt = alt_cm - ( linear_distance + (int32_t)climb_rate_cms*(int32_t)climb_rate_cms/(2*ALT_HOLD_ACCEL_MAX) );
        }
    }
    return constrain_int32(target_alt, alt_cm - ALT_HOLD_INIT_MAX_OVERSHOOT, alt_cm + ALT_HOLD_INIT_MAX_OVERSHOOT);
}

// get_throttle_rate - calculates desired accel required to achieve desired z_target_speed
// sets accel based throttle controller target
static void
get_throttle_rate(float z_target_speed)
{
    static uint32_t last_call_ms = 0;
    static float z_rate_error = 0;   // The velocity error in cm.
    static float z_target_speed_filt = 0;   // The filtered requested speed
    float z_target_speed_delta;   // The change in requested speed
    int32_t p;          // used to capture pid values for logging
    int32_t output;     // the target acceleration if the accel based throttle is enabled, otherwise the output to be sent to the motors
    uint32_t now = millis();

    // reset target altitude if this controller has just been engaged
    if( now - last_call_ms > 100 ) {
        // Reset Filter
        z_rate_error = 0;
        z_target_speed_filt = z_target_speed;
        output = 0;
    } else {
        // calculate rate error and filter with cut off frequency of 2 Hz
        z_rate_error    = z_rate_error + 0.20085f * ((z_target_speed - climb_rate) - z_rate_error);
        // feed forward acceleration based on change in the filtered desired speed.
        z_target_speed_delta = 0.20085f * (z_target_speed - z_target_speed_filt);
        z_target_speed_filt    = z_target_speed_filt + z_target_speed_delta;
        output = z_target_speed_delta * 50.0f;   // To-Do: replace 50 with dt
    }
    last_call_ms = now;

    // calculate p
    p = g.pid_throttle_rate.kP() * z_rate_error;

    // consolidate and constrain target acceleration
    output += p;
    output = constrain_int32(output, -32000, 32000);

    // set target for accel based throttle controller
    set_throttle_accel_target(output);

    // update throttle cruise
    // TO-DO: this may not be correct because g.rc_3.servo_out has not been updated for this iteration
    if( z_target_speed == 0 ) {
        update_throttle_cruise(g.rc_3.servo_out);
    }
}

// get_throttle_althold - hold at the desired altitude in cm
// updates accel based throttle controller targets
// Note: max_climb_rate is an optional parameter to allow reuse of this function by landing controller
static void
get_throttle_althold(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate)
{
    int32_t alt_error;
    float desired_rate;
    int32_t linear_distance;      // half the distace we swap between linear and sqrt and the distace we offset sqrt.

    // calculate altitude error
    alt_error    = target_alt - current_loc.alt;

    // check kP to avoid division by zero
    if( g.pi_alt_hold.kP() != 0 ) {
        linear_distance = ALT_HOLD_ACCEL_MAX/(2*g.pi_alt_hold.kP()*g.pi_alt_hold.kP());
        if( alt_error > 2*linear_distance ) {
            desired_rate = safe_sqrt(2*ALT_HOLD_ACCEL_MAX*(alt_error-linear_distance));
        }else if( alt_error < -2*linear_distance ) {
            desired_rate = -safe_sqrt(2*ALT_HOLD_ACCEL_MAX*(-alt_error-linear_distance));
        }else{
            desired_rate = g.pi_alt_hold.get_p(alt_error);
        }
    }else{
        desired_rate = 0;
    }

    desired_rate = constrain_float(desired_rate, min_climb_rate, max_climb_rate);

    // call rate based throttle controller which will update accel based throttle controller targets
    get_throttle_rate(desired_rate);

    // update altitude error reported to GCS
    altitude_error = alt_error;

    // TO-DO: enabled PID logging for this controller
}

// get_throttle_althold_with_slew - altitude controller with slew to avoid step changes in altitude target
// calls normal althold controller which updates accel based throttle controller targets
static void
get_throttle_althold_with_slew(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate)
{
    float alt_change = target_alt-controller_desired_alt;
    // adjust desired alt if motors have not hit their limits
    if ((alt_change<0 && !motors.limit.throttle_lower) || (alt_change>0 && !motors.limit.throttle_upper)) {
        controller_desired_alt += constrain_float(alt_change, min_climb_rate*0.02f, max_climb_rate*0.02f);
    }

    // do not let target altitude get too far from current altitude
    controller_desired_alt = constrain_float(controller_desired_alt,current_loc.alt-750,current_loc.alt+750);

    get_throttle_althold(controller_desired_alt, min_climb_rate-250, max_climb_rate+250);   // 250 is added to give head room to alt hold controller
}

// get_throttle_rate_stabilized - rate controller with additional 'stabilizer'
// 'stabilizer' ensure desired rate is being met
// calls normal throttle rate controller which updates accel based throttle controller targets
static void
get_throttle_rate_stabilized(int16_t target_rate)
{
    // adjust desired alt if motors have not hit their limits
    if ((target_rate<0 && !motors.limit.throttle_lower) || (target_rate>0 && !motors.limit.throttle_upper)) {
        controller_desired_alt += target_rate * 0.02f;
    }

    // do not let target altitude get too far from current altitude
    controller_desired_alt = constrain_float(controller_desired_alt,current_loc.alt-750,current_loc.alt+750);

#if AC_FENCE == ENABLED
    // do not let target altitude be too close to the fence
    // To-Do: add this to other altitude controllers
    if((fence.get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) != 0) {
        float alt_limit = fence.get_safe_alt() * 100.0f;
        if (controller_desired_alt > alt_limit) {
            controller_desired_alt = alt_limit;
        }
    }
#endif

    // update target altitude for reporting purposes
    set_target_alt_for_reporting(controller_desired_alt);

    get_throttle_althold(controller_desired_alt, -g.pilot_velocity_z_max-250, g.pilot_velocity_z_max+250);   // 250 is added to give head room to alt hold controller
}

////// PLANE SECTION

//****************************************************************
// Function that controls aileron/rudder, elevator, rudder (if 4 channel control) and throttle to produce desired attitude and airspeed.
//****************************************************************


/*
  get a speed scaling number for control surfaces. This is applied to
  PIDs to change the scaling of the PID with speed. At high speed we
  move the surfaces less, and at low speeds we move them more.
 */
static float get_speed_scaler(void)
{
    float aspeed, speed_scaler;
    if (ahrs.airspeed_estimate(&aspeed)) {
        if (aspeed > 0) {
            speed_scaler = g.scaling_speed / aspeed;
        } else {
            speed_scaler = 2.0;
        }
        speed_scaler = constrain_float(speed_scaler, 0.5, 2.0);
    } else {
        if (channel_throttle->servo_out > 0) {
            speed_scaler = 0.5 + ((float)PLTHR_CRUISE / channel_throttle->servo_out / 2.0);                 // First order taylor expansion of square root
            // Should maybe be to the 2/7 power, but we aren't goint to implement that...
        }else{
            speed_scaler = 1.67;
        }
        // This case is constrained tighter as we don't have real speed info
        speed_scaler = constrain_float(speed_scaler, 0.6, 1.67);
    }
    return speed_scaler;
}

/*
  return true if the current settings and mode should allow for stick mixing
 */
static bool stick_mixing_enabled(void)
{
    if (auto_throttle_mode) {
        // we're in an auto mode. Check the stick mixing flag
        if (g.stick_mixing != STICK_MIXING_DISABLED &&
            geofence_stickmixing() &&
            plane_failsafe.state == FAILSAFE_NONE &&
            !throttle_failsafe_level()) {
            // we're in an auto mode, and haven't triggered failsafe
            return true;
        } else {
            return false;
        }
    }

    if (plane_failsafe.ch3_failsafe && g.short_fs_action == 2) {
        // don't do stick mixing in FBWA glide mode
        return false;
    }

    // non-auto mode. Always do stick mixing
    return true;
}


/*
  this is the main roll stabilization function. It takes the
  previously set nav_roll calculates roll servo_out to try to
  stabilize the plane at the given roll
 */
static void stabilize_roll(float speed_scaler)
{
    if (inverted_flight) {
        // we want to fly upside down. We need to cope with wrap of
        // the roll_sensor interfering with wrap of nav_roll, which
        // would really confuse the PID code. The easiest way to
        // handle this is to ensure both go in the same direction from
        // zero
        nav_roll_cd += 18000;
        if (ahrs.roll_sensor < 0) nav_roll_cd -= 36000;
    }

    bool disable_integrator = false;
    if (plane_control_mode == PLANE_STABILIZE && channel_roll->control_in != 0) {
        disable_integrator = true;
    }
    channel_roll->servo_out = rollController.get_servo_out(nav_roll_cd - ahrs.roll_sensor, 
                                                           speed_scaler, 
                                                           disable_integrator);
}

/*
  this is the main pitch stabilization function. It takes the
  previously set nav_pitch and calculates servo_out values to try to
  stabilize the plane at the given attitude.
 */
static void stabilize_pitch(float speed_scaler)
{
    int32_t demanded_pitch = nav_pitch_cd + g.pitch_trim_cd + channel_throttle->servo_out * g.kff_throttle_to_pitch;
    bool disable_integrator = false;
    if (plane_control_mode == PLANE_STABILIZE && channel_pitch->control_in != 0) {
        disable_integrator = true;
    }
    channel_pitch->servo_out = pitchController.get_servo_out(demanded_pitch - ahrs.pitch_sensor, 
                                                             speed_scaler, 
                                                             disable_integrator);
}

/*
  perform stick mixing on one channel
  This type of stick mixing reduces the influence of the auto
  controller as it increases the influence of the users stick input,
  allowing the user full deflection if needed
 */
static void stick_mix_channel(RC_Channel *channel)
{
    float ch_inf;
        
    ch_inf = (float)channel->radio_in - (float)channel->radio_trim;
    ch_inf = fabsf(ch_inf);
    ch_inf = min(ch_inf, 400.0);
    ch_inf = ((400.0 - ch_inf) / 400.0);
    channel->servo_out *= ch_inf;
    channel->servo_out += channel->pwm_to_angle();
}

/*
  this gives the user control of the aircraft in stabilization modes
 */
static void stabilize_stick_mixing_direct()
{
    if (!stick_mixing_enabled() ||
        plane_control_mode == PLANE_ACRO ||
        plane_control_mode == PLANE_FLY_BY_WIRE_A ||
        plane_control_mode == PLANE_FLY_BY_WIRE_B ||
        plane_control_mode == PLANE_CRUISE ||
        plane_control_mode == PLANE_TRAINING) {
        return;
    }
    stick_mix_channel(channel_roll);
    stick_mix_channel(channel_pitch);
}

/*
  this gives the user control of the aircraft in stabilization modes
  using FBW style controls
 */
static void stabilize_stick_mixing_fbw()
{
    if (!stick_mixing_enabled() ||
        plane_control_mode == PLANE_ACRO ||
        plane_control_mode == PLANE_FLY_BY_WIRE_A ||
        plane_control_mode == PLANE_FLY_BY_WIRE_B ||
        plane_control_mode == PLANE_CRUISE ||
        plane_control_mode == PLANE_TRAINING ||
        (plane_control_mode == PLANE_AUTO && g.auto_fbw_steer)) {
        return;
    }
    // do FBW style stick mixing. We don't treat it linearly
    // however. For inputs up to half the maximum, we use linear
    // addition to the nav_roll and nav_pitch. Above that it goes
    // non-linear and ends up as 2x the maximum, to ensure that
    // the user can direct the plane in any direction with stick
    // mixing.
    float roll_input = channel_roll->norm_input();
    if (roll_input > 0.5f) {
        roll_input = (3*roll_input - 1);
    } else if (roll_input < -0.5f) {
        roll_input = (3*roll_input + 1);
    }
    nav_roll_cd += roll_input * roll_limit_cd;
    nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
    
    float pitch_input = channel_pitch->norm_input();
    if (fabsf(pitch_input) > 0.5f) {
        pitch_input = (3*pitch_input - 1);
    }
    if (inverted_flight) {
        pitch_input = -pitch_input;
    }
    if (pitch_input > 0) {
        nav_pitch_cd += pitch_input * aparm.pitch_limit_max_cd;
    } else {
        nav_pitch_cd += -(pitch_input * pitch_limit_min_cd);
    }
    nav_pitch_cd = constrain_int32(nav_pitch_cd, pitch_limit_min_cd, aparm.pitch_limit_max_cd.get());
}


/*
  stabilize the yaw axis. There are 3 modes of operation:

    - hold a specific heading with ground steering
    - rate controlled with ground steering
    - yaw control for coordinated flight    
 */
static void stabilize_yaw(float speed_scaler)
{
    bool ground_steering = (channel_roll->control_in == 0 && fabsf(relative_altitude()) < g.ground_steer_alt);

    if (steer_state.hold_course_cd != -1 && ground_steering) {
        calc_nav_yaw_course();
    } else if (ground_steering) {
        calc_nav_yaw_ground();
    } else {
        calc_nav_yaw_coordinated(speed_scaler);
    }
}


/*
  a special stabilization function for training mode
 */
static void stabilize_training(float speed_scaler)
{
    if (training_manual_roll) {
        channel_roll->servo_out = channel_roll->control_in;
    } else {
        // calculate what is needed to hold
        stabilize_roll(speed_scaler);
        if ((nav_roll_cd > 0 && channel_roll->control_in < channel_roll->servo_out) ||
            (nav_roll_cd < 0 && channel_roll->control_in > channel_roll->servo_out)) {
            // allow user to get out of the roll
            channel_roll->servo_out = channel_roll->control_in;            
        }
    }

    if (training_manual_pitch) {
        channel_pitch->servo_out = channel_pitch->control_in;
    } else {
        stabilize_pitch(speed_scaler);
        if ((nav_pitch_cd > 0 && channel_pitch->control_in < channel_pitch->servo_out) ||
            (nav_pitch_cd < 0 && channel_pitch->control_in > channel_pitch->servo_out)) {
            // allow user to get back to level
            channel_pitch->servo_out = channel_pitch->control_in;            
        }
    }

    stabilize_yaw(speed_scaler);
}


/*
  this is the PLANE_ACRO mode stabilization function. It does rate
  stabilization on roll and pitch axes
 */
static void stabilize_acro(float speed_scaler)
{
    float roll_rate = (channel_roll->control_in/4500.0f) * g.acro_roll_ratep;
    float pitch_rate = (channel_pitch->control_in/4500.0f) * g.acro_pitch_ratep;

    /*
      check for special roll handling near the pitch poles
     */
    if (g.acro_locking && roll_rate == 0) {
        /*
          we have no roll stick input, so we will enter "roll locked"
          mode, and hold the roll we had when the stick was released
         */
        if (!acro_state.locked_roll) {
            acro_state.locked_roll = true;
            acro_state.locked_roll_err = 0;
        } else {
            acro_state.locked_roll_err += ahrs.get_gyro().x * G_Dt;
        }
        int32_t roll_error_cd = -ToDeg(acro_state.locked_roll_err)*100;
        nav_roll_cd = ahrs.roll_sensor + roll_error_cd;
        // try to reduce the integrated angular error to zero. We set
        // 'stabilze' to true, which disables the roll integrator
        channel_roll->servo_out  = rollController.get_servo_out(roll_error_cd,
                                                                speed_scaler,
                                                                true);
    } else {
        /*
          aileron stick is non-zero, use pure rate control until the
          user releases the stick
         */
        acro_state.locked_roll = false;
        channel_roll->servo_out  = rollController.get_rate_out(roll_rate,  speed_scaler);
    }

    if (g.acro_locking && pitch_rate == 0) {
        /*
          user has zero pitch stick input, so we lock pitch at the
          point they release the stick
         */
        if (!acro_state.locked_pitch) {
            acro_state.locked_pitch = true;
            acro_state.locked_pitch_cd = ahrs.pitch_sensor;
        }
        // try to hold the locked pitch. Note that we have the pitch
        // integrator enabled, which helps with inverted flight
        nav_pitch_cd = acro_state.locked_pitch_cd;
        channel_pitch->servo_out  = pitchController.get_servo_out(nav_pitch_cd - ahrs.pitch_sensor,
                                                                  speed_scaler,
                                                                  false);
    } else {
        /*
          user has non-zero pitch input, use a pure rate controller
         */
        acro_state.locked_pitch = false;
        channel_pitch->servo_out = pitchController.get_rate_out(pitch_rate, speed_scaler);
    }

    /*
      manual rudder for now
     */
    channel_rudder->servo_out = channel_rudder->control_in;
}

/*
  main stabilization function for all 3 axes
 */
static void plane_stabilize()
{
    if (plane_control_mode == PLANE_MANUAL) {
        // nothing to do
        return;
    }
    float speed_scaler = get_speed_scaler();

    if (plane_control_mode == PLANE_TRAINING) {
        stabilize_training(speed_scaler);
    } else if (plane_control_mode == PLANE_ACRO) {
        stabilize_acro(speed_scaler);
    } else {
        if (g.stick_mixing == STICK_MIXING_FBW && plane_control_mode != PLANE_STABILIZE) {
            stabilize_stick_mixing_fbw();
        }
        stabilize_roll(speed_scaler);
        stabilize_pitch(speed_scaler);
        if (g.stick_mixing == STICK_MIXING_DIRECT || plane_control_mode == PLANE_STABILIZE) {
            stabilize_stick_mixing_direct();
        }
        stabilize_yaw(speed_scaler);
    }

    /*
      see if we should zero the attitude controller integrators. 
     */
    if (channel_throttle->control_in == 0 &&
        relative_altitude_abs_cm() < 500 && 
        fabs(barometer.get_climb_rate()) < 0.5f &&
        g_gps->ground_speed_cm < 300) {
        // we are low, with no climb rate, and zero throttle, and very
        // low ground speed. Zero the attitude controller
        // integrators. This prevents integrator buildup pre-takeoff.
        rollController.reset_I();
        pitchController.reset_I();
        yawController.reset_I();
    }
}


static void calc_throttle()
{
    if (aparm.plthr_cruise <= 1) {
        // user has asked for zero throttle - this may be done by a
        // mission which wants to turn off the engine for a parachute
        // landing
        channel_throttle->servo_out = 0;
        return;
    }

    channel_throttle->servo_out = SpdHgt_Controller->get_throttle_demand();
}

/*****************************************
* Calculate desired roll/pitch/yaw angles (in medium freq loop)
*****************************************/

/*
  calculate yaw control for coordinated flight
 */
static void calc_nav_yaw_coordinated(float speed_scaler)
{
    bool disable_integrator = false;
    if (plane_control_mode == PLANE_STABILIZE && channel_rudder->control_in != 0) {
        disable_integrator = true;
    }
    channel_rudder->servo_out = yawController.get_servo_out(speed_scaler, disable_integrator);

    // add in rudder mixing from roll
    channel_rudder->servo_out += channel_roll->servo_out * g.kff_rudder_mix;
    channel_rudder->servo_out += channel_rudder->control_in;
    channel_rudder->servo_out = constrain_int16(channel_rudder->servo_out, -4500, 4500);
}

/*
  calculate yaw control for ground steering with specific course
 */
static void calc_nav_yaw_course(void)
{
    // holding a specific navigation course on the ground. Used in
    // auto-takeoff and landing
    int32_t bearing_error_cd = nav_controller->bearing_error_cd();
    channel_rudder->servo_out = steerController.get_steering_out_angle_error(bearing_error_cd);
    if (stick_mixing_enabled()) {
        stick_mix_channel(channel_rudder);
    }
    channel_rudder->servo_out = constrain_int16(channel_rudder->servo_out, -4500, 4500);
}

/*
  calculate yaw control for ground steering
 */
static void calc_nav_yaw_ground(void)
{
    if (g_gps->ground_speed_cm < 100 && 
        channel_throttle->control_in == 0) {
        // manual rudder control while still
        steer_state.locked_course = false;
        steer_state.locked_course_err = 0;
        channel_rudder->servo_out = channel_rudder->control_in;
        return;
    }

    float steer_rate = (channel_rudder->control_in/4500.0f) * g.ground_steer_dps;
    if (steer_rate != 0) {
        // pilot is giving rudder input
        steer_state.locked_course = false;        
    } else if (!steer_state.locked_course) {
        // pilot has released the rudder stick or we are still - lock the course
        steer_state.locked_course = true;
        steer_state.locked_course_err = 0;
    }
    if (!steer_state.locked_course) {
        // use a rate controller at the pilot specified rate
        channel_rudder->servo_out = steerController.get_steering_out_rate(steer_rate);
    } else {
        // use a error controller on the summed error
        steer_state.locked_course_err += ahrs.get_gyro().z * G_Dt;
        int32_t yaw_error_cd = -ToDeg(steer_state.locked_course_err)*100;
        channel_rudder->servo_out = steerController.get_steering_out_angle_error(yaw_error_cd);
    }
    channel_rudder->servo_out = constrain_int16(channel_rudder->servo_out, -4500, 4500);
}


static void calc_nav_pitch()
{
    // Calculate the Pitch of the plane
    // --------------------------------
    nav_pitch_cd = SpdHgt_Controller->get_pitch_demand();
    nav_pitch_cd = constrain_int32(nav_pitch_cd, pitch_limit_min_cd, aparm.pitch_limit_max_cd.get());
}


static void calc_nav_roll()
{
    nav_roll_cd = nav_controller->nav_roll_cd();
    nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
}


/*****************************************
* Throttle slew limit
*****************************************/
static void throttle_slew_limit(int16_t last_throttle)
{
    // if slew limit rate is set to zero then do not slew limit
    if (aparm.throttle_slewrate) {                   
        // limit throttle change by the given percentage per second
        float temp = aparm.throttle_slewrate * G_Dt * 0.01f * fabsf(channel_throttle->radio_max - channel_throttle->radio_min);
        // allow a minimum change of 1 PWM per cycle
        if (temp < 1) {
            temp = 1;
        }
        channel_throttle->radio_out = constrain_int16(channel_throttle->radio_out, last_throttle - temp, last_throttle + temp);
    }
}


/*   Check for automatic takeoff conditions being met using the following sequence:
 *   1) Check for adequate GPS lock - if not return false
 *   2) Check the gravity compensated longitudinal acceleration against the threshold and start the timer if true
 *   3) Wait until the timer has reached the specified value (increments of 0.1 sec) and then check the GPS speed against the threshold
 *   4) If the GPS speed is above the threshold and the attitude is within limits then return true and reset the timer
 *   5) If the GPS speed and attitude within limits has not been achieved after 2.5 seconds, return false and reset the timer
 *   6) If the time lapsed since the last timecheck is greater than 0.2 seconds, return false and reset the timer
 *   NOTE : This function relies on the TECS 50Hz processing for its acceleration measure.
 */
static bool auto_takeoff_check(void)
{
    // this is a more advanced check that relies on TECS
    uint32_t now = hal.scheduler->millis();
    static bool launchTimerStarted;
    static uint32_t last_tkoff_arm_time;
    static uint32_t last_check_ms;

    // Reset states if process has been interrupted
    if (last_check_ms && (now - last_check_ms) > 200) {
        plane_gcs_send_text_fmt(PSTR("Timer Interrupted PLANE_AUTO"));
	    launchTimerStarted = false;
	    last_tkoff_arm_time = 0;
        last_check_ms = now;
        return false;
    }

    last_check_ms = now;

    // Check for bad GPS
    if (g_gps == NULL || g_gps->status() != GPS::GPS_OK_FIX_3D) {
        // no auto takeoff without GPS lock
        return false;
    }

    // Check for launch acceleration or timer started. NOTE: relies on TECS 50Hz processing
    if (!launchTimerStarted &&
        g.takeoff_throttle_min_accel != 0.0 &&
        SpdHgt_Controller->get_VXdot() < g.takeoff_throttle_min_accel) {
        goto no_launch;
    }

    // we've reached the acceleration threshold, so start the timer
    if (!launchTimerStarted) {
        launchTimerStarted = true;
        last_tkoff_arm_time = now;
        plane_gcs_send_text_fmt(PSTR("Armed PLANE_AUTO, xaccel = %.1f m/s/s, waiting %.1f sec"), 
                          SpdHgt_Controller->get_VXdot(), 0.1f*float(min(g.takeoff_throttle_delay,25)));
    }

    // Only perform velocity check if not timed out
    if ((now - last_tkoff_arm_time) > 2500) {
        plane_gcs_send_text_fmt(PSTR("Timeout PLANE_AUTO"));
        goto no_launch;
    }

    // Check aircraft attitude for bad launch
    if (ahrs.pitch_sensor <= -3000 ||
        ahrs.pitch_sensor >= 4500 ||
        abs(ahrs.roll_sensor) > 3000) {
        plane_gcs_send_text_fmt(PSTR("Bad Launch PLANE_AUTO"));
        goto no_launch;
    }

    // Check ground speed and time delay
    if (((g_gps->ground_speed_cm > g.takeoff_throttle_min_speed*100.0f || g.takeoff_throttle_min_speed == 0.0)) && 
        ((now - last_tkoff_arm_time) >= min(uint16_t(g.takeoff_throttle_delay)*100,2500))) {
        plane_gcs_send_text_fmt(PSTR("Triggered PLANE_AUTO, GPSspd = %.1f"), g_gps->ground_speed_cm*0.01f);
        launchTimerStarted = false;
        last_tkoff_arm_time = 0;
        return true;
    }

    // we're not launching yet, but the timer is still going
    return false;

no_launch:
    launchTimerStarted = false;
    last_tkoff_arm_time = 0;
    return false;
}


/**
  Do we think we are flying?
  This is a heuristic so it could be wrong in some cases.  In particular, if we don't have GPS lock we'll fall
  back to only using altitude.  (This is probably more optimistic than what suppress_throttle wants...)
*/
static bool is_flying(void)
{
    // If we don't have a GPS lock then don't use GPS for this test
    bool gpsMovement = (g_gps == NULL ||
                        g_gps->status() < GPS::GPS_OK_FIX_2D ||
                        g_gps->ground_speed_cm >= 500);
    
    bool airspeedMovement = !airspeed.use() || airspeed.get_airspeed() >= 5;
    
    // we're more than 5m from the home altitude
    bool inAir = relative_altitude_abs_cm() > 500;

    return inAir && gpsMovement && airspeedMovement;
}


/* We want to supress the throttle if we think we are on the ground and in an autopilot controlled throttle mode.

   Disable throttle if following conditions are met:
   *       1 - We are in Circle mode (which we use for short term failsafe), or in FBW-B or higher
   *       AND
   *       2 - Our reported altitude is within 10 meters of the home altitude.
   *       3 - Our reported speed is under 5 meters per second.
   *       4 - We are not performing a takeoff in Auto mode or takeoff speed/accel not yet reached
   *       OR
   *       5 - Home location is not set
*/
static bool suppress_throttle(void)
{
    if (!throttle_suppressed) {
        // we've previously met a condition for unsupressing the throttle
        return false;
    }
    if (!auto_throttle_mode) {
        // the user controls the throttle
        throttle_suppressed = false;
        return false;
    }

    if (plane_control_mode==PLANE_AUTO && g.auto_fbw_steer) {
        // user has throttle control
        return false;
    }

    if (plane_control_mode==PLANE_AUTO && ap.takeoff_complete == false && auto_takeoff_check()) {
        // we're in auto takeoff 
        throttle_suppressed = false;
        if (steer_state.hold_course_cd != -1) {
            // update takeoff course hold, if already initialised
            steer_state.hold_course_cd = ahrs.yaw_sensor;
            plane_gcs_send_text_fmt(PSTR("Holding course %ld"), steer_state.hold_course_cd);
        }
        return false;
    }
    
    if (relative_altitude_abs_cm() >= 1000) {
        // we're more than 10m from the home altitude
        throttle_suppressed = false;
        return false;
    }

    if (g_gps != NULL && 
        g_gps->status() >= GPS::GPS_OK_FIX_2D && 
        g_gps->ground_speed_cm >= 500) {
        // if we have an airspeed sensor, then check it too, and
        // require 5m/s. This prevents throttle up due to spiky GPS
        // groundspeed with bad GPS reception
        if (!airspeed.use() || airspeed.get_airspeed() >= 5) {
            // we're moving at more than 5 m/s
            throttle_suppressed = false;
            return false;        
        }
    }

    // throttle remains suppressed
    return true;
}

/*
  implement a software VTail or elevon mixer. There are 4 different mixing modes
 */
static void channel_output_mixer(uint8_t mixing_type, int16_t &chan1_out, int16_t &chan2_out)
{
    int16_t c1, c2;
    int16_t v1, v2;

    // first get desired elevator and rudder as -500..500 values
    c1 = chan1_out - 1500;
    c2 = chan2_out - 1500;

    v1 = (c1 - c2) * g.mixing_gain;
    v2 = (c1 + c2) * g.mixing_gain;

    // now map to mixed output
    switch (mixing_type) {
    case MIXING_DISABLED:
        return;

    case MIXING_UPUP:
        break;

    case MIXING_UPDN:
        v2 = -v2;
        break;

    case MIXING_DNUP:
        v1 = -v1;
        break;

    case MIXING_DNDN:
        v1 = -v1;
        v2 = -v2;
        break;
    }

    // scale for a 1500 center and 900..2100 range, symmetric
    v1 = constrain_int16(v1, -600, 600);
    v2 = constrain_int16(v2, -600, 600);

    chan1_out = 1500 + v1;
    chan2_out = 1500 + v2;
}

/*****************************************
* Set the flight control servos based on the current calculated values
*****************************************/

 // PLANE TO COPTER MOTOR MAP  - NEEDS WORK
void throttle_plane_to_copter(){ 
    #if (FRAME_CONFIG == QUAD_FRAME)  // NEEDS WORK, THIS IS FOR X FRAME
        hal.rcout->write(0, channel_throttle->radio_out);	// motor 1
		hal.rcout->write(1, channel_throttle->radio_min);
	    hal.rcout->write(2, channel_throttle->radio_out);	// motor 3
		hal.rcout->write(3, channel_throttle->radio_min);
		    // Route configurable aux. functions to their respective servos
		g.rc_5.output_ch(CH_5);
		g.rc_6.output_ch(CH_6);
		g.rc_7.output_ch(CH_7);
		g.rc_8.output_ch(CH_8);
		
    #elif (FRAME_CONFIG == TRI_FRAME)
		hal.rcout->write(0, channel_throttle->radio_out);	// motor 1
	    hal.rcout->write(1, channel_throttle->radio_out);	// motor 2
		hal.rcout->write(2, channel_throttle->radio_trim); // center the tail servo
		hal.rcout->write(3, channel_throttle->radio_min);
		    // Route configurable aux. functions to their respective servos
		g.rc_5.output_ch(CH_5);
		g.rc_6.output_ch(CH_6);
		g.rc_7.output_ch(CH_7);
		g.rc_8.output_ch(CH_8);
		
    #elif (FRAME_CONFIG == Y6_FRAME)  // FRAME Y6B!!!
		hal.rcout->write(0, channel_throttle->radio_out);	// motor 1
		hal.rcout->write(1, channel_throttle->radio_out);	// motor 2
	    hal.rcout->write(2, channel_throttle->radio_min);
		hal.rcout->write(3, channel_throttle->radio_min);
	    hal.rcout->write(4, channel_throttle->radio_out);	// motor 5
		hal.rcout->write(5, channel_throttle->radio_out);	// motor 6
		    // Route configurable aux. functions to their respective servos
		g.rc_7.output_ch(CH_7);
		g.rc_8.output_ch(CH_8);

	#elif (FRAME_CONFIG == OCTA_QUAD_FRAME)  // FRAME X
		hal.rcout->write(0, channel_throttle->radio_out);	// motor 1
		hal.rcout->write(1, channel_throttle->radio_out);	// motor 2
	    hal.rcout->write(2, channel_throttle->radio_min);
		hal.rcout->write(3, channel_throttle->radio_min);
	    hal.rcout->write(4, channel_throttle->radio_out);	// motor 5
		hal.rcout->write(5, channel_throttle->radio_out);	// motor 6
	    hal.rcout->write(6, channel_throttle->radio_min);
		hal.rcout->write(7, channel_throttle->radio_min);
    #else
        // throw compile error if frame type is unrecognise
        #error Unrecognised frame type
    #endif 
}

static void plane_set_servos(void)
{
    int16_t last_throttle = channel_throttle->radio_out;

    if (plane_control_mode == PLANE_MANUAL) {
        // do a direct pass through of radio values
        if (g.mix_mode == 0 || g.elevon_output != MIXING_DISABLED) {
            channel_roll->radio_out                	= channel_roll->radio_in;
            channel_pitch->radio_out               	= channel_pitch->radio_in;
        } else {
            channel_roll->radio_out                	= channel_roll->read();
            channel_pitch->radio_out               	= channel_pitch->read();
        }
			channel_throttle->radio_out    			= channel_throttle->radio_in;
			channel_rudder->radio_out             	= channel_rudder->radio_in;

        // setup extra channels. We want this to come from the
        // main input channel, but using the 2nd channels dead
        // zone, reverse and min/max settings. We need to use
        // pwm_to_angle_dz() to ensure we don't trim the value for the
        // deadzone of the main aileron channel, otherwise the 2nd
        // aileron won't quite follow the first one
//        RC_Channel_aux::set_servo_out(RC_Channel_aux::k_aileron, channel_roll->pwm_to_angle_dz(0));
//        RC_Channel_aux::set_servo_out(RC_Channel_aux::k_elevator, channel_pitch->pwm_to_angle_dz(0));
//        RC_Channel_aux::set_servo_out(RC_Channel_aux::k_rudder, channel_rudder->pwm_to_angle_dz(0));

        // this variant assumes you have the corresponding
        // input channel setup in your transmitter for manual control
        // of the 2nd aileron
//        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_aileron_with_input);
//        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_elevator_with_input);
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_flap_auto);

        if (g.mix_mode == 0 && g.elevon_output == MIXING_DISABLED) {
            // set any differential spoilers to follow the elevons in
            // manual mode. 
            RC_Channel_aux::set_radio(RC_Channel_aux::k_dspoiler1, channel_roll->radio_out);
            RC_Channel_aux::set_radio(RC_Channel_aux::k_dspoiler2, channel_pitch->radio_out);
        }
    } else {
        if (g.mix_mode == 0) {
            // both types of secondary aileron are slaved to the roll servo out
//            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_aileron, channel_roll->servo_out);
//            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_aileron_with_input, channel_roll->servo_out);

            // both types of secondary elevator are slaved to the pitch servo out
//            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_elevator, channel_pitch->servo_out);
//            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_elevator_with_input, channel_pitch->servo_out);

            // setup secondary rudder
//            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_rudder, channel_rudder->servo_out);
        }else{
            /*Elevon mode*/
            float ch1;
            float ch2;
            ch1 = channel_pitch->servo_out - (BOOL_TO_SIGN(g.reverse_elevons) * channel_roll->servo_out);
            ch2 = channel_pitch->servo_out + (BOOL_TO_SIGN(g.reverse_elevons) * channel_roll->servo_out);

			/* Differential Spoilers
               If differential spoilers are setup, then we translate
               rudder control into splitting of the two ailerons on
               the side of the aircraft where we want to induce
               additional drag.
             */
			if (RC_Channel_aux::function_assigned(RC_Channel_aux::k_dspoiler1) && RC_Channel_aux::function_assigned(RC_Channel_aux::k_dspoiler2)) {
				float ch3 = ch1;
				float ch4 = ch2;
				if ( BOOL_TO_SIGN(g.reverse_elevons) * channel_rudder->servo_out < 0) {
				    ch1 += abs(channel_rudder->servo_out);
				    ch3 -= abs(channel_rudder->servo_out);
				} else {
					ch2 += abs(channel_rudder->servo_out);
				    ch4 -= abs(channel_rudder->servo_out);
				}
				RC_Channel_aux::set_servo_out(RC_Channel_aux::k_dspoiler1, ch3);
				RC_Channel_aux::set_servo_out(RC_Channel_aux::k_dspoiler2, ch4);
			}

            // directly set the radio_out values for elevon mode
            channel_roll->radio_out  =     elevon.trim1 + (BOOL_TO_SIGN(g.reverse_ch1_elevon) * (ch1 * 500.0/ SERVO_MAX));
            channel_pitch->radio_out =     elevon.trim2 + (BOOL_TO_SIGN(g.reverse_ch2_elevon) * (ch2 * 500.0/ SERVO_MAX));
        }

#if OBC_FAILSAFE == ENABLED
        // this is to allow the failsafe module to deliberately crash 
        // the plane. Only used in extreme circumstances to meet the
        // OBC rules
        if (obc.crash_plane()) {
            channel_roll->servo_out = -4500;
            channel_pitch->servo_out = -4500;
            channel_rudder->servo_out = -4500;
            channel_throttle->servo_out = 0;
        }
#endif
        

        // push out the PWM values
        if (g.mix_mode == 0) {
            channel_roll->calc_pwm();
            channel_pitch->calc_pwm();
        }
        channel_rudder->calc_pwm();

#if THROTTLE_OUT == 0
        channel_throttle->servo_out = 0;
#else
        // convert 0 to 100% into PWM
        channel_throttle->servo_out = constrain_int16(channel_throttle->servo_out, 
                                                       aparm.plthr_min.get(), 
                                                       aparm.plthr_max.get());

        if (suppress_throttle()) {
            // throttle is suppressed in auto mode
            channel_throttle->servo_out = 0;
            if (g.throttle_suppress_manual) {
                // manual pass through of throttle while throttle is suppressed
                channel_throttle->radio_out = channel_throttle->radio_in;
            } else {
                channel_throttle->calc_pwm();                
            }
        } else if (g.throttle_passthru_stabilize && 
                   (plane_control_mode == PLANE_STABILIZE || 
                    plane_control_mode == PLANE_TRAINING ||
                    plane_control_mode == PLANE_ACRO ||
                    plane_control_mode == PLANE_FLY_BY_WIRE_A)) {
            // manual pass through of throttle while in FBWA or
            // PLANE_STABILIZE mode with THR_PASS_STAB set
            channel_throttle->radio_out = channel_throttle->radio_in;
        } else if (plane_control_mode == PLANE_GUIDED && 
                   guided_throttle_passthru) {
            // manual pass through of throttle while in PLANE_GUIDED
            channel_throttle->radio_out = channel_throttle->radio_in;
        } else {
            // normal throttle calculation based on servo_out
            channel_throttle->calc_pwm();
        }
#endif
    }

    // Auto flap deployment
    if(plane_control_mode < PLANE_FLY_BY_WIRE_B) {
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_flap_auto);
    } else if (plane_control_mode >= PLANE_FLY_BY_WIRE_B) {
        int16_t flapSpeedSource = 0;

        // FIXME: use target_airspeed in both FBW_B and g.airspeed_enabled cases - Doug?
        if (plane_control_mode == PLANE_FLY_BY_WIRE_B) {
            flapSpeedSource = target_airspeed_cm * 0.01;
        } else if (airspeed.use()) {
            flapSpeedSource = g.airspeed_cruise_cm * 0.01;
        } else {
            flapSpeedSource = aparm.plthr_cruise;
        }
        if ( g.flap_1_speed != 0 && flapSpeedSource > g.flap_1_speed) {
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_flap_auto, 0);
        } else if (g.flap_2_speed != 0 && flapSpeedSource > g.flap_2_speed) {
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_flap_auto, g.flap_1_percent);
        } else {
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_flap_auto, g.flap_2_percent);
        }
    }

    if (plane_control_mode >= PLANE_FLY_BY_WIRE_B) {
        /* only do throttle slew limiting in modes where throttle
         *  control is automatic */
        throttle_slew_limit(last_throttle);
    }

    if (plane_control_mode == PLANE_TRAINING) {
        // copy rudder in training mode
        channel_rudder->radio_out   = channel_rudder->radio_in;
    }

    if (g.vtail_output != MIXING_DISABLED) {
        channel_output_mixer(g.vtail_output, channel_pitch->radio_out, channel_rudder->radio_out);
    } else if (g.elevon_output != MIXING_DISABLED) {
        channel_output_mixer(g.elevon_output, channel_pitch->radio_out, channel_roll->radio_out);
    }

    //send throttle to 0 or MIN_PWM if not yet armed
    if (!arming.is_armed()) {
        //Some ESCs get noisy (beep error msgs) if PWM == 0.
        //This little segment aims to avoid this.
        switch (arming.arming_required()) { 
            case AP_Arming::YES_MIN_PWM:
                channel_throttle->radio_out = channel_throttle->radio_min;
            break;
            case AP_Arming::YES_ZERO_PWM:
                channel_throttle->radio_out = 0;
            break;
            default:
                //keep existing behavior: do nothing to radio_out
                //(don't disarm throttle channel even if AP_Arming class is)
            break;
        }
    }

#if HIL_MODE != HIL_MODE_DISABLED
    // get the servos to the GCS immediately for HIL
    if (comm_get_txspace(MAVLINK_COMM_0) - MAVLINK_NUM_NON_PAYLOAD_BYTES >= MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN) {
        plane_send_servo_out(MAVLINK_COMM_0);
    }
    if (!g.hil_servos) {
        return;
    }
#endif

    // send values to the PWM timers for output
    // ----------------------------------------

    RC_Channel_aux::set_radio(RC_Channel_aux::k_aileron, channel_roll->radio_out);
    RC_Channel_aux::set_radio(RC_Channel_aux::k_elevator, channel_pitch->radio_out);
    RC_Channel_aux::set_radio(RC_Channel_aux::k_rudder, channel_rudder->radio_out);
 	throttle_plane_to_copter();
      
    // Route configurable aux. functions to their respective servos

 #if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    g.rc_9.output_ch(CH_9);
 #endif
 #if CONFIG_HAL_BOARD == HAL_BOARD_APM2 || CONFIG_HAL_BOARD == HAL_BOARD_PX4
    g.rc_10.output_ch(CH_10);
    g.rc_11.output_ch(CH_11);
 #endif
 #if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    g.rc_12.output_ch(CH_12);
 #endif
}

static bool demoing_servos;

static void demo_servos(uint8_t i) 
{
    while(i > 0) {
        gcs_send_text_P(SEVERITY_LOW,PSTR("Demo Servos!"));
        demoing_servos = true;
        hal.rcout->write(7, 1400);
        mavlink_delay(400);
        hal.rcout->write(7, 1600);
        mavlink_delay(200);
        hal.rcout->write(7, 1500);
        demoing_servos = false;
        mavlink_delay(400);
        i--;
    }
}


////// END PLANE SECTION


// get_throttle_land - high level landing logic
// sends the desired acceleration in the accel based throttle controller
// called at 50hz
static void
get_throttle_land()
{
    // if we are above 10m and the sonar does not sense anything perform regular alt hold descent
    if (current_loc.alt >= LAND_START_ALT && !(g.sonar_enabled && sonar_alt_health >= SONAR_ALT_HEALTH_MAX)) {
        get_throttle_althold_with_slew(LAND_START_ALT, -wp_nav.get_descent_velocity(), -abs(g.land_speed));
    }else{
        get_throttle_rate_stabilized(-abs(g.land_speed));

        // disarm when the landing detector says we've landed and throttle is at min (or we're in failsafe so we have no pilot thorottle input)
#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
        if( ap.land_complete && (g.rc_3.control_in == 0 || failsafe.radio) ) {
#else
        if (ap.land_complete) {
#endif
            init_disarm_motors();
        }
    }
}

// reset_land_detector - initialises land detector
static void reset_land_detector()
{
    set_land_complete(false);
    land_detector = 0;
}

// update_land_detector - checks if we have landed and updates the ap.land_complete flag
// returns true if we have landed
static bool update_land_detector()
{
    // detect whether we have landed by watching for low climb rate and minimum throttle
    if (abs(climb_rate) < 20 && motors.limit.throttle_lower) {
        if (!ap.land_complete) {
            // run throttle controller if accel based throttle controller is enabled and active (active means it has been given a target)
            if( land_detector < LAND_DETECTOR_TRIGGER) {
                land_detector++;
            }else{
                set_land_complete(true);
                land_detector = 0;
            }
        }
    }else if (g.rc_3.control_in != 0 || failsafe.radio){    // zero throttle locks land_complete as true
        // we've sensed movement up or down so reset land_detector
        land_detector = 0;
        if(ap.land_complete) {
            set_land_complete(false);
        }
    }

    // return current state of landing
    return ap.land_complete;
}

// get_throttle_surface_tracking - hold copter at the desired distance above the ground
// updates accel based throttle controller targets
static void
get_throttle_surface_tracking(int16_t target_rate)
{
    static uint32_t last_call_ms = 0;
    float distance_error;
    float velocity_correction;

    uint32_t now = millis();

    // reset target altitude if this controller has just been engaged
    if( now - last_call_ms > 200 ) {
        target_sonar_alt = sonar_alt + controller_desired_alt - current_loc.alt;
    }
    last_call_ms = now;

    // adjust sonar target alt if motors have not hit their limits
    if ((target_rate<0 && !motors.limit.throttle_lower) || (target_rate>0 && !motors.limit.throttle_upper)) {
        target_sonar_alt += target_rate * 0.02f;
    }

    // do not let target altitude get too far from current altitude above ground
    // Note: the 750cm limit is perhaps too wide but is consistent with the regular althold limits and helps ensure a smooth transition
    target_sonar_alt = constrain_float(target_sonar_alt,sonar_alt-750,sonar_alt+750);

    // calc desired velocity correction from target sonar alt vs actual sonar alt
    distance_error = target_sonar_alt-sonar_alt;
    velocity_correction = distance_error * g.sonar_gain;
    velocity_correction = constrain_float(velocity_correction, -THR_SURFACE_TRACKING_VELZ_MAX, THR_SURFACE_TRACKING_VELZ_MAX);

    // call regular rate stabilize alt hold controller
    get_throttle_rate_stabilized(target_rate + velocity_correction);
}

/*
 *  reset all I integrators
 */
static void reset_I_all(void)
{
    reset_rate_I();
    reset_throttle_I();
    reset_optflow_I();
}

static void reset_rate_I()
{
    g.pid_rate_roll.reset_I();
    g.pid_rate_pitch.reset_I();
    g.pid_rate_yaw.reset_I();
}

static void reset_optflow_I(void)
{
    g.pid_optflow_roll.reset_I();
    g.pid_optflow_pitch.reset_I();
    of_roll = 0;
    of_pitch = 0;
}

static void reset_throttle_I(void)
{
    // For Altitude Hold
    g.pi_alt_hold.reset_I();
    g.pid_throttle_accel.reset_I();
}

static void set_accel_throttle_I_from_pilot_throttle(int16_t pilot_throttle)
{
    // shift difference between pilot's throttle and hover throttle into accelerometer I
    g.pid_throttle_accel.set_integrator(pilot_throttle-g.throttle_cruise);
}

#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/GCS_Mavlink.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// default sensors are present and healthy: gyro, accelerometer, barometer, rate_control, attitude_stabilization, yaw_position, altitude control, x/y position control, motor_control
#define MAVLINK_SENSOR_PRESENT_DEFAULT (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION | MAV_SYS_STATUS_SENSOR_YAW_POSITION | MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL | MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL | MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS)

// use this to prevent recursion during sensor init
static bool in_mavlink_delay;

// true when we have received at least 1 MAVLink packet
static bool mavlink_active;

// true if we are out of time in our event timeslice
static bool	gcs_out_of_time;

// check if a message will fit in the payload space available
#define CHECK_PAYLOAD_SIZE(id) if (payload_space < MAVLINK_MSG_ID_ ## id ## _LEN) return false

static void gcs_send_heartbeat(void)
{
    gcs_send_message(MSG_HEARTBEAT);
}

static void gcs_send_deferred(void)
{
    gcs_send_message(MSG_RETRY_DEFERRED);
}

/*
 *  !!NOTE!!
 *
 *  the use of NOINLINE separate functions for each message type avoids
 *  a compiler bug in gcc that would cause it to use far more stack
 *  space than is needed. Without the NOINLINE we use the sum of the
 *  stack needed for each message type. Please be careful to follow the
 *  pattern below when adding any new messages
 */

static NOINLINE void send_heartbeat(mavlink_channel_t chan)
{
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status = ap.land_complete ? MAV_STATE_STANDBY : MAV_STATE_ACTIVE;
    uint32_t custom_mode = control_mode;

    // set system as critical if any failsafe have triggered
    if (failsafe.radio || failsafe.battery || failsafe.gps || failsafe.gcs)  {
        system_status = MAV_STATE_CRITICAL;
    }
    
    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
    switch (control_mode) {
    case AUTO:
    case RTL:
    case LOITER:
    case GUIDED:
    case CIRCLE:
        base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    }

    // all modes except INITIALISING have some form of manual
    // override if stick mixing is enabled
    base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

#if HIL_MODE != HIL_MODE_DISABLED
    base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
#endif

    // we are armed if we are not initialising
    if (motors.armed()) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    mavlink_msg_heartbeat_send(
        chan,
#if (FRAME_CONFIG == QUAD_FRAME)
        MAV_TYPE_QUADROTOR,
#elif (FRAME_CONFIG == TRI_FRAME)
        MAV_TYPE_TRICOPTER,
#elif (FRAME_CONFIG == HEXA_FRAME || FRAME_CONFIG == Y6_FRAME)
        MAV_TYPE_HEXAROTOR,
#elif (FRAME_CONFIG == OCTA_FRAME || FRAME_CONFIG == OCTA_QUAD_FRAME)
        MAV_TYPE_OCTOROTOR,
#elif (FRAME_CONFIG == HELI_FRAME)
        MAV_TYPE_HELICOPTER,
#elif (FRAME_CONFIG == SINGLE_FRAME)  //because mavlink did not define a singlecopter, we use a rocket
        MAV_TYPE_ROCKET,
#else
  #error Unrecognised frame type
#endif
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        custom_mode,
        system_status);
}

static NOINLINE void plane_send_heartbeat(mavlink_channel_t chan)
{
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status = is_flying() ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;
    uint32_t custom_mode = plane_control_mode;
    
    if (plane_failsafe.state != FAILSAFE_NONE) {
        system_status = MAV_STATE_CRITICAL;
    }

    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    switch (plane_control_mode) {
    case PLANE_MANUAL:
    case PLANE_TRAINING:
    case PLANE_ACRO:
        base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        break;
    case PLANE_STABILIZE:
    case PLANE_FLY_BY_WIRE_A:
    case PLANE_FLY_BY_WIRE_B:
    case PLANE_CRUISE:
        base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
        break;
    case PLANE_AUTO:
    case PLANE_RTL:
    case PLANE_LOITER:
    case PLANE_GUIDED:
    case PLANE_CIRCLE:
        base_mode = MAV_MODE_FLAG_GUIDED_ENABLED |
                    MAV_MODE_FLAG_STABILIZE_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    case PLANE_INITIALISING:
        system_status = MAV_STATE_CALIBRATING;
        break;
    }

    if (!training_manual_pitch || !training_manual_roll) {
        base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;        
    }

    if (plane_control_mode != PLANE_MANUAL && plane_control_mode != PLANE_INITIALISING) {
        // stabiliser of some form is enabled
        base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }

    if (g.stick_mixing != STICK_MIXING_DISABLED && plane_control_mode != PLANE_INITIALISING) {
        // all modes except PLANE_INITIALISING have some form of manual
        // override if stick mixing is enabled
        base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }

#if HIL_MODE != HIL_MODE_DISABLED
    base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
#endif

    // we are armed if we are not initialising
    if (plane_control_mode != PLANE_INITIALISING && arming.is_armed()) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    mavlink_msg_heartbeat_send(
        chan,
        MAV_TYPE_FIXED_WING,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        custom_mode,
        system_status);
}

static NOINLINE void send_attitude(mavlink_channel_t chan)
{
    mavlink_msg_attitude_send(
        chan,
        millis(),
        ahrs.roll,
        ahrs.pitch,
        ahrs.yaw,
        omega.x,
        omega.y,
        omega.z);
}

static NOINLINE void plane_send_attitude(mavlink_channel_t chan)
{
    omega = ahrs.get_gyro();
    mavlink_msg_attitude_send(
        chan,
        millis(),
        ahrs.roll,
        ahrs.pitch - radians(g.pitch_trim_cd*0.01),
        ahrs.yaw,
        omega.x,
        omega.y,
        omega.z);
}

#if AC_FENCE == ENABLED
static NOINLINE void send_limits_status(mavlink_channel_t chan)
{
    fence_send_mavlink_status(chan);
}
#endif

#if AC_FENCE == ENABLED
static NOINLINE void send_fence_status(mavlink_channel_t chan)
{
    geofence_send_status(chan);
}
#endif


static NOINLINE void send_extended_status1(mavlink_channel_t chan, uint16_t packet_drops)
{
    uint32_t control_sensors_present;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;

    // default sensors present
    control_sensors_present = MAVLINK_SENSOR_PRESENT_DEFAULT;

    // first what sensors/controllers we have
    if (g.compass_enabled) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG; // compass present
    }
    if (g_gps != NULL && g_gps->status() > GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
    }
#if OPTFLOW == ENABLED
    if (g.optflow_enabled) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif
    if (ap.rc_receiver_present) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }

    // all present sensors enabled by default except altitude and position control which we will set individually
    control_sensors_enabled = control_sensors_present & (~MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL & ~MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL);

    switch (control_mode) {
    case ALT_HOLD:
    case AUTO:
    case GUIDED:
    case LOITER:
    case RTL:
    case CIRCLE:
    case LAND:
    case OF_LOITER:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        break;
    case POSITION:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        break;
    case SPORT:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        break;
    }

    // default to all healthy except compass, gps and receiver which we set individually
    control_sensors_health = control_sensors_present & (~MAV_SYS_STATUS_SENSOR_3D_MAG & ~MAV_SYS_STATUS_SENSOR_GPS & ~MAV_SYS_STATUS_SENSOR_RC_RECEIVER);
    if (g.compass_enabled && compass.healthy() && ahrs.use_compass()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (g_gps != NULL && g_gps->status() > GPS::NO_GPS && (!gps_glitch.glitching()||ap.usb_connected)) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
    }
    if (ap.rc_receiver_present && !failsafe.radio) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }
    if (!ins.healthy()) {
        control_sensors_health &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
    }

    int16_t battery_current = -1;
    int8_t battery_remaining = -1;

    if (battery.monitoring() == AP_BATT_MONITOR_VOLTAGE_AND_CURRENT) {
        battery_remaining = battery.capacity_remaining_pct();
        battery_current = battery.current_amps() * 100;
    }

    mavlink_msg_sys_status_send(
        chan,
        control_sensors_present,
        control_sensors_enabled,
        control_sensors_health,
        (uint16_t)(scheduler.load_average(10000) * 1000),
        battery.voltage() * 1000, // mV
        battery_current,        // in 10mA units
        battery_remaining,      // in %
        0, // comm drops %,
        0, // comm drops in pkts,
        0, 0, 0, 0);

}

static NOINLINE void plane_send_extended_status1(mavlink_channel_t chan)
{
    uint32_t control_sensors_present;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;

    // default sensors present
    control_sensors_present = MAVLINK_SENSOR_PRESENT_DEFAULT;

    // first what sensors/controllers we have
    if (g.compass_enabled) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG; // compass present
    }

    if (airspeed.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
    }
    if (g_gps != NULL && g_gps->status() > GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
    }

    // all present sensors enabled by default except rate control, attitude stabilization, yaw, altitude, position control and motor output which we will set individually
    control_sensors_enabled = control_sensors_present & (~MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL & ~MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION & ~MAV_SYS_STATUS_SENSOR_YAW_POSITION & ~MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL & ~MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL & ~MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS);

    if (airspeed.enabled() && airspeed.use()) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
    }

    switch (plane_control_mode) {
    case PLANE_MANUAL:
        break;

    case PLANE_ACRO:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        break;

    case PLANE_STABILIZE:
    case PLANE_FLY_BY_WIRE_A:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation
        break;

    case PLANE_FLY_BY_WIRE_B:
    case PLANE_CRUISE:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS; // motor control
        break;

    case PLANE_TRAINING:
        if (!training_manual_roll || !training_manual_pitch) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation        
        }
        break;

    case PLANE_AUTO:
    case PLANE_RTL:
    case PLANE_LOITER:
    case PLANE_GUIDED:
    case PLANE_CIRCLE:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL; // 3D angular rate control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION; // attitude stabilisation
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_YAW_POSITION; // yaw position
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL; // altitude control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL; // X/Y position control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS; // motor control
        break;

    case PLANE_INITIALISING:
        break;
    }

    // default to all healthy
    control_sensors_health = control_sensors_present & ~(MAV_SYS_STATUS_SENSOR_3D_MAG | 
                                                         MAV_SYS_STATUS_SENSOR_GPS |
                                                         MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
    if (g.compass_enabled && compass.healthy() && ahrs.use_compass()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (g_gps != NULL && g_gps->status() >= GPS::GPS_OK_FIX_3D) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
    }
    if (!ins.healthy()) {
        control_sensors_health &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
    }
    if (airspeed.healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
    }

    int16_t battery_current = -1;
    int8_t battery_remaining = -1;

    if (battery.monitoring() == AP_BATT_MONITOR_VOLTAGE_AND_CURRENT) {
        battery_remaining = battery.capacity_remaining_pct();
        battery_current = battery.current_amps() * 100;
    }

    mavlink_msg_sys_status_send(
        chan,
        control_sensors_present,
        control_sensors_enabled,
        control_sensors_health,
        (uint16_t)(scheduler.load_average(20000) * 1000),
        battery.voltage() * 1000, // mV
        battery_current,        // in 10mA units
        battery_remaining,      // in %
        0, // comm drops %,
        0, // comm drops in pkts,
        0, 0, 0, 0);

}

static void NOINLINE send_location(mavlink_channel_t chan)
{
    uint32_t fix_time;
    // if we have a GPS fix, take the time as the last fix time. That
    // allows us to correctly calculate velocities and extrapolate
    // positions.
    // If we don't have a GPS fix then we are dead reckoning, and will
    // use the current boot time as the fix time.    
    if (g_gps->status() >= GPS::GPS_OK_FIX_2D) {
        fix_time = g_gps->last_fix_time;
    } else {
        fix_time = millis();
    }
    mavlink_msg_global_position_int_send(
        chan,
        fix_time,
        current_loc.lat,                // in 1E7 degrees
        current_loc.lng,                // in 1E7 degrees
        g_gps->altitude_cm * 10,             // millimeters above sea level
        (current_loc.alt - home.alt) * 10,           // millimeters above ground
        g_gps->velocity_north() * 100,  // X speed cm/s (+ve North)
        g_gps->velocity_east()  * 100,  // Y speed cm/s (+ve East)
        g_gps->velocity_down()  * -100, // Z speed cm/s (+ve up)
        ahrs.yaw_sensor);               // compass heading in 1/100 degree
}

static void NOINLINE plane_send_location(mavlink_channel_t chan)
{
    uint32_t fix_time;
    // if we have a GPS fix, take the time as the last fix time. That
    // allows us to correctly calculate velocities and extrapolate
    // positions.
    // If we don't have a GPS fix then we are dead reckoning, and will
    // use the current boot time as the fix time.    
    if (g_gps->status() >= GPS::GPS_OK_FIX_2D) {
        fix_time = g_gps->last_fix_time;
    } else {
        fix_time = millis();
    }
    mavlink_msg_global_position_int_send(
        chan,
        fix_time,
        current_loc.lat,                // in 1E7 degrees
        current_loc.lng,                // in 1E7 degrees
        g_gps->altitude_cm * 10,        // millimeters above sea level
        relative_altitude() * 1.0e3,    // millimeters above ground
        g_gps->velocity_north() * 100,  // X speed cm/s (+ve North)
        g_gps->velocity_east()  * 100,  // Y speed cm/s (+ve East)
        g_gps->velocity_down()  * -100, // Z speed cm/s (+ve up)
        ahrs.yaw_sensor);
}

static void NOINLINE send_nav_controller_output(mavlink_channel_t chan)
{
    mavlink_msg_nav_controller_output_send(
        chan,
        control_roll / 1.0e2f,
        control_pitch / 1.0e2f,
        control_yaw / 1.0e2f,
        wp_bearing / 1.0e2f,
        wp_distance / 1.0e2f,
        altitude_error / 1.0e2f,
        0,
        0);
}

static void NOINLINE plane_send_nav_controller_output(mavlink_channel_t chan)
{
    mavlink_msg_nav_controller_output_send(
        chan,
        nav_roll_cd * 0.01,
        nav_pitch_cd * 0.01,
        nav_controller->nav_bearing_cd() * 0.01f,
        nav_controller->target_bearing_cd() * 0.01f,
        plane_wp_distance,
        altitude_error_cm * 0.01,
        airspeed_error_cm,
        nav_controller->crosstrack_error());
}

static void NOINLINE send_ahrs(mavlink_channel_t chan)
{
    const Vector3f &omega_I = ahrs.get_gyro_drift();
    mavlink_msg_ahrs_send(
        chan,
        omega_I.x,
        omega_I.y,
        omega_I.z,
        1,
        0,
        ahrs.get_error_rp(),
        ahrs.get_error_yaw());
}

static void NOINLINE plane_send_ahrs(mavlink_channel_t chan)
{
    const Vector3f &omega_I = ahrs.get_gyro_drift();
    mavlink_msg_ahrs_send(
        chan,
        omega_I.x,
        omega_I.y,
        omega_I.z,
        0,
        0,
        ahrs.get_error_rp(),
        ahrs.get_error_yaw());
}

#if HIL_MODE != HIL_MODE_DISABLED
/*
  keep last HIL_STATE message to allow sending SIM_STATE
 */
static mavlink_hil_state_t last_hil_state;
#endif

// report simulator state
static void NOINLINE send_simstate(mavlink_channel_t chan)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    sitl.simstate_send(chan);
#endif
}

static void NOINLINE plane_send_simstate(mavlink_channel_t chan)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    sitl.simstate_send(chan);
#elif HIL_MODE != HIL_MODE_DISABLED
    mavlink_msg_simstate_send(chan,
                              last_hil_state.roll,
                              last_hil_state.pitch,
                              last_hil_state.yaw,
                              last_hil_state.xacc*0.001*GRAVITY_MSS,
                              last_hil_state.yacc*0.001*GRAVITY_MSS,
                              last_hil_state.zacc*0.001*GRAVITY_MSS,
                              last_hil_state.rollspeed,
                              last_hil_state.pitchspeed,
                              last_hil_state.yawspeed,
                              last_hil_state.lat,
                              last_hil_state.lon);
#endif
}

static void NOINLINE send_hwstatus(mavlink_channel_t chan)
{
    mavlink_msg_hwstatus_send(
        chan,
        board_voltage(),
        hal.i2c->lockup_count());
}

static void NOINLINE send_gps_raw(mavlink_channel_t chan)
{
    mavlink_msg_gps_raw_int_send(
        chan,
        g_gps->last_fix_time*(uint64_t)1000,
        g_gps->status(),
        g_gps->latitude,      // in 1E7 degrees
        g_gps->longitude,     // in 1E7 degrees
        g_gps->altitude_cm * 10, // in mm
        g_gps->hdop,
        65535,
        g_gps->ground_speed_cm,  // cm/s
        g_gps->ground_course_cd, // 1/100 degrees,
        g_gps->num_sats);

}

static void NOINLINE plane_send_gps_raw(mavlink_channel_t chan)
{
    static uint32_t last_send_time;
    if (last_send_time != 0 && last_send_time == g_gps->last_fix_time && g_gps->status() >= GPS::GPS_OK_FIX_3D) {
        return;
    }
    last_send_time = g_gps->last_fix_time;
    mavlink_msg_gps_raw_int_send(
        chan,
        g_gps->last_fix_time*(uint64_t)1000,
        g_gps->status(),
        g_gps->latitude,      // in 1E7 degrees
        g_gps->longitude,     // in 1E7 degrees
        g_gps->altitude_cm * 10, // in mm
        g_gps->hdop,
        65535,
        g_gps->ground_speed_cm,  // cm/s
        g_gps->ground_course_cd, // 1/100 degrees,
        g_gps->num_sats);
}

static void NOINLINE send_system_time(mavlink_channel_t chan)
{
    mavlink_msg_system_time_send(
        chan,
        g_gps->time_epoch_usec(),
        hal.scheduler->millis());
}

#if HIL_MODE != HIL_MODE_DISABLED
static void NOINLINE send_servo_out(mavlink_channel_t chan)
{
    // normalized values scaled to -10000 to 10000
    // This is used for HIL.  Do not change without discussing with HIL maintainers

#if FRAME_CONFIG == HELI_FRAME

    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0, // port 0
        g.rc_1.servo_out,
        g.rc_2.servo_out,
        g.rc_3.radio_out,
        g.rc_4.servo_out,
        0,
        0,
        0,
        0,
        receiver_rssi);
#else
 #if X_PLANE == ENABLED
    /* update by JLN for X-Plane HIL */
    if(motors.armed() && ap.auto_armed) {
        mavlink_msg_rc_channels_scaled_send(
            chan,
            millis(),
            0,         // port 0
            g.rc_1.servo_out,
            g.rc_2.servo_out,
            10000 * g.rc_3.norm_output(),
            g.rc_4.servo_out,
            10000 * g.rc_1.norm_output(),
            10000 * g.rc_2.norm_output(),
            10000 * g.rc_3.norm_output(),
            10000 * g.rc_4.norm_output(),
            receiver_rssi);
    }else{
        mavlink_msg_rc_channels_scaled_send(
            chan,
            millis(),
            0,         // port 0
            0,
            0,
            -10000,
            0,
            10000 * g.rc_1.norm_output(),
            10000 * g.rc_2.norm_output(),
            10000 * g.rc_3.norm_output(),
            10000 * g.rc_4.norm_output(),
            receiver_rssi);
    }

 #else
    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0,         // port 0
        g.rc_1.servo_out,
        g.rc_2.servo_out,
        g.rc_3.radio_out,
        g.rc_4.servo_out,
        10000 * g.rc_1.norm_output(),
        10000 * g.rc_2.norm_output(),
        10000 * g.rc_3.norm_output(),
        10000 * g.rc_4.norm_output(),
        receiver_rssi);
 #endif
#endif
}

static void NOINLINE plane_send_servo_out(mavlink_channel_t chan)
{
    // normalized values scaled to -10000 to 10000
    // This is used for HIL.  Do not change without discussing with
    // HIL maintainers
    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0, // port 0
        10000 * channel_roll->norm_output(),
        10000 * channel_pitch->norm_output(),
        10000 * channel_throttle->norm_output(),
        10000 * channel_rudder->norm_output(),
        0,
        0,
        0,
        0,
        receiver_rssi);
}
#endif

static void NOINLINE send_radio_in(mavlink_channel_t chan)
{
    mavlink_msg_rc_channels_raw_send(
        chan,
        millis(),
        0, // port
        g.rc_1.radio_in,
        g.rc_2.radio_in,
        g.rc_3.radio_in,
        g.rc_4.radio_in,
        g.rc_5.radio_in,
        g.rc_6.radio_in,
        g.rc_7.radio_in,
        g.rc_8.radio_in,
        receiver_rssi);
}

static void NOINLINE plane_send_radio_in(mavlink_channel_t chan)
{
    mavlink_msg_rc_channels_raw_send(
        chan,
        millis(),
        0, // port
        hal.rcin->read(CH_1),
        hal.rcin->read(CH_2),
        hal.rcin->read(CH_3),
        hal.rcin->read(CH_4),
        hal.rcin->read(CH_5),
        hal.rcin->read(CH_6),
        hal.rcin->read(CH_7),
        hal.rcin->read(CH_8),
        receiver_rssi);
}

static void NOINLINE send_radio_out(mavlink_channel_t chan)
{
    uint8_t i;
    uint16_t rcout[8];
    hal.rcout->read(rcout,8);
    // clear out unreasonable values
    for (i=0; i<8; i++) {
        if (rcout[i] > 10000) {
            rcout[i] = 0;
        }
    }
    mavlink_msg_servo_output_raw_send(
        chan,
        micros(),
        0, // port
        rcout[0],
        rcout[1],
        rcout[2],
        rcout[3],
        rcout[4],
        rcout[5],
        rcout[6],
        rcout[7]);
}

static void NOINLINE plane_send_radio_out(mavlink_channel_t chan)
{
#if HIL_MODE != HIL_MODE_DISABLED
    if (!g.hil_servos) {
        mavlink_msg_servo_output_raw_send(
            chan,
            micros(),
            0,     // port
            RC_Channel::rc_channel(0)->radio_out,
            RC_Channel::rc_channel(1)->radio_out,
            RC_Channel::rc_channel(2)->radio_out,
            RC_Channel::rc_channel(3)->radio_out,
            RC_Channel::rc_channel(4)->radio_out,
            RC_Channel::rc_channel(5)->radio_out,
            RC_Channel::rc_channel(6)->radio_out,
            RC_Channel::rc_channel(7)->radio_out);
        return;
    }
#endif
    mavlink_msg_servo_output_raw_send(
        chan,
        micros(),
        0,     // port
        hal.rcout->read(0),
        hal.rcout->read(1),
        hal.rcout->read(2),
        hal.rcout->read(3),
        hal.rcout->read(4),
        hal.rcout->read(5),
        hal.rcout->read(6),
        hal.rcout->read(7));
}

static void NOINLINE send_vfr_hud(mavlink_channel_t chan)
{
    mavlink_msg_vfr_hud_send(
        chan,
        (float)g_gps->ground_speed_cm / 100.0f,
        (float)g_gps->ground_speed_cm / 100.0f,
        (ahrs.yaw_sensor / 100) % 360,
        g.rc_3.servo_out/10,
        current_loc.alt / 100.0f,
        climb_rate / 100.0f);
}

static void NOINLINE plane_send_vfr_hud(mavlink_channel_t chan)
{
    float aspeed;
    if (airspeed.enabled()) {
        aspeed = airspeed.get_airspeed();
    } else if (!ahrs.airspeed_estimate(&aspeed)) {
        aspeed = 0;
    }
    float throttle_norm = channel_throttle->norm_output() * 100;
    throttle_norm = constrain_int16(throttle_norm, -100, 100);
    uint16_t throttle = ((uint16_t)(throttle_norm + 100)) / 2;
    mavlink_msg_vfr_hud_send(
        chan,
        aspeed,
        (float)g_gps->ground_speed_cm * 0.01f,
        (ahrs.yaw_sensor / 100) % 360,
        throttle,
        current_loc.alt / 100.0,
        barometer.get_climb_rate());
}

static void NOINLINE send_raw_imu1(mavlink_channel_t chan)
{
    const Vector3f &accel = ins.get_accel();
    const Vector3f &gyro = ins.get_gyro();
    const Vector3f &mag = compass.get_field();
    mavlink_msg_raw_imu_send(
        chan,
        micros(),
        accel.x * 1000.0f / GRAVITY_MSS,
        accel.y * 1000.0f / GRAVITY_MSS,
        accel.z * 1000.0f / GRAVITY_MSS,
        gyro.x * 1000.0f,
        gyro.y * 1000.0f,
        gyro.z * 1000.0f,
        mag.x,
        mag.y,
        mag.z);
    if (ins.get_gyro_count() <= 1 &&
        ins.get_accel_count() <= 1 &&
        compass.get_count() <= 1) {
        return;
    }
    const Vector3f &accel2 = ins.get_accel(1);
    const Vector3f &gyro2 = ins.get_gyro(1);
    const Vector3f &mag2 = compass.get_field(1);
    mavlink_msg_scaled_imu2_send(
        chan,
        millis(),
        accel2.x * 1000.0f / GRAVITY_MSS,
        accel2.y * 1000.0f / GRAVITY_MSS,
        accel2.z * 1000.0f / GRAVITY_MSS,
        gyro2.x * 1000.0f,
        gyro2.y * 1000.0f,
        gyro2.z * 1000.0f,
        mag2.x,
        mag2.y,
        mag2.z);        
}

static void NOINLINE send_raw_imu2(mavlink_channel_t chan)
{
    mavlink_msg_scaled_pressure_send(
        chan,
        millis(),
        barometer.get_pressure()*0.01f, // hectopascal
        (barometer.get_pressure() - barometer.get_ground_pressure())*0.01f, // hectopascal
        (int16_t)(barometer.get_temperature()*100)); // 0.01 degrees C
}

static void NOINLINE send_raw_imu3(mavlink_channel_t chan)
{
    const Vector3f &mag_offsets = compass.get_offsets();
    const Vector3f &accel_offsets = ins.get_accel_offsets();
    const Vector3f &gyro_offsets = ins.get_gyro_offsets();

    mavlink_msg_sensor_offsets_send(chan,
                                    mag_offsets.x,
                                    mag_offsets.y,
                                    mag_offsets.z,
                                    compass.get_declination(),
                                    barometer.get_pressure(),
                                    barometer.get_temperature()*100,
                                    gyro_offsets.x,
                                    gyro_offsets.y,
                                    gyro_offsets.z,
                                    accel_offsets.x,
                                    accel_offsets.y,
                                    accel_offsets.z);
}

static void NOINLINE plane_send_raw_imu1(mavlink_channel_t chan)
{
    const Vector3f &accel = ins.get_accel();
    const Vector3f &gyro = ins.get_gyro();
    const Vector3f &mag = compass.get_field();

    mavlink_msg_raw_imu_send(
        chan,
        micros(),
        accel.x * 1000.0 / GRAVITY_MSS,
        accel.y * 1000.0 / GRAVITY_MSS,
        accel.z * 1000.0 / GRAVITY_MSS,
        gyro.x * 1000.0,
        gyro.y * 1000.0,
        gyro.z * 1000.0,
        mag.x,
        mag.y,
        mag.z);

    if (ins.get_gyro_count() <= 1 &&
        ins.get_accel_count() <= 1 &&
        compass.get_count() <= 1) {
        return;
    }
    const Vector3f &accel2 = ins.get_accel(1);
    const Vector3f &gyro2 = ins.get_gyro(1);
    const Vector3f &mag2 = compass.get_field(1);
    mavlink_msg_scaled_imu2_send(
        chan,
        millis(),
        accel2.x * 1000.0f / GRAVITY_MSS,
        accel2.y * 1000.0f / GRAVITY_MSS,
        accel2.z * 1000.0f / GRAVITY_MSS,
        gyro2.x * 1000.0f,
        gyro2.y * 1000.0f,
        gyro2.z * 1000.0f,
        mag2.x,
        mag2.y,
        mag2.z);        
}

static void NOINLINE plane_send_raw_imu2(mavlink_channel_t chan)
{
    float pressure = barometer.get_pressure();
    mavlink_msg_scaled_pressure_send(
        chan,
        millis(),
        pressure*0.01f, // hectopascal
        (pressure - barometer.get_ground_pressure())*0.01f, // hectopascal
        barometer.get_temperature()*100); // 0.01 degrees C
}

static void NOINLINE plane_send_raw_imu3(mavlink_channel_t chan)
{
    // run this message at a much lower rate - otherwise it
    // pointlessly wastes quite a lot of bandwidth
    static uint8_t counter;
    if (counter++ < 10) {
        return;
    }
    counter = 0;

    Vector3f mag_offsets = compass.get_offsets();
    Vector3f accel_offsets = ins.get_accel_offsets();
    Vector3f gyro_offsets = ins.get_gyro_offsets();

    mavlink_msg_sensor_offsets_send(chan,
                                    mag_offsets.x,
                                    mag_offsets.y,
                                    mag_offsets.z,
                                    compass.get_declination(),
                                    barometer.get_pressure(),
                                    barometer.get_temperature()*100,
                                    gyro_offsets.x,
                                    gyro_offsets.y,
                                    gyro_offsets.z,
                                    accel_offsets.x,
                                    accel_offsets.y,
                                    accel_offsets.z);
}

static void NOINLINE send_wind(mavlink_channel_t chan)
{
    Vector3f wind = ahrs.wind_estimate();
    mavlink_msg_wind_send(
        chan,
        degrees(atan2f(-wind.y, -wind.x)), // use negative, to give
                                          // direction wind is coming from
        wind.length(),
        wind.z);
}

static void NOINLINE send_current_waypoint(mavlink_channel_t chan)
{
    mavlink_msg_mission_current_send(
        chan,
        (uint16_t)g.command_index);
}

static void NOINLINE send_statustext(mavlink_channel_t chan)
{
    mavlink_statustext_t *s = &gcs[chan-MAVLINK_COMM_0].pending_status;
    mavlink_msg_statustext_send(
        chan,
        s->severity,
        s->text);
}

// are we still delaying telemetry to try to avoid Xbee bricking?
static bool telemetry_delayed(mavlink_channel_t chan)
{
    uint32_t tnow = millis() >> 10;
    if (tnow > (uint32_t)g.telem_delay) {
        return false;
    }
    if (chan == MAVLINK_COMM_0 && hal.gpio->usb_connected()) {
        // this is USB telemetry, so won't be an Xbee
        return false;
    }
    // we're either on the 2nd UART, or no USB cable is connected
    // we need to delay telemetry by the TELEM_DELAY time
    return true;
}


// try to send a message, return false if it won't fit in the serial tx buffer
static bool mavlink_try_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops)
{
    int16_t payload_space = comm_get_txspace(chan) - MAVLINK_NUM_NON_PAYLOAD_BYTES;

    if (telemetry_delayed(chan)) {
        return false;
    }

#if HIL_MODE != HIL_MODE_SENSORS
    // if we don't have at least 250 micros remaining before the main loop
    // wants to fire then don't send a mavlink message. We want to
    // prioritise the main flight control loop over communications
    if (scheduler.time_available_usec() < 250 && motors.armed()) {
        gcs_out_of_time = true;
        return false;
    }
#endif

    switch(id) {
    case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        gcs[chan-MAVLINK_COMM_0].last_heartbeat_time = hal.scheduler->millis();
        send_heartbeat(chan);
        break;

    case MSG_EXTENDED_STATUS1:
        CHECK_PAYLOAD_SIZE(SYS_STATUS);
        send_extended_status1(chan, packet_drops);
        break;

    case MSG_EXTENDED_STATUS2:
        CHECK_PAYLOAD_SIZE(MEMINFO);
        gcs[chan-MAVLINK_COMM_0].send_meminfo();
        break;

    case MSG_ATTITUDE:
        CHECK_PAYLOAD_SIZE(ATTITUDE);
        send_attitude(chan);
        break;

    case MSG_LOCATION:
        CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
        send_location(chan);
        break;

    case MSG_NAV_CONTROLLER_OUTPUT:
        CHECK_PAYLOAD_SIZE(NAV_CONTROLLER_OUTPUT);
        send_nav_controller_output(chan);
        break;

    case MSG_GPS_RAW:
        CHECK_PAYLOAD_SIZE(GPS_RAW_INT);
        send_gps_raw(chan);
        break;

    case MSG_SYSTEM_TIME:
        CHECK_PAYLOAD_SIZE(SYSTEM_TIME);
        send_system_time(chan);
        break;

    case MSG_SERVO_OUT:
#if HIL_MODE != HIL_MODE_DISABLED
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_SCALED);
        send_servo_out(chan);
#endif
        break;

    case MSG_RADIO_IN:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_RAW);
        send_radio_in(chan);
        break;

    case MSG_RADIO_OUT:
        CHECK_PAYLOAD_SIZE(SERVO_OUTPUT_RAW);
        send_radio_out(chan);
        break;

    case MSG_VFR_HUD:
        CHECK_PAYLOAD_SIZE(VFR_HUD);
        send_vfr_hud(chan);
        break;

    case MSG_RAW_IMU1:
        CHECK_PAYLOAD_SIZE(RAW_IMU);
        send_raw_imu1(chan);
        break;

    case MSG_RAW_IMU2:
        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE);
        send_raw_imu2(chan);
        break;

    case MSG_RAW_IMU3:
        CHECK_PAYLOAD_SIZE(SENSOR_OFFSETS);
        send_raw_imu3(chan);
        break;

    case MSG_CURRENT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_CURRENT);
        send_current_waypoint(chan);
        break;

    case MSG_NEXT_PARAM:
        CHECK_PAYLOAD_SIZE(PARAM_VALUE);
        gcs[chan-MAVLINK_COMM_0].queued_param_send();
        break;

    case MSG_NEXT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_REQUEST);
        gcs[chan-MAVLINK_COMM_0].queued_waypoint_send();
        break;

    case MSG_STATUSTEXT:
        CHECK_PAYLOAD_SIZE(STATUSTEXT);
        send_statustext(chan);
        break;

#if AC_FENCE == ENABLED
    case MSG_LIMITS_STATUS:
        CHECK_PAYLOAD_SIZE(LIMITS_STATUS);
        send_limits_status(chan);
        break;
#endif

    case MSG_AHRS:
        CHECK_PAYLOAD_SIZE(AHRS);
        send_ahrs(chan);
        break;

    case MSG_SIMSTATE:
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
        CHECK_PAYLOAD_SIZE(SIMSTATE);
        send_simstate(chan);
#endif
        break;

    case MSG_HWSTATUS:
        CHECK_PAYLOAD_SIZE(HWSTATUS);
        send_hwstatus(chan);
        break;

    case MSG_FENCE_STATUS:
    case MSG_WIND:
    case MSG_RANGEFINDER:
        // unused
        break;

    case MSG_RETRY_DEFERRED:
        break; // just here to prevent a warning
    }

    return true;
}

static bool plane_mavlink_try_send_message(mavlink_channel_t chan, enum ap_message id)
{
    int16_t payload_space = comm_get_txspace(chan) - MAVLINK_NUM_NON_PAYLOAD_BYTES;

    if (telemetry_delayed(chan)) {
        return false;
    }

    // if we don't have at least 1ms remaining before the main loop
    // wants to fire then don't send a mavlink message. We want to
    // prioritise the main flight control loop over communications
    if (!in_mavlink_delay && scheduler.time_available_usec() < 1200) {
        gcs_out_of_time = true;
        return false;
    }

    switch (id) {
    case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        gcs[chan-MAVLINK_COMM_0].last_heartbeat_time = hal.scheduler->millis();
        plane_send_heartbeat(chan);
        return true;

    case MSG_EXTENDED_STATUS1:
        CHECK_PAYLOAD_SIZE(SYS_STATUS);
        plane_send_extended_status1(chan);
        break;

    case MSG_EXTENDED_STATUS2:
        CHECK_PAYLOAD_SIZE(MEMINFO);
        gcs[chan-MAVLINK_COMM_0].send_meminfo();
        break;

    case MSG_ATTITUDE:
        CHECK_PAYLOAD_SIZE(ATTITUDE);
        plane_send_attitude(chan);
        break;

    case MSG_LOCATION:
        CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
        plane_send_location(chan);
        break;

    case MSG_NAV_CONTROLLER_OUTPUT:
        if (plane_control_mode != PLANE_MANUAL) {
            CHECK_PAYLOAD_SIZE(NAV_CONTROLLER_OUTPUT);
            plane_send_nav_controller_output(chan);
        }
        break;

    case MSG_GPS_RAW:
        CHECK_PAYLOAD_SIZE(GPS_RAW_INT);
        plane_send_gps_raw(chan);
        break;

    case MSG_SYSTEM_TIME:
        CHECK_PAYLOAD_SIZE(SYSTEM_TIME);
        send_system_time(chan);
        break;

    case MSG_SERVO_OUT:
#if HIL_MODE != HIL_MODE_DISABLED
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_SCALED);
        plane_send_servo_out(chan);
#endif
        break;

    case MSG_RADIO_IN:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_RAW);
        plane_send_radio_in(chan);
        break;

    case MSG_RADIO_OUT:
        CHECK_PAYLOAD_SIZE(SERVO_OUTPUT_RAW);
        plane_send_radio_out(chan);
        break;

    case MSG_VFR_HUD:
        CHECK_PAYLOAD_SIZE(VFR_HUD);
        plane_send_vfr_hud(chan);
        break;

    case MSG_RAW_IMU1:
        CHECK_PAYLOAD_SIZE(RAW_IMU);
        plane_send_raw_imu1(chan);
        break;

    case MSG_RAW_IMU2:
        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE);
        plane_send_raw_imu2(chan);
        break;

    case MSG_RAW_IMU3:
        CHECK_PAYLOAD_SIZE(SENSOR_OFFSETS);
        plane_send_raw_imu3(chan);
        break;

    case MSG_CURRENT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_CURRENT);
        send_current_waypoint(chan);
        break;

    case MSG_NEXT_PARAM:
        CHECK_PAYLOAD_SIZE(PARAM_VALUE);
        gcs[chan-MAVLINK_COMM_0].queued_param_send();
        break;

    case MSG_NEXT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_REQUEST);
        gcs[chan-MAVLINK_COMM_0].queued_waypoint_send();
        break;

    case MSG_STATUSTEXT:
        CHECK_PAYLOAD_SIZE(STATUSTEXT);
        send_statustext(chan);
        break;

#if AC_FENCE == ENABLED
    case MSG_FENCE_STATUS:
        CHECK_PAYLOAD_SIZE(FENCE_STATUS);
        send_fence_status(chan);
        break;
#endif

    case MSG_AHRS:
        CHECK_PAYLOAD_SIZE(AHRS);
        plane_send_ahrs(chan);
        break;

    case MSG_SIMSTATE:
        CHECK_PAYLOAD_SIZE(SIMSTATE);
        plane_send_simstate(chan);
        break;

    case MSG_HWSTATUS:
        CHECK_PAYLOAD_SIZE(HWSTATUS);
        send_hwstatus(chan);
        break;

    case MSG_RANGEFINDER:
        CHECK_PAYLOAD_SIZE(RANGEFINDER);
        break;

    case MSG_WIND:
        CHECK_PAYLOAD_SIZE(WIND);
        send_wind(chan);
        break;

    case MSG_RETRY_DEFERRED:
        break; // just here to prevent a warning

    case MSG_LIMITS_STATUS:
        // unused
        break;
    }
    return true;
}

#define MAX_DEFERRED_MESSAGES MSG_RETRY_DEFERRED
static struct mavlink_queue {
    enum ap_message deferred_messages[MAX_DEFERRED_MESSAGES];
    uint8_t next_deferred_message;
    uint8_t num_deferred_messages;
} mavlink_queue[MAVLINK_COMM_NUM_BUFFERS];

// send a message using mavlink
static void mavlink_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops)
{
    uint8_t i, nextid;
    struct mavlink_queue *q = &mavlink_queue[(uint8_t)chan];

    // see if we can send the deferred messages, if any
    while (q->num_deferred_messages != 0) {
        if (!mavlink_try_send_message(chan,
                                      q->deferred_messages[q->next_deferred_message],
                                      packet_drops)) {
            break;
        }
        q->next_deferred_message++;
        if (q->next_deferred_message == MAX_DEFERRED_MESSAGES) {
            q->next_deferred_message = 0;
        }
        q->num_deferred_messages--;
    }

    if (id == MSG_RETRY_DEFERRED) {
        return;
    }

    // this message id might already be deferred
    for (i=0, nextid = q->next_deferred_message; i < q->num_deferred_messages; i++) {
        if (q->deferred_messages[nextid] == id) {
            // its already deferred, discard
            return;
        }
        nextid++;
        if (nextid == MAX_DEFERRED_MESSAGES) {
            nextid = 0;
        }
    }

    if (q->num_deferred_messages != 0 ||
        !mavlink_try_send_message(chan, id, packet_drops)) {
        // can't send it now, so defer it
        if (q->num_deferred_messages == MAX_DEFERRED_MESSAGES) {
            // the defer buffer is full, discard
            return;
        }
        nextid = q->next_deferred_message + q->num_deferred_messages;
        if (nextid >= MAX_DEFERRED_MESSAGES) {
            nextid -= MAX_DEFERRED_MESSAGES;
        }
        q->deferred_messages[nextid] = id;
        q->num_deferred_messages++;
    }
}

static void plane_mavlink_send_message(mavlink_channel_t chan, enum ap_message id)
{
    uint8_t i, nextid;
    struct mavlink_queue *q = &mavlink_queue[(uint8_t)chan];

    // see if we can send the deferred messages, if any
    while (q->num_deferred_messages != 0) {
        if (!plane_mavlink_try_send_message(chan,
                                      q->deferred_messages[q->next_deferred_message])) {
            break;
        }
        q->next_deferred_message++;
        if (q->next_deferred_message == MAX_DEFERRED_MESSAGES) {
            q->next_deferred_message = 0;
        }
        q->num_deferred_messages--;
    }

    if (id == MSG_RETRY_DEFERRED) {
        return;
    }

    // this message id might already be deferred
    for (i=0, nextid = q->next_deferred_message; i < q->num_deferred_messages; i++) {
        if (q->deferred_messages[nextid] == id) {
            // its already deferred, discard
            return;
        }
        nextid++;
        if (nextid == MAX_DEFERRED_MESSAGES) {
            nextid = 0;
        }
    }

    if (q->num_deferred_messages != 0 ||
        !plane_mavlink_try_send_message(chan, id)) {
        // can't send it now, so defer it
        if (q->num_deferred_messages == MAX_DEFERRED_MESSAGES) {
            // the defer buffer is full, discard
            return;
        }
        nextid = q->next_deferred_message + q->num_deferred_messages;
        if (nextid >= MAX_DEFERRED_MESSAGES) {
            nextid -= MAX_DEFERRED_MESSAGES;
        }
        q->deferred_messages[nextid] = id;
        q->num_deferred_messages++;
    }
}

void mavlink_send_text(mavlink_channel_t chan, gcs_severity severity, const char *str)
{
    if (telemetry_delayed(chan)) {
        return;
    }

    if (severity == SEVERITY_LOW) {
        // send via the deferred queuing system
        mavlink_statustext_t *s = &gcs[chan-MAVLINK_COMM_0].pending_status;
        s->severity = (uint8_t)severity;
        strncpy((char *)s->text, str, sizeof(s->text));
        mavlink_send_message(chan, MSG_STATUSTEXT, 0);
    } else {
        // send immediately
        mavlink_msg_statustext_send(chan, severity, str);
    }
}

void plane_mavlink_send_text(mavlink_channel_t chan, gcs_severity severity, const char *str)
{
    if (telemetry_delayed(chan)) {
        return;
    }

    if (severity == SEVERITY_LOW) {
        // send via the deferred queuing system
        mavlink_statustext_t *s = &gcs[chan-MAVLINK_COMM_0].pending_status;
        s->severity = (uint8_t)severity;
        strncpy((char *)s->text, str, sizeof(s->text));
        plane_mavlink_send_message(chan, MSG_STATUSTEXT);
    } else {
        // send immediately
        mavlink_msg_statustext_send(chan, severity, str);
    }
}

/*
  default stream rates to 1Hz
 */
const AP_Param::GroupInfo GCS_MAVLINK::var_info[] PROGMEM = {
    // @Param: RAW_SENS
    // @DisplayName: Raw sensor stream rate
    // @Description: Raw sensor stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK, streamRates[0],  0),

    // @Param: EXT_STAT
    // @DisplayName: Extended status stream rate to ground station
    // @Description: Extended status stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK, streamRates[1],  0),

    // @Param: RC_CHAN
    // @DisplayName: RC Channel stream rate to ground station
    // @Description: RC Channel stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK, streamRates[2],  0),

    // @Param: RAW_CTRL
    // @DisplayName: Raw Control stream rate to ground station
    // @Description: Raw Control stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_CTRL", 3, GCS_MAVLINK, streamRates[3],  0),

    // @Param: POSITION
    // @DisplayName: Position stream rate to ground station
    // @Description: Position stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK, streamRates[4],  0),

    // @Param: EXTRA1
    // @DisplayName: Extra data type 1 stream rate to ground station
    // @Description: Extra data type 1 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK, streamRates[5],  0),

    // @Param: EXTRA2
    // @DisplayName: Extra data type 2 stream rate to ground station
    // @Description: Extra data type 2 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK, streamRates[6],  0),

    // @Param: EXTRA3
    // @DisplayName: Extra data type 3 stream rate to ground station
    // @Description: Extra data type 3 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK, streamRates[7],  0),

    // @Param: PARAMS
    // @DisplayName: Parameter stream rate to ground station
    // @Description: Parameter stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK, streamRates[8],  0),
    AP_GROUPEND
};

void
GCS_MAVLINK::update(void)
{
    // receive new packets
    mavlink_message_t msg;
    mavlink_status_t status;
    status.packet_rx_drop_count = 0;

    // process received bytes
    uint16_t nbytes = comm_get_available(chan);
    for (uint16_t i=0; i<nbytes; i++)
    {
        uint8_t c = comm_receive_ch(chan);

#if CLI_ENABLED == ENABLED
        /* allow CLI to be started by hitting enter 3 times, if no
         *  heartbeat packets have been received */
	if (!isplane){
        if (mavlink_active == 0 && (millis() - _cli_timeout) < 20000 && 
            !motors.armed() && comm_is_idle(chan)) {
            if (c == '\n' || c == '\r') {
                crlf_count++;
            } else {
                crlf_count = 0;
            }
            if (crlf_count == 3) {
                run_cli(_port);
            }
        }
	}
	if (isplane){
        if (mavlink_active == 0 && (millis() - _cli_timeout) < 20000 && 
            comm_is_idle(chan)) {
            if (c == '\n' || c == '\r') {
                crlf_count++;
            } else {
                crlf_count = 0;
            }
            if (crlf_count == 3) {
                run_cli(_port);
            }
        }
	}
#endif

        // Try to get a new message
        if (mavlink_parse_char(chan, c, &msg, &status)) {
            // we exclude radio packets to make it possible to use the
            // CLI over the radio
            if (msg.msgid != MAVLINK_MSG_ID_RADIO && msg.msgid != MAVLINK_MSG_ID_RADIO_STATUS) {
                mavlink_active = true;
            }
            handleMessage(&msg);
        }
    }

if (!isplane){
    // Update packet drops counter
    packet_drops += status.packet_rx_drop_count;
}
	
    if (!waypoint_receiving) {
        return;
    }

    uint32_t tnow = millis();

    if (waypoint_receiving &&
        waypoint_request_i <= waypoint_request_last &&
        tnow > waypoint_timelast_request + 500 + (stream_slowdown*20)) {
        waypoint_timelast_request = tnow;
        send_message(MSG_NEXT_WAYPOINT);
    }

    // stop waypoint receiving if timeout
if (!isplane){
    if (waypoint_receiving && (tnow - waypoint_timelast_receive) > waypoint_receive_timeout) {
        waypoint_receiving = false;
    }
}
if (isplane){
    // stop waypoint receiving if timeout
    if (waypoint_receiving && (millis() - waypoint_timelast_receive) > waypoint_receive_timeout) {
        waypoint_receiving = false;
    }
}
}

// see if we should send a stream now. Called at 50Hz
bool GCS_MAVLINK::stream_trigger(enum streams stream_num)
{
    if (stream_num >= NUM_STREAMS) {
        return false;
    }
    float rate = (uint8_t)streamRates[stream_num].get();

    // send at a much lower rate while handling waypoints and
    // parameter sends
    if ((stream_num != STREAM_PARAMS) && 
        (waypoint_receiving || _queued_parameter != NULL)) {
        rate *= 0.25;
    }

    if (rate <= 0) {
        return false;
    }

    if (stream_ticks[stream_num] == 0) {
        // we're triggering now, setup the next trigger point
        if (rate > 50) {
            rate = 50;
        }
        stream_ticks[stream_num] = (50 / rate) + stream_slowdown;
        return true;
    }

    // count down at 50Hz
    stream_ticks[stream_num]--;
    return false;
}

void
GCS_MAVLINK::data_stream_send(void)
{
if (!isplane){
    if (waypoint_receiving) {
        // don't interfere with mission transfer
        return;
    }
}

if (!isplane){
    if (!in_mavlink_delay && !motors.armed()) {
        handle_log_send(DataFlash);
    }
}
if (isplane){
    if (!in_mavlink_delay) {
        handle_log_send(DataFlash);
    }
}

    gcs_out_of_time = false;

    if (_queued_parameter != NULL) {
        if (streamRates[STREAM_PARAMS].get() <= 0) {
            streamRates[STREAM_PARAMS].set(10);
        }
        if (stream_trigger(STREAM_PARAMS)) {
            send_message(MSG_NEXT_PARAM);
        }
        // don't send anything else at the same time as parameters
        return;
    }

    if (gcs_out_of_time) return;

    if (in_mavlink_delay) {
if (isplane){
#if HIL_MODE != HIL_MODE_DISABLED
        // in HIL we need to keep sending servo values to ensure
        // the simulator doesn't pause, otherwise our sensor
        // calibration could stall
        if (stream_trigger(STREAM_RAW_CONTROLLER)) {
            send_message(MSG_SERVO_OUT);
        }
        if (stream_trigger(STREAM_RC_CHANNELS)) {
            send_message(MSG_RADIO_OUT);
        }
#endif
}
        // don't send any other stream types while in the delay callback
        return;
    }

if (isplane){
    if (gcs_out_of_time) return;
}
    if (stream_trigger(STREAM_RAW_SENSORS)) {
        send_message(MSG_RAW_IMU1);
        send_message(MSG_RAW_IMU2);
        send_message(MSG_RAW_IMU3);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTENDED_STATUS)) {
        send_message(MSG_EXTENDED_STATUS1);
        send_message(MSG_EXTENDED_STATUS2);
        send_message(MSG_CURRENT_WAYPOINT);
        send_message(MSG_GPS_RAW);
        send_message(MSG_NAV_CONTROLLER_OUTPUT);
		if (!isplane){
        send_message(MSG_LIMITS_STATUS);
		}
		if (isplane){
        send_message(MSG_FENCE_STATUS);
		}
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_POSITION)) {
        send_message(MSG_LOCATION);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_RAW_CONTROLLER)) {
        send_message(MSG_SERVO_OUT);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_RC_CHANNELS)) {
        send_message(MSG_RADIO_OUT);
        send_message(MSG_RADIO_IN);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTRA1)) {
        send_message(MSG_ATTITUDE);
        send_message(MSG_SIMSTATE);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTRA2)) {
        send_message(MSG_VFR_HUD);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTRA3)) {
        send_message(MSG_AHRS);
        send_message(MSG_HWSTATUS);
		if (isplane){
        send_message(MSG_WIND);
        send_message(MSG_RANGEFINDER);
		}
        send_message(MSG_SYSTEM_TIME);
    }
}

void
GCS_MAVLINK::send_message(enum ap_message id)
{
if (!isplane){
    mavlink_send_message(chan,id, packet_drops);
}
if (isplane){
    plane_mavlink_send_message(chan, id);
}
}

void
GCS_MAVLINK::send_text_P(gcs_severity severity, const prog_char_t *str)
{
    mavlink_statustext_t m;
    uint8_t i;
    for (i=0; i<sizeof(m.text); i++) {
        m.text[i] = pgm_read_byte((const prog_char *)(str++));
        if (m.text[i] == '\0') {
            break;
        }
    }
    if (i < sizeof(m.text)) m.text[i] = 0;
if (!isplane){
    mavlink_send_text(chan, severity, (const char *)m.text);
}
 if (isplane){
    plane_mavlink_send_text(chan, severity, (const char *)m.text);
}
}

void GCS_MAVLINK::handleMessage(mavlink_message_t* msg)
{
if (!isplane){
    struct Location tell_command;
    memset(&tell_command, 0, sizeof(tell_command));

    switch (msg->msgid) {

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
    {
        // decode
        mavlink_request_data_stream_t packet;
        mavlink_msg_request_data_stream_decode(msg, &packet);

        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;

        int16_t freq = 0;                 // packet frequency

        if (packet.start_stop == 0)
            freq = 0;                     // stop sending
        else if (packet.start_stop == 1)
            freq = packet.req_message_rate;                     // start sending
        else
            break;

        switch(packet.req_stream_id) {

        case MAV_DATA_STREAM_ALL:
            // note that we don't set STREAM_PARAMS - that is internal only
            for (uint8_t i=0; i<STREAM_PARAMS; i++) {
                streamRates[i].set(freq);
            }
            break;
        case MAV_DATA_STREAM_RAW_SENSORS:
            streamRates[STREAM_RAW_SENSORS].set(freq);
            break;
        case MAV_DATA_STREAM_EXTENDED_STATUS:
            streamRates[STREAM_EXTENDED_STATUS].set(freq);
            break;
        case MAV_DATA_STREAM_RC_CHANNELS:
            streamRates[STREAM_RC_CHANNELS].set(freq);
            break;
        case MAV_DATA_STREAM_RAW_CONTROLLER:
            streamRates[STREAM_RAW_CONTROLLER].set(freq);
            break;

        //case MAV_DATA_STREAM_RAW_SENSOR_FUSION:
        //	streamRateRawSensorFusion.set_and_save(freq);
        //	break;

        case MAV_DATA_STREAM_POSITION:
            streamRates[STREAM_POSITION].set(freq);
            break;
        case MAV_DATA_STREAM_EXTRA1:
            streamRates[STREAM_EXTRA1].set(freq);
            break;
        case MAV_DATA_STREAM_EXTRA2:
            streamRates[STREAM_EXTRA2].set(freq);
            break;
        case MAV_DATA_STREAM_EXTRA3:
            streamRates[STREAM_EXTRA3].set(freq);
            break;
        }
        break;
    }

    case MAVLINK_MSG_ID_COMMAND_LONG:
    {
        // decode
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component)) break;

        uint8_t result = MAV_RESULT_UNSUPPORTED;

        // do command
        send_text_P(SEVERITY_LOW,PSTR("command received: "));

        switch(packet.command) {

        case MAV_CMD_NAV_LOITER_UNLIM:
            set_mode(LOITER);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
            set_mode(RTL);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_NAV_LAND:
            set_mode(LAND);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_MISSION_START:
            set_mode(AUTO);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_PREFLIGHT_CALIBRATION:
            if (packet.param1 == 1 ||
                packet.param2 == 1) {
                ins.init_accel();
                ahrs.set_trim(Vector3f(0,0,0));             // clear out saved trim
            } 
            if (packet.param3 == 1) {
#if HIL_MODE != HIL_MODE_ATTITUDE
                init_barometer(false);                      // fast barometer calibratoin
#endif
            }
            if (packet.param4 == 1) {
                trim_radio();
            }
            if (packet.param5 == 1) {
                float trim_roll, trim_pitch;
                // this blocks
                AP_InertialSensor_UserInteract_MAVLink interact(chan);
                if(ins.calibrate_accel(&interact, trim_roll, trim_pitch)) {
                    // reset ahrs's trim to suggested values from calibration routine
                    ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
                }
            }
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_COMPONENT_ARM_DISARM:
            if (packet.target_component == MAV_COMP_ID_SYSTEM_CONTROL) {
                if (packet.param1 == 1.0f) {
                    // run pre_arm_checks and arm_checks and display failures
                    pre_arm_checks(true);
                    if(ap.pre_arm_check && arm_checks(true)) {
                        init_arm_motors();
                    }
                    result = MAV_RESULT_ACCEPTED;
                } else if (packet.param1 == 0.0f)  {
                    init_disarm_motors();
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_UNSUPPORTED;
                }
            } else {
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_DO_SET_SERVO:
            if (ServoRelayEvents.do_set_servo(packet.param1, packet.param2)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_REPEAT_SERVO:
            if (ServoRelayEvents.do_repeat_servo(packet.param1, packet.param2, packet.param3, packet.param4*1000)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_SET_RELAY:
            if (ServoRelayEvents.do_set_relay(packet.param1, packet.param2)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_REPEAT_RELAY:
            if (ServoRelayEvents.do_repeat_relay(packet.param1, packet.param2, packet.param3*1000)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            if (packet.param1 == 1 || packet.param1 == 3) {
                // when packet.param1 == 3 we reboot to hold in bootloader
                hal.scheduler->reboot(packet.param1 == 3);
                result = MAV_RESULT_ACCEPTED;
            }
            break;


        default:
            result = MAV_RESULT_UNSUPPORTED;
            break;
        }

        mavlink_msg_command_ack_send(
            chan,
            packet.command,
            result);

        break;
    }


    case MAVLINK_MSG_ID_SET_MODE:
    {
        // decode
        mavlink_set_mode_t packet;
        mavlink_msg_set_mode_decode(msg, &packet);

        if (!(packet.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) {
            // we ignore base_mode as there is no sane way to map
            // from that bitmap to a APM flight mode. We rely on
            // custom_mode instead.
            break;
        }
        set_mode(packet.custom_mode);
        break;
    }

    /*case MAVLINK_MSG_ID_SET_NAV_MODE:
     *       {
     *               // decode
     *               mavlink_set_nav_mode_t packet;
     *               mavlink_msg_set_nav_mode_decode(msg, &packet);
     *               // To set some flight modes we must first receive a "set nav mode" message and then a "set mode" message
     *               mav_nav = packet.nav_mode;
     *               break;
     *       }
     */
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:     //43
    {
        // decode
        mavlink_mission_request_list_t packet;
        mavlink_msg_mission_request_list_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;

        // Start sending waypoints
        mavlink_msg_mission_count_send(
            chan,msg->sysid,
            msg->compid,
            g.command_total);                     // includes home

        waypoint_receiving                      = false;
        waypoint_dest_sysid                     = msg->sysid;
        waypoint_dest_compid            = msg->compid;
        break;
    }

    // XXX read a WP from EEPROM and send it to the GCS
    case MAVLINK_MSG_ID_MISSION_REQUEST:     // 40
    {
        // decode
        mavlink_mission_request_t packet;
        mavlink_msg_mission_request_decode(msg, &packet);

        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;

        // send waypoint
        tell_command = get_cmd_with_index(packet.seq);

        // set frame of waypoint
        uint8_t frame;

        if (tell_command.options & MASK_OPTIONS_RELATIVE_ALT) {
            frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;                     // reference frame
        } else {
            frame = MAV_FRAME_GLOBAL;                     // reference frame
        }

        float param1 = 0, param2 = 0, param3 = 0, param4 = 0;

        // time that the mav should loiter in milliseconds
        uint8_t current = 0;                 // 1 (true), 0 (false)

        if (packet.seq == (uint16_t)g.command_index)
            current = 1;

        uint8_t autocontinue = 1;                 // 1 (true), 0 (false)

        float x = 0, y = 0, z = 0;

        if (tell_command.id < MAV_CMD_NAV_LAST) {
            // command needs scaling
            x = tell_command.lat/1.0e7f;                     // local (x), global (latitude)
            y = tell_command.lng/1.0e7f;                     // local (y), global (longitude)
            // ACM is processing alt inside each command. so we save and load raw values. - this is diffrent to APM
            z = tell_command.alt/1.0e2f;                     // local (z), global/relative (altitude)
        }

        // Switch to map APM command fields into MAVLink command fields
        switch (tell_command.id) {

        case MAV_CMD_NAV_LOITER_TURNS:
        case MAV_CMD_CONDITION_CHANGE_ALT:
        case MAV_CMD_DO_SET_HOME:
            param1 = tell_command.p1;
            break;

        case MAV_CMD_NAV_ROI:
        case MAV_CMD_DO_SET_ROI:
            param1 = tell_command.p1;                        // MAV_ROI (aka roi mode) is held in wp's parameter but we actually do nothing with it because we only support pointing at a specific location provided by x,y and z parameters
            x = tell_command.lat/1.0e7f;                     // local (x), global (latitude)
            y = tell_command.lng/1.0e7f;                     // local (y), global (longitude)
            z = tell_command.alt/1.0e2f;                     // local (z), global/relative (altitude)
            break;

        case MAV_CMD_CONDITION_YAW:
            param3 = tell_command.p1;
            param1 = tell_command.alt;
            param2 = tell_command.lat;
            param4 = tell_command.lng;
            break;

        case MAV_CMD_NAV_TAKEOFF:
            param1 = 0;
            break;

        case MAV_CMD_NAV_LOITER_TIME:
            param1 = tell_command.p1;                                   // ACM loiter time is in 1 second increments
            break;

        case MAV_CMD_CONDITION_DELAY:
        case MAV_CMD_CONDITION_DISTANCE:
            param1 = tell_command.lat;
            break;

        case MAV_CMD_DO_JUMP:
            param2 = tell_command.lat;
            param1 = tell_command.p1;
            break;

        case MAV_CMD_DO_REPEAT_SERVO:
            param4 = tell_command.lng*0.001f; // time
            param3 = tell_command.lat;        // repeat
            param2 = tell_command.alt;        // pwm
            param1 = tell_command.p1;         // channel
            break;
            
        case MAV_CMD_DO_REPEAT_RELAY:
            param3 = tell_command.lat*0.001f; // time
            param2 = tell_command.alt;        // count
            param1 = tell_command.p1;         // relay number
            break;
            
        case MAV_CMD_DO_CHANGE_SPEED:
            param3 = tell_command.lat;
            param2 = tell_command.alt;
            param1 = tell_command.p1;
            break;

        case MAV_CMD_NAV_WAYPOINT:
            param1 = tell_command.p1;
            break;

        case MAV_CMD_DO_SET_PARAMETER:
        case MAV_CMD_DO_SET_RELAY:
        case MAV_CMD_DO_SET_SERVO:
            param2 = tell_command.alt;
            param1 = tell_command.p1;
            break;

        case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
            param1 = tell_command.alt;
            break;
        }

        mavlink_msg_mission_item_send(chan,msg->sysid,
                                      msg->compid,
                                      packet.seq,
                                      frame,
                                      tell_command.id,
                                      current,
                                      autocontinue,
                                      param1,
                                      param2,
                                      param3,
                                      param4,
                                      x,
                                      y,
                                      z);
        break;
    }


    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    {
        // decode
        mavlink_param_request_list_t packet;
        mavlink_msg_param_request_list_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // mark the firmware version in the tlog
        send_text_P(SEVERITY_LOW, PSTR(FIRMWARE_STRING));

#if defined(PX4_GIT_VERSION) && defined(NUTTX_GIT_VERSION)
        send_text_P(SEVERITY_LOW, PSTR("PX4: " PX4_GIT_VERSION " NuttX: " NUTTX_GIT_VERSION));
#endif

        // send system ID if we can
        char sysid[40];
        if (hal.util->get_system_id(sysid)) {
            mavlink_send_text(chan, SEVERITY_LOW, sysid);
        }

        // Start sending parameters - next call to ::update will kick the first one out
        _queued_parameter = AP_Param::first(&_queued_parameter_token, &_queued_parameter_type);
        _queued_parameter_index = 0;
        _queued_parameter_count = _count_parameters();
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    {
        // decode
        mavlink_param_request_read_t packet;
        mavlink_msg_param_request_read_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;
        enum ap_var_type p_type;
        AP_Param *vp;
        char param_name[AP_MAX_NAME_SIZE+1];
        if (packet.param_index != -1) {
            AP_Param::ParamToken token;
            vp = AP_Param::find_by_index(packet.param_index, &p_type, &token);
            if (vp == NULL) {
                gcs_send_text_fmt(PSTR("Unknown parameter index %d"), packet.param_index);
                break;
            }
            vp->copy_name_token(token, param_name, AP_MAX_NAME_SIZE, true);
            param_name[AP_MAX_NAME_SIZE] = 0;
        } else {
            strncpy(param_name, packet.param_id, AP_MAX_NAME_SIZE);
            param_name[AP_MAX_NAME_SIZE] = 0;
            vp = AP_Param::find(param_name, &p_type);
            if (vp == NULL) {
                gcs_send_text_fmt(PSTR("Unknown parameter %.16s"), packet.param_id);
                break;
            }
        }

        float value = vp->cast_to_float(p_type);
        mavlink_msg_param_value_send(
            chan,
            param_name,
            value,
            mav_var_type(p_type),
            _count_parameters(),
            packet.param_index);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
    {
        // decode
        mavlink_mission_clear_all_t packet;
        mavlink_msg_mission_clear_all_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component)) break;

        // clear all waypoints
        uint8_t type = 0;                 // ok (0), error(1)
        g.command_total.set_and_save(1);

        // send acknowledgement 3 times to makes sure it is received
        for (int16_t i=0; i<3; i++)
            mavlink_msg_mission_ack_send(chan, msg->sysid, msg->compid, type);

        break;
    }

    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
    {
        // decode
        mavlink_mission_set_current_t packet;
        mavlink_msg_mission_set_current_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // set current command
        change_command(packet.seq);

        mavlink_msg_mission_current_send(chan, g.command_index);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_COUNT:
    {
        // decode
        mavlink_mission_count_t packet;
        mavlink_msg_mission_count_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // start waypoint receiving
        if (packet.count > MAX_WAYPOINTS) {
            packet.count = MAX_WAYPOINTS;
        }
        g.command_total.set_and_save(packet.count);

        waypoint_timelast_receive = millis();
        waypoint_receiving   = true;
        waypoint_request_i   = 0;
        // note that ArduCopter sets waypoint_request_last to
        // command_total-1, whereas plane and rover use
        // command_total. This is because the copter code assumes
        // command_total includes home
        waypoint_request_last= g.command_total - 1;
        waypoint_timelast_request = 0;
        break;
    }

    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
    {
        // decode
        mavlink_mission_write_partial_list_t packet;
        mavlink_msg_mission_write_partial_list_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // start waypoint receiving
        if (packet.start_index > g.command_total ||
            packet.end_index > g.command_total ||
            packet.end_index < packet.start_index) {
            send_text_P(SEVERITY_LOW,PSTR("flight plan update rejected"));
            break;
        }

        waypoint_timelast_receive = millis();
        waypoint_timelast_request = 0;
        waypoint_receiving   = true;
        waypoint_request_i   = packet.start_index;
        waypoint_request_last= packet.end_index;
        break;
    }

#ifdef MAVLINK_MSG_ID_SET_MAG_OFFSETS
    case MAVLINK_MSG_ID_SET_MAG_OFFSETS:
    {
        mavlink_set_mag_offsets_t packet;
        mavlink_msg_set_mag_offsets_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;
        compass.set_offsets(Vector3f(packet.mag_ofs_x, packet.mag_ofs_y, packet.mag_ofs_z));
        break;
    }
#endif

    // XXX receive a WP from GCS and store in EEPROM
    case MAVLINK_MSG_ID_MISSION_ITEM:
    {
        // decode
        uint8_t result = MAV_MISSION_ACCEPTED;
        mavlink_mission_item_t packet;
        mavlink_msg_mission_item_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // defaults
        tell_command.id = packet.command;

        /*
         *  switch (packet.frame){
         *
         *       case MAV_FRAME_MISSION:
         *       case MAV_FRAME_GLOBAL:
         *               {
         *                       tell_command.lat = 1.0e7*packet.x; // in as DD converted to * t7
         *                       tell_command.lng = 1.0e7*packet.y; // in as DD converted to * t7
         *                       tell_command.alt = packet.z*1.0e2; // in as m converted to cm
         *                       tell_command.options = 0; // absolute altitude
         *                       break;
         *               }
         *
         *       case MAV_FRAME_LOCAL: // local (relative to home position)
         *               {
         *                       tell_command.lat = 1.0e7*ToDeg(packet.x/
         *                       (radius_of_earth*cosf(ToRad(home.lat/1.0e7)))) + home.lat;
         *                       tell_command.lng = 1.0e7*ToDeg(packet.y/radius_of_earth) + home.lng;
         *                       tell_command.alt = packet.z*1.0e2;
         *                       tell_command.options = MASK_OPTIONS_RELATIVE_ALT;
         *                       break;
         *               }
         *       //case MAV_FRAME_GLOBAL_RELATIVE_ALT: // absolute lat/lng, relative altitude
         *       default:
         *               {
         *                       tell_command.lat = 1.0e7 * packet.x; // in as DD converted to * t7
         *                       tell_command.lng = 1.0e7 * packet.y; // in as DD converted to * t7
         *                       tell_command.alt = packet.z * 1.0e2;
         *                       tell_command.options = MASK_OPTIONS_RELATIVE_ALT; // store altitude relative!! Always!!
         *                       break;
         *               }
         *  }
         */

        // we only are supporting Abs position, relative Alt
        tell_command.lat = 1.0e7f * packet.x;                 // in as DD converted to * t7
        tell_command.lng = 1.0e7f * packet.y;                 // in as DD converted to * t7
        tell_command.alt = packet.z * 1.0e2f;
        tell_command.options = 1;                 // store altitude relative to home alt!! Always!!

        switch (tell_command.id) {                                                      // Switch to map APM command fields into MAVLink command fields
        case MAV_CMD_NAV_LOITER_TURNS:
        case MAV_CMD_DO_SET_HOME:
            tell_command.p1 = packet.param1;
            break;

        case MAV_CMD_NAV_ROI:
        case MAV_CMD_DO_SET_ROI:
            tell_command.p1 = packet.param1;                                    // MAV_ROI (aka roi mode) is held in wp's parameter but we actually do nothing with it because we only support pointing at a specific location provided by x,y and z parameters
            break;

        case MAV_CMD_CONDITION_YAW:
            tell_command.p1 = packet.param3;
            tell_command.alt = packet.param1;
            tell_command.lat = packet.param2;
            tell_command.lng = packet.param4;
            break;

        case MAV_CMD_NAV_TAKEOFF:
            tell_command.p1 = 0;
            break;

        case MAV_CMD_CONDITION_CHANGE_ALT:
            tell_command.p1 = packet.param1 * 100;
            break;

        case MAV_CMD_NAV_LOITER_TIME:
            tell_command.p1 = packet.param1;                                    // APM loiter time is in ten second increments
            break;

        case MAV_CMD_CONDITION_DELAY:
        case MAV_CMD_CONDITION_DISTANCE:
            tell_command.lat = packet.param1;
            break;

        case MAV_CMD_DO_JUMP:
            tell_command.lat = packet.param2;
            tell_command.p1  = packet.param1;
            break;

        case MAV_CMD_DO_REPEAT_SERVO:
            tell_command.lng = packet.param4*1000; // time
            tell_command.lat = packet.param3;      // count
            tell_command.alt = packet.param2;      // PWM
            tell_command.p1  = packet.param1;      // channel
            break;
            
        case MAV_CMD_DO_REPEAT_RELAY:
            tell_command.lat = packet.param3*1000; // time
            tell_command.alt = packet.param2;      // count
            tell_command.p1  = packet.param1;      // relay number
            break;
            
        case MAV_CMD_DO_CHANGE_SPEED:
            tell_command.lat = packet.param3;
            tell_command.alt = packet.param2;
            tell_command.p1 = packet.param1;
            break;

        case MAV_CMD_NAV_WAYPOINT:
            tell_command.p1 = packet.param1;
            break;

        case MAV_CMD_DO_SET_PARAMETER:
        case MAV_CMD_DO_SET_RELAY:
        case MAV_CMD_DO_SET_SERVO:
            tell_command.alt = packet.param2;
            tell_command.p1 = packet.param1;
            break;

        case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
            // use alt so we can support 32 bit values
            tell_command.alt = packet.param1;
            break;
        }

        if(packet.current == 2) {                                               //current = 2 is a flag to tell us this is a "guided mode" waypoint and not for the mission
            // initiate guided mode
            do_guided(&tell_command);

            // verify we recevied the command
            mavlink_msg_mission_ack_send(
                chan,
                msg->sysid,
                msg->compid,
                0);

        } else if(packet.current == 3) {                                               //current = 3 is a flag to tell us this is a alt change only

            // add home alt if needed
            if (tell_command.options & MASK_OPTIONS_RELATIVE_ALT) {
                tell_command.alt += home.alt;
            }

            // To-Do: update target altitude for loiter or waypoint controller depending upon nav mode
            // similar to how do_change_alt works
            wp_nav.set_desired_alt(tell_command.alt);

            // verify we recevied the command
            mavlink_msg_mission_ack_send(
                chan,
                msg->sysid,
                msg->compid,
                0);

        } else {
            // Check if receiving waypoints (mission upload expected)
            if (!waypoint_receiving) {
                result = MAV_MISSION_ERROR;
                goto mission_failed;
            }

            // check if this is the requested waypoint
            if (packet.seq != waypoint_request_i) {
                result = MAV_MISSION_INVALID_SEQUENCE;
                goto mission_failed;
            }

                set_cmd_with_index(tell_command, packet.seq);

            // update waypoint receiving state machine
            waypoint_timelast_receive = millis();
            waypoint_timelast_request = 0;
            waypoint_request_i++;

            if (waypoint_request_i > waypoint_request_last) {
                mavlink_msg_mission_ack_send(
                    chan,
                    msg->sysid,
                    msg->compid,
                    result);

                send_text_P(SEVERITY_LOW,PSTR("flight plan received"));
                waypoint_receiving = false;
                // XXX ignores waypoint radius for individual waypoints, can
                // only set WP_RADIUS parameter
            }
        }
        break;

mission_failed:
        // we are rejecting the mission/waypoint
        mavlink_msg_mission_ack_send(
            chan,
            msg->sysid,
            msg->compid,
            result);
        break;
    }

    case MAVLINK_MSG_ID_PARAM_SET:     // 23
    {
        AP_Param                  *vp;
        enum ap_var_type var_type;

        // decode
        mavlink_param_set_t packet;
        mavlink_msg_param_set_decode(msg, &packet);

        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;

        // set parameter

        char key[AP_MAX_NAME_SIZE+1];
        strncpy(key, (char *)packet.param_id, AP_MAX_NAME_SIZE);
        key[AP_MAX_NAME_SIZE] = 0;

        // find the requested parameter
        vp = AP_Param::find(key, &var_type);
        if ((NULL != vp) &&                                                                                     // exists
            !isnan(packet.param_value) &&                                                  // not nan
            !isinf(packet.param_value)) {                                                  // not inf

            // add a small amount before casting parameter values
            // from float to integer to avoid truncating to the
            // next lower integer value.
            float rounding_addition = 0.01;

            // handle variables with standard type IDs
            if (var_type == AP_PARAM_FLOAT) {
                ((AP_Float *)vp)->set_and_save(packet.param_value);
            } else if (var_type == AP_PARAM_INT32) {
                if (packet.param_value < 0) rounding_addition = -rounding_addition;
                float v = packet.param_value+rounding_addition;
                v = constrain_float(v, -2147483648.0, 2147483647.0);
                ((AP_Int32 *)vp)->set_and_save(v);
            } else if (var_type == AP_PARAM_INT16) {
                if (packet.param_value < 0) rounding_addition = -rounding_addition;
                float v = packet.param_value+rounding_addition;
                v = constrain_float(v, -32768, 32767);
                ((AP_Int16 *)vp)->set_and_save(v);
            } else if (var_type == AP_PARAM_INT8) {
                if (packet.param_value < 0) rounding_addition = -rounding_addition;
                float v = packet.param_value+rounding_addition;
                v = constrain_float(v, -128, 127);
                ((AP_Int8 *)vp)->set_and_save(v);
            } else {
                // we don't support mavlink set on this parameter
                break;
            }

            // Report back the new value if we accepted the change
            // we send the value we actually set, which could be
            // different from the value sent, in case someone sent
            // a fractional value to an integer type
            mavlink_msg_param_value_send(
                chan,
                key,
                vp->cast_to_float(var_type),
                mav_var_type(var_type),
                _count_parameters(),
                -1);                         // XXX we don't actually know what its index is...
            DataFlash.Log_Write_Parameter(key, vp->cast_to_float(var_type));
        }

        break;
    }             // end case

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
    {
        // allow override of RC channel values for HIL
        // or for complete GCS control of switch position
        // and RC PWM values.
        if(msg->sysid != g.sysid_my_gcs) break;                         // Only accept control from our gcs
        mavlink_rc_channels_override_t packet;
        int16_t v[8];
        mavlink_msg_rc_channels_override_decode(msg, &packet);

        if (mavlink_check_target(packet.target_system,packet.target_component))
            break;

        v[0] = packet.chan1_raw;
        v[1] = packet.chan2_raw;
        v[2] = packet.chan3_raw;
        v[3] = packet.chan4_raw;
        v[4] = packet.chan5_raw;
        v[5] = packet.chan6_raw;
        v[6] = packet.chan7_raw;
        v[7] = packet.chan8_raw;
        hal.rcin->set_overrides(v, 8);

        // record that rc are overwritten so we can trigger a failsafe if we lose contact with groundstation
        failsafe.rc_override_active = true;
        // a RC override message is consiered to be a 'heartbeat' from the ground station for failsafe purposes
        failsafe.last_heartbeat_ms = millis();
        break;
    }


#if HIL_MODE != HIL_MODE_DISABLED
    case MAVLINK_MSG_ID_HIL_STATE:
    {
        mavlink_hil_state_t packet;
        mavlink_msg_hil_state_decode(msg, &packet);

        float vel = pythagorous2(packet.vx, packet.vy);
        float cog = wrap_360_cd(ToDeg(atan2f(packet.vx, packet.vy)) * 100);

		// if we are erasing the dataflash this object doesnt exist yet. as its called from delay_cb
		if (g_gps == NULL)
			break;
		
        // set gps hil sensor
        g_gps->setHIL(packet.time_usec/1000,
                      packet.lat*1.0e-7, packet.lon*1.0e-7, packet.alt*1.0e-3,
                      vel*1.0e-2, cog*1.0e-2, 0, 10);

        if (!ap.home_is_set) {
            init_home();
        }


        // rad/sec
        Vector3f gyros;
        gyros.x = packet.rollspeed;
        gyros.y = packet.pitchspeed;
        gyros.z = packet.yawspeed;

        // m/s/s
        Vector3f accels;
        accels.x = packet.xacc * (GRAVITY_MSS/1000.0);
        accels.y = packet.yacc * (GRAVITY_MSS/1000.0);
        accels.z = packet.zacc * (GRAVITY_MSS/1000.0);

        ins.set_gyro(gyros);

        ins.set_accel(accels);

        barometer.setHIL(packet.alt*0.001f);
        compass.setHIL(packet.roll, packet.pitch, packet.yaw);

 #if HIL_MODE == HIL_MODE_ATTITUDE
        // set AHRS hil sensor
        ahrs.setHil(packet.roll,packet.pitch,packet.yaw,packet.rollspeed,
                    packet.pitchspeed,packet.yawspeed);
 #endif



        break;
    }
#endif //  HIL_MODE != HIL_MODE_DISABLED


    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        // We keep track of the last time we received a heartbeat from our GCS for failsafe purposes
        if(msg->sysid != g.sysid_my_gcs) break;
        failsafe.last_heartbeat_ms = millis();
        pmTest1++;
        break;
    }

#if CAMERA == ENABLED
    case MAVLINK_MSG_ID_DIGICAM_CONFIGURE:
    {
        camera.configure_msg(msg);
        break;
    }

    case MAVLINK_MSG_ID_DIGICAM_CONTROL:
    {
        camera.control_msg(msg);
        break;
    }
#endif // CAMERA == ENABLED

#if MOUNT == ENABLED
    case MAVLINK_MSG_ID_MOUNT_CONFIGURE:
    {
        camera_mount.configure_msg(msg);
        break;
    }

    case MAVLINK_MSG_ID_MOUNT_CONTROL:
    {
        camera_mount.control_msg(msg);
        break;
    }

    case MAVLINK_MSG_ID_MOUNT_STATUS:
    {
        camera_mount.status_msg(msg);
        break;
    }
#endif // MOUNT == ENABLED

    case MAVLINK_MSG_ID_RADIO:
    case MAVLINK_MSG_ID_RADIO_STATUS:
    {
        mavlink_radio_t packet;
        mavlink_msg_radio_decode(msg, &packet);
        // use the state of the transmit buffer in the radio to
        // control the stream rate, giving us adaptive software
        // flow control
        if (packet.txbuf < 20 && stream_slowdown < 100) {
            // we are very low on space - slow down a lot
            stream_slowdown += 3;
        } else if (packet.txbuf < 50 && stream_slowdown < 100) {
            // we are a bit low on space, slow down slightly
            stream_slowdown += 1;
        } else if (packet.txbuf > 95 && stream_slowdown > 10) {
            // the buffer has plenty of space, speed up a lot
            stream_slowdown -= 2;
        } else if (packet.txbuf > 90 && stream_slowdown != 0) {
            // the buffer has enough space, speed up a bit
            stream_slowdown--;
        }
        break;
    }

    case MAVLINK_MSG_ID_LOG_REQUEST_LIST ... MAVLINK_MSG_ID_LOG_REQUEST_END:
        if (!in_mavlink_delay && !motors.armed()) {
            handle_log_message(msg, DataFlash);
        }
        break;    

/* To-Do: add back support for polygon type fence
#if AC_FENCE == ENABLED
    // receive an AP_Limits fence point from GCS and store in EEPROM
    // receive a fence point from GCS and store in EEPROM
    case MAVLINK_MSG_ID_FENCE_POINT: {
        mavlink_fence_point_t packet;
        mavlink_msg_fence_point_decode(msg, &packet);
        if (packet.count != geofence_limit.fence_total()) {
            send_text_P(SEVERITY_LOW,PSTR("bad fence point"));
        } else {
            Vector2l point;
            point.x = packet.lat*1.0e7f;
            point.y = packet.lng*1.0e7f;
            geofence_limit.set_fence_point_with_index(point, packet.idx);
        }
        break;
    }
    // send a fence point to GCS
    case MAVLINK_MSG_ID_FENCE_FETCH_POINT: {
        mavlink_fence_fetch_point_t packet;
        mavlink_msg_fence_fetch_point_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;
        if (packet.idx >= geofence_limit.fence_total()) {
            send_text_P(SEVERITY_LOW,PSTR("bad fence point"));
        } else {
            Vector2l point = geofence_limit.get_fence_point_with_index(packet.idx);
            mavlink_msg_fence_point_send(chan, 0, 0, packet.idx, geofence_limit.fence_total(),
                                         point.x*1.0e-7f, point.y*1.0e-7f);
        }
        break;
    }
#endif // AC_FENCE ENABLED
*/

    }     // end switch
}
 if (isplane){
    struct Location tell_command = {};                // command for telemetry

    switch (msg->msgid) {

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
    {
        // decode
        mavlink_request_data_stream_t packet;
        mavlink_msg_request_data_stream_decode(msg, &packet);

        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;

        int16_t freq = 0;     // packet frequency

        if (packet.start_stop == 0)
            freq = 0;                     // stop sending
        else if (packet.start_stop == 1)
            freq = packet.req_message_rate;                     // start sending
        else
            break;

        switch (packet.req_stream_id) {
        case MAV_DATA_STREAM_ALL:
            // note that we don't set STREAM_PARAMS - that is internal only
            for (uint8_t i=0; i<STREAM_PARAMS; i++) {
                streamRates[i].set_and_save_ifchanged(freq);
            }
            break;
        case MAV_DATA_STREAM_RAW_SENSORS:
            streamRates[STREAM_RAW_SENSORS].set_and_save_ifchanged(freq);
            break;
        case MAV_DATA_STREAM_EXTENDED_STATUS:
            streamRates[STREAM_EXTENDED_STATUS].set_and_save_ifchanged(freq);
            break;
        case MAV_DATA_STREAM_RC_CHANNELS:
            streamRates[STREAM_RC_CHANNELS].set_and_save_ifchanged(freq);
            break;
        case MAV_DATA_STREAM_RAW_CONTROLLER:
            streamRates[STREAM_RAW_CONTROLLER].set_and_save_ifchanged(freq);
            break;
        case MAV_DATA_STREAM_POSITION:
            streamRates[STREAM_POSITION].set_and_save_ifchanged(freq);
            break;
        case MAV_DATA_STREAM_EXTRA1:
            streamRates[STREAM_EXTRA1].set_and_save_ifchanged(freq);
            break;
        case MAV_DATA_STREAM_EXTRA2:
            streamRates[STREAM_EXTRA2].set_and_save_ifchanged(freq);
            break;
        case MAV_DATA_STREAM_EXTRA3:
            streamRates[STREAM_EXTRA3].set_and_save_ifchanged(freq);
            break;
        }
        break;
    }

    case MAVLINK_MSG_ID_COMMAND_LONG:
    {
        // decode
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component)) break;

        uint8_t result = MAV_RESULT_UNSUPPORTED;

        // do command
        send_text_P(SEVERITY_LOW,PSTR("command received: "));

        switch(packet.command) {

        case MAV_CMD_NAV_LOITER_UNLIM:
            plane_set_mode(PLANE_LOITER);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
            plane_set_mode(PLANE_RTL);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_MISSION_START:
            plane_set_mode(PLANE_AUTO);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_PREFLIGHT_CALIBRATION:
            if (packet.param1 == 1 ||
                packet.param2 == 1) {
                ins.init_accel();
                ahrs.set_trim(Vector3f(0,0,0));             // clear out saved trim
            } else if (packet.param3 == 1) {
                init_barometer(true);
                if (airspeed.enabled()) {
                    zero_airspeed();
                }
            }
            if (packet.param4 == 1) {
                trim_radio();
            }
#if !defined( __AVR_ATmega1280__ )
            if (packet.param5 == 1) {
                float trim_roll, trim_pitch;
                AP_InertialSensor_UserInteract_MAVLink interact(chan);
                if(ins.calibrate_accel(&interact, trim_roll, trim_pitch)) {
                    // reset ahrs's trim to suggested values from calibration routine
                    ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
                }
            }
#endif
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_COMPONENT_ARM_DISARM:
            if (packet.target_component == MAV_COMP_ID_SYSTEM_CONTROL) {
                if (packet.param1 == 1.0f) {
                    // run pre_arm_checks and arm_checks and display failures
                    if (arming.arm(AP_Arming::MAVLINK)) {
                        //only log if arming was successful
                        channel_throttle->enable_out();                        
#if LOGGING_ENABLED == ENABLED
                        Log_Arm_Disarm();
#endif
                        result = MAV_RESULT_ACCEPTED;
                    } else {
                        result = MAV_RESULT_FAILED;
                    }
                } else if (packet.param1 == 0.0f)  {
                    if (arming.disarm()) {
                        if (arming.arming_required() == AP_Arming::YES_ZERO_PWM) {
                            channel_throttle->disable_out();  
                        }
                        // reset the mission on disarm
                        plane_change_command(0);
                        //only log if disarming was successful
#if LOGGING_ENABLED == ENABLED
                        Log_Arm_Disarm();
#endif
                        result = MAV_RESULT_ACCEPTED;
                    } else {
                        result = MAV_RESULT_FAILED;
                    }
                } else {
                    result = MAV_RESULT_UNSUPPORTED;
                }
            } else {
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_DO_SET_MODE:
            switch ((uint16_t)packet.param1) {
            case MAV_MODE_MANUAL_ARMED:
            case MAV_MODE_MANUAL_DISARMED:
                plane_set_mode(PLANE_MANUAL);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_MODE_AUTO_ARMED:
            case MAV_MODE_AUTO_DISARMED:
                plane_set_mode(PLANE_AUTO);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_MODE_STABILIZE_DISARMED:
            case MAV_MODE_STABILIZE_ARMED:
                plane_set_mode(PLANE_FLY_BY_WIRE_A);
                result = MAV_RESULT_ACCEPTED;
                break;

            default:
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_DO_SET_SERVO:
            if (ServoRelayEvents.do_set_servo(packet.param1, packet.param2)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_REPEAT_SERVO:
            if (ServoRelayEvents.do_repeat_servo(packet.param1, packet.param2, packet.param3, packet.param4*1000)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_SET_RELAY:
            if (ServoRelayEvents.do_set_relay(packet.param1, packet.param2)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_REPEAT_RELAY:
            if (ServoRelayEvents.do_repeat_relay(packet.param1, packet.param2, packet.param3*1000)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            if (packet.param1 == 1 || packet.param1 == 3) {
                // when packet.param1 == 3 we reboot to hold in bootloader
                hal.scheduler->reboot(packet.param1 == 3);
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        default:
            break;
        }

        mavlink_msg_command_ack_send(
            chan,
            packet.command,
            result);

        break;
    }


    case MAVLINK_MSG_ID_SET_MODE:
    {
        // decode
        mavlink_set_mode_t packet;
        mavlink_msg_set_mode_decode(msg, &packet);

        if (!(packet.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) {
            // we ignore base_mode as there is no sane way to map
            // from that bitmap to a APM flight mode. We rely on
            // custom_mode instead.
            break;
        }
        switch (packet.custom_mode) {
        case PLANE_MANUAL:
        case PLANE_CIRCLE:
        case PLANE_STABILIZE:
        case PLANE_TRAINING:
        case PLANE_ACRO:
        case PLANE_FLY_BY_WIRE_A:
        case PLANE_FLY_BY_WIRE_B:
        case PLANE_CRUISE:
        case PLANE_AUTO:
        case PLANE_RTL:
        case PLANE_LOITER:
            plane_set_mode((enum FlightMode)packet.custom_mode);
            break;
        }

        break;
    }

    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
    {
        // decode
        mavlink_mission_request_list_t packet;
        mavlink_msg_mission_request_list_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;

        // Start sending waypoints
        mavlink_msg_mission_count_send(
            chan,msg->sysid,
            msg->compid,
            g.command_total + 1);     // + home

        waypoint_receiving       = false;
        waypoint_dest_sysid      = msg->sysid;
        waypoint_dest_compid     = msg->compid;
        break;
    }


    // XXX read a WP from EEPROM and send it to the GCS
    case MAVLINK_MSG_ID_MISSION_REQUEST:
    {
        // decode
        mavlink_mission_request_t packet;
        mavlink_msg_mission_request_decode(msg, &packet);

        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;

        // send waypoint
        tell_command = get_cmd_with_index(packet.seq);

        // set frame of waypoint
        uint8_t frame;

        if (tell_command.options & MASK_OPTIONS_RELATIVE_ALT) {
            frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;     // reference frame
        } else {
            frame = MAV_FRAME_GLOBAL;     // reference frame
        }

        float param1 = 0, param2 = 0, param3 = 0, param4 = 0;

        // time that the mav should loiter in milliseconds
        uint8_t current = 0;     // 1 (true), 0 (false)

        if (packet.seq == (uint16_t)g.command_index)
            current = 1;

        uint8_t autocontinue = 1;     // 1 (true), 0 (false)

        float x = 0, y = 0, z = 0;

        if (tell_command.id < MAV_CMD_NAV_LAST || tell_command.id == MAV_CMD_CONDITION_CHANGE_ALT) {
            // command needs scaling
            x = tell_command.lat/1.0e7;     // local (x), global (latitude)
            y = tell_command.lng/1.0e7;     // local (y), global (longitude)
            z = tell_command.alt/1.0e2;
        }

        switch (tell_command.id) {                                              // Switch to map APM command fields inot MAVLink command fields

        case MAV_CMD_NAV_LOITER_TIME:
        case MAV_CMD_NAV_LOITER_TURNS:
            if (tell_command.options & MASK_OPTIONS_LOITER_DIRECTION) {
                param3 = -abs(g.loiter_radius);
            } else {
                param3 = abs(g.loiter_radius);
            }
        case MAV_CMD_NAV_TAKEOFF:
        case MAV_CMD_DO_SET_HOME:
            param1 = tell_command.p1;
            break;

        case MAV_CMD_NAV_LOITER_UNLIM:
            if (tell_command.options & MASK_OPTIONS_LOITER_DIRECTION) {
                param3 = -abs(g.loiter_radius);
            } else {
                param3 = abs(g.loiter_radius);
            }
            break;
        case MAV_CMD_CONDITION_CHANGE_ALT:
            x=0;                                // Clear fields loaded above that we don't want sent for this command
            y=0;
        case MAV_CMD_CONDITION_DELAY:
        case MAV_CMD_CONDITION_DISTANCE:
            param1 = tell_command.lat;
            break;

        case MAV_CMD_DO_JUMP:
            param2 = tell_command.lat;
            param1 = tell_command.p1;
            break;

        case MAV_CMD_DO_REPEAT_SERVO:
            param4 = tell_command.lng*0.001f; // time
            param3 = tell_command.lat;        // repeat
            param2 = tell_command.alt;        // pwm
            param1 = tell_command.p1;         // channel
            break;

        case MAV_CMD_DO_REPEAT_RELAY:
            param3 = tell_command.lat*0.001f; // time
            param2 = tell_command.alt;        // count
            param1 = tell_command.p1;         // relay number
            break;

        case MAV_CMD_DO_CHANGE_SPEED:
            param3 = tell_command.lat;
            param2 = tell_command.alt;
            param1 = tell_command.p1;
            break;

        case MAV_CMD_DO_SET_PARAMETER:
        case MAV_CMD_DO_SET_RELAY:
        case MAV_CMD_DO_SET_SERVO:
            param2 = tell_command.alt;
            param1 = tell_command.p1;
            break;

        case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
            param1 = tell_command.alt;
            break;
        }

        mavlink_msg_mission_item_send(chan,msg->sysid,
                                      msg->compid,
                                      packet.seq,
                                      frame,
                                      tell_command.id,
                                      current,
                                      autocontinue,
                                      param1,
                                      param2,
                                      param3,
                                      param4,
                                      x,
                                      y,
                                      z);
        break;
    }


    case MAVLINK_MSG_ID_MISSION_ACK:
    {
        // decode
        mavlink_mission_ack_t packet;
        mavlink_msg_mission_ack_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    {
        // decode
        mavlink_param_request_list_t packet;
        mavlink_msg_param_request_list_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // mark the firmware version in the tlog
        send_text_P(SEVERITY_LOW, PSTR(FIRMWARE_STRING));

#if defined(PX4_GIT_VERSION) && defined(NUTTX_GIT_VERSION)
        send_text_P(SEVERITY_LOW, PSTR("PX4: " PX4_GIT_VERSION " NuttX: " NUTTX_GIT_VERSION));
#endif

        // send system ID if we can
        char sysid[40];
        if (hal.util->get_system_id(sysid)) {
            plane_mavlink_send_text(chan, SEVERITY_LOW, sysid);
        }

        // Start sending parameters - next call to ::update will kick the first one out
        _queued_parameter = AP_Param::first(&_queued_parameter_token, &_queued_parameter_type);
        _queued_parameter_index = 0;
        _queued_parameter_count = _count_parameters();
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    {
        // decode
        mavlink_param_request_read_t packet;
        mavlink_msg_param_request_read_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;
        enum ap_var_type p_type;
        AP_Param *vp;
        char param_name[AP_MAX_NAME_SIZE+1];
        if (packet.param_index != -1) {
            AP_Param::ParamToken token;
            vp = AP_Param::find_by_index(packet.param_index, &p_type, &token);
            if (vp == NULL) {
                plane_gcs_send_text_fmt(PSTR("Unknown parameter index %d"), packet.param_index);
                break;
            }
            vp->copy_name_token(token, param_name, AP_MAX_NAME_SIZE, true);
            param_name[AP_MAX_NAME_SIZE] = 0;
        } else {
            strncpy(param_name, packet.param_id, AP_MAX_NAME_SIZE);
            param_name[AP_MAX_NAME_SIZE] = 0;
            vp = AP_Param::find(param_name, &p_type);
            if (vp == NULL) {
                plane_gcs_send_text_fmt(PSTR("Unknown parameter %.16s"), packet.param_id);
                break;
            }
        }

        float value = vp->cast_to_float(p_type);
        mavlink_msg_param_value_send(
            chan,
            param_name,
            value,
            mav_var_type(p_type),
            _count_parameters(),
            packet.param_index);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
    {
        // decode
        mavlink_mission_clear_all_t packet;
        mavlink_msg_mission_clear_all_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component)) break;

        // clear all commands
        g.command_total.set_and_save(0);

        // note that we don't send multiple acks, as otherwise a
        // GCS that is doing a clear followed by a set may see
        // the additional ACKs as ACKs of the set operations
        mavlink_msg_mission_ack_send(chan, msg->sysid, msg->compid, MAV_MISSION_ACCEPTED);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
    {
        // decode
        mavlink_mission_set_current_t packet;
        mavlink_msg_mission_set_current_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // set current command
        plane_change_command(packet.seq);

        mavlink_msg_mission_current_send(chan, g.command_index);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_COUNT:
    {
        // decode
        mavlink_mission_count_t packet;
        mavlink_msg_mission_count_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // start waypoint receiving
        if (packet.count > MAX_WAYPOINTS) {
            packet.count = MAX_WAYPOINTS;
        }
        g.command_total.set_and_save(packet.count - 1);

        waypoint_timelast_receive = millis();
        waypoint_timelast_request = 0;
        waypoint_receiving   = true;
        waypoint_request_i   = 0;
        waypoint_request_last= g.command_total;
        break;
    }

    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
    {
        // decode
        mavlink_mission_write_partial_list_t packet;
        mavlink_msg_mission_write_partial_list_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // start waypoint receiving
        if (packet.start_index > g.command_total ||
            packet.end_index > g.command_total ||
            packet.end_index < packet.start_index) {
            send_text_P(SEVERITY_LOW,PSTR("flight plan update rejected"));
            break;
        }

        waypoint_timelast_receive = millis();
        waypoint_timelast_request = 0;
        waypoint_receiving   = true;
        waypoint_request_i   = packet.start_index;
        waypoint_request_last= packet.end_index;
        break;
    }

#ifdef MAVLINK_MSG_ID_SET_MAG_OFFSETS
    case MAVLINK_MSG_ID_SET_MAG_OFFSETS:
    {
        mavlink_set_mag_offsets_t packet;
        mavlink_msg_set_mag_offsets_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;
        compass.set_offsets(Vector3f(packet.mag_ofs_x, packet.mag_ofs_y, packet.mag_ofs_z));
        break;
    }
#endif

    // XXX receive a WP from GCS and store in EEPROM
    case MAVLINK_MSG_ID_MISSION_ITEM:
    {
        // decode
        mavlink_mission_item_t packet;
        uint8_t result = MAV_MISSION_ACCEPTED;

        mavlink_msg_mission_item_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // defaults
        tell_command.id = packet.command;

        switch (packet.frame)
        {
        case MAV_FRAME_MISSION:
        case MAV_FRAME_GLOBAL:
        {
            tell_command.lat = 1.0e7f*packet.x;                                     // in as DD converted to * t7
            tell_command.lng = 1.0e7f*packet.y;                                     // in as DD converted to * t7
            tell_command.alt = packet.z*1.0e2f;                                     // in as m converted to cm
            tell_command.options = 0;                                     // absolute altitude
            break;
        }

#ifdef MAV_FRAME_LOCAL_NED
        case MAV_FRAME_LOCAL_NED:                         // local (relative to home position)
        {
            tell_command.lat = 1.0e7f*ToDeg(packet.x/
                                           (RADIUS_OF_EARTH*cosf(ToRad(home.lat/1.0e7f)))) + home.lat;
            tell_command.lng = 1.0e7f*ToDeg(packet.y/RADIUS_OF_EARTH) + home.lng;
            tell_command.alt = -packet.z*1.0e2f;
            tell_command.options = MASK_OPTIONS_RELATIVE_ALT;
            break;
        }
#endif

#ifdef MAV_FRAME_LOCAL
        case MAV_FRAME_LOCAL:                         // local (relative to home position)
        {
            tell_command.lat = 1.0e7f*ToDeg(packet.x/
                                           (RADIUS_OF_EARTH*cosf(ToRad(home.lat/1.0e7f)))) + home.lat;
            tell_command.lng = 1.0e7f*ToDeg(packet.y/RADIUS_OF_EARTH) + home.lng;
            tell_command.alt = packet.z*1.0e2f;
            tell_command.options = MASK_OPTIONS_RELATIVE_ALT;
            break;
        }
#endif

        case MAV_FRAME_GLOBAL_RELATIVE_ALT:                         // absolute lat/lng, relative altitude
        {
            tell_command.lat = 1.0e7f * packet.x;                                     // in as DD converted to * t7
            tell_command.lng = 1.0e7f * packet.y;                                     // in as DD converted to * t7
            tell_command.alt = packet.z * 1.0e2f;
            tell_command.options = MASK_OPTIONS_RELATIVE_ALT;                                     // store altitude relative!! Always!!
            break;
        }

        default:
            result = MAV_MISSION_UNSUPPORTED_FRAME;
            break;
        }


        if (result != MAV_MISSION_ACCEPTED) goto plane_mission_failed;

        // Switch to map APM command fields into MAVLink command fields
        switch (tell_command.id) {
        case MAV_CMD_NAV_LOITER_UNLIM:
		    if (packet.param3 < 0) {
		        tell_command.options |= MASK_OPTIONS_LOITER_DIRECTION;
		    }
        case MAV_CMD_NAV_WAYPOINT:
        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        case MAV_CMD_NAV_LAND:
            break;

        case MAV_CMD_NAV_LOITER_TURNS:
        case MAV_CMD_NAV_LOITER_TIME:
            if (packet.param3 < 0) {
                tell_command.options |= MASK_OPTIONS_LOITER_DIRECTION;
            } 
        case MAV_CMD_NAV_TAKEOFF:
        case MAV_CMD_DO_SET_HOME:
            tell_command.p1 = packet.param1;
            break;

        case MAV_CMD_CONDITION_CHANGE_ALT:
            tell_command.lat = packet.param1;
            break;

        case MAV_CMD_CONDITION_DELAY:
        case MAV_CMD_CONDITION_DISTANCE:
            tell_command.lat = packet.param1;
            break;

        case MAV_CMD_DO_JUMP:
            tell_command.lat = packet.param2;
            tell_command.p1 = packet.param1;
            break;

        case MAV_CMD_DO_REPEAT_SERVO:
            tell_command.lng = packet.param4*1000; // time
            tell_command.lat = packet.param3;      // count
            tell_command.alt = packet.param2;      // PWM
            tell_command.p1  = packet.param1;      // channel
            break;

        case MAV_CMD_DO_REPEAT_RELAY:
            tell_command.lat = packet.param3*1000; // time
            tell_command.alt = packet.param2;      // count
            tell_command.p1  = packet.param1;      // relay number
            break;

        case MAV_CMD_DO_CHANGE_SPEED:
            tell_command.lat = packet.param3*1000; // convert to milliseconds
            tell_command.alt = packet.param2;
            tell_command.p1 = packet.param1;
            break;

        case MAV_CMD_DO_SET_PARAMETER:
        case MAV_CMD_DO_SET_RELAY:
        case MAV_CMD_DO_SET_SERVO:
            tell_command.alt = packet.param2;
            tell_command.p1 = packet.param1;
            break;

        case MAV_CMD_DO_DIGICAM_CONTROL:
            break;

        case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
            // use alt so we can support 32 bit values
            tell_command.alt = packet.param1;
            break;

        default:
            result = MAV_MISSION_UNSUPPORTED;
            break;
        }

        if (result != MAV_MISSION_ACCEPTED) goto plane_mission_failed;

        if(packet.current == 2) {                                               //current = 2 is a flag to tell us this is a "guided mode" waypoint and not for the mission
            guided_WP = tell_command;

            // add home alt if needed
            if (guided_WP.options & MASK_OPTIONS_RELATIVE_ALT) {
                guided_WP.alt += home.alt;
            }

            plane_set_mode(PLANE_GUIDED);

            // make any new wp uploaded instant (in case we are already in Guided mode)

			
			//            set_guided_WP();  WORKAROUND FOR PIXHAWK BUILD FAIL
    if (g.loiter_radius < 0) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }

    // copy the current location into the OldWP slot
    // ---------------------------------------
    prev_WP = current_loc;

    // Load the next_WP slot
    // ---------------------
    next_WP = guided_WP;

    // used to control FBW and limit the rate of climb
    // -----------------------------------------------
    target_altitude_cm = current_loc.alt;

    setup_glide_slope();

    loiter_angle_reset();
	// END WORKAROUND
	



            // verify we recevied the command
            mavlink_msg_mission_ack_send(
                chan,
                msg->sysid,
                msg->compid,
                0);

        } else if(packet.current == 3) {                                               //current = 3 is a flag to tell us this is a alt change only

            // add home alt if needed
            if (tell_command.options & MASK_OPTIONS_RELATIVE_ALT) {
                tell_command.alt += home.alt;
            }

            next_WP.alt = tell_command.alt;

            // verify we recevied the command
            mavlink_msg_mission_ack_send(
                chan,
                msg->sysid,
                msg->compid,
                0);

        } else {
            // Check if receiving waypoints (mission upload expected)
            if (!waypoint_receiving) {
                result = MAV_MISSION_ERROR;
                goto plane_mission_failed;
            }

            // check if this is the requested waypoint
            if (packet.seq != waypoint_request_i) {
                result = MAV_MISSION_INVALID_SEQUENCE;
                goto plane_mission_failed;
            }

            plane_set_cmd_with_index(tell_command, packet.seq);

            // update waypoint receiving state machine
            waypoint_timelast_receive = millis();
            waypoint_timelast_request = 0;
            waypoint_request_i++;

            if (waypoint_request_i > waypoint_request_last) {
                mavlink_msg_mission_ack_send(
                    chan,
                    msg->sysid,
                    msg->compid,
                    result);

                send_text_P(SEVERITY_LOW,PSTR("flight plan received"));
                waypoint_receiving = false;
                // XXX ignores waypoint radius for individual waypoints, can
                // only set WP_RADIUS parameter
            }
        }
        break;

plane_mission_failed:
        // we are rejecting the mission/waypoint
        mavlink_msg_mission_ack_send(
            chan,
            msg->sysid,
            msg->compid,
            result);
        break;
    }

#if AC_FENCE == ENABLED
    // receive a fence point from GCS and store in EEPROM
    case MAVLINK_MSG_ID_FENCE_POINT: {
        mavlink_fence_point_t packet;
        mavlink_msg_fence_point_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;
        if (g.fence_action != FENCE_ACTION_NONE) {
            send_text_P(SEVERITY_LOW,PSTR("fencing must be disabled"));
        } else if (packet.count != g.fence_total) {
            send_text_P(SEVERITY_LOW,PSTR("bad fence point"));
        } else {
            Vector2l point;
            point.x = packet.lat*1.0e7;
            point.y = packet.lng*1.0e7;
            set_fence_point_with_index(point, packet.idx);
        }
        break;
    }

    // send a fence point to GCS
    case MAVLINK_MSG_ID_FENCE_FETCH_POINT: {
        mavlink_fence_fetch_point_t packet;
        mavlink_msg_fence_fetch_point_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;
        if (packet.idx >= g.fence_total) {
            send_text_P(SEVERITY_LOW,PSTR("bad fence point"));
        } else {
            Vector2l point = get_fence_point_with_index(packet.idx);
            mavlink_msg_fence_point_send(chan, msg->sysid, msg->compid, packet.idx, g.fence_total,
                                         point.x*1.0e-7, point.y*1.0e-7);
        }
        break;
    }
#endif // GEOFENCE_ENABLED

    // receive a rally point from GCS and store in EEPROM
    case MAVLINK_MSG_ID_RALLY_POINT: {
        mavlink_rally_point_t packet;
        mavlink_msg_rally_point_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;
        
        if (packet.idx >= g.rally_total || 
            packet.idx >= MAX_RALLYPOINTS) {
            send_text_P(SEVERITY_LOW,PSTR("bad rally point message ID"));
            break;
        }

        if (packet.count != g.rally_total) {
            send_text_P(SEVERITY_LOW,PSTR("bad rally point message count"));
            break;
        }

        RallyLocation rally_point;
        rally_point.lat = packet.lat;
        rally_point.lng = packet.lng;
        rally_point.alt = packet.alt;
        rally_point.break_alt = packet.break_alt;
        rally_point.land_dir = packet.land_dir;
        rally_point.flags = packet.flags;
        set_rally_point_with_index(packet.idx, rally_point);
        break;
    }

    //send a rally point to the GCS
    case MAVLINK_MSG_ID_RALLY_FETCH_POINT: {
        mavlink_rally_fetch_point_t packet;
        mavlink_msg_rally_fetch_point_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;
        if (packet.idx > g.rally_total) {
            send_text_P(SEVERITY_LOW, PSTR("bad rally point index"));   
            break;
        }
        RallyLocation rally_point;
        if (!get_rally_point_with_index(packet.idx, rally_point)) {
            send_text_P(SEVERITY_LOW, PSTR("failed to set rally point"));   
            break;
        }

        mavlink_msg_rally_point_send(chan, msg->sysid, msg->compid, packet.idx, 
                                     g.rally_total, rally_point.lat, rally_point.lng, 
                                     rally_point.alt, rally_point.break_alt, rally_point.land_dir, 
                                     rally_point.flags);
        break;
    }    

    case MAVLINK_MSG_ID_PARAM_SET:
    {
        AP_Param                  *vp;
        enum ap_var_type var_type;

        // decode
        mavlink_param_set_t packet;
        mavlink_msg_param_set_decode(msg, &packet);

        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;

        // set parameter

        char key[AP_MAX_NAME_SIZE+1];
        strncpy(key, (char *)packet.param_id, AP_MAX_NAME_SIZE);
        key[AP_MAX_NAME_SIZE] = 0;

        // find the requested parameter
        vp = AP_Param::find(key, &var_type);
        if ((NULL != vp) &&                                 // exists
            !isnan(packet.param_value) &&                       // not nan
            !isinf(packet.param_value)) {                       // not inf

            // add a small amount before casting parameter values
            // from float to integer to avoid truncating to the
            // next lower integer value.
            float rounding_addition = 0.01;

            // handle variables with standard type IDs
            if (var_type == AP_PARAM_FLOAT) {
                ((AP_Float *)vp)->set_and_save(packet.param_value);
            } else if (var_type == AP_PARAM_INT32) {
                if (packet.param_value < 0) rounding_addition = -rounding_addition;
                float v = packet.param_value+rounding_addition;
                v = constrain_float(v, -2147483648.0, 2147483647.0);
                ((AP_Int32 *)vp)->set_and_save(v);
            } else if (var_type == AP_PARAM_INT16) {
                if (packet.param_value < 0) rounding_addition = -rounding_addition;
                float v = packet.param_value+rounding_addition;
                v = constrain_float(v, -32768, 32767);
                ((AP_Int16 *)vp)->set_and_save(v);
            } else if (var_type == AP_PARAM_INT8) {
                if (packet.param_value < 0) rounding_addition = -rounding_addition;
                float v = packet.param_value+rounding_addition;
                v = constrain_float(v, -128, 127);
                ((AP_Int8 *)vp)->set_and_save(v);
            } else {
                // we don't support mavlink set on this parameter
                break;
            }

            // Report back the new value if we accepted the change
            // we send the value we actually set, which could be
            // different from the value sent, in case someone sent
            // a fractional value to an integer type
            mavlink_msg_param_value_send(
                chan,
                key,
                vp->cast_to_float(var_type),
                mav_var_type(var_type),
                _count_parameters(),
                -1);     // XXX we don't actually know what its index is...
#if LOGGING_ENABLED == ENABLED
            DataFlash.Log_Write_Parameter(key, vp->cast_to_float(var_type));
#endif
        }

        break;
    }     // end case

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
    {
        // allow override of RC channel values for HIL
        // or for complete GCS control of switch position
        // and RC PWM values.
        if(msg->sysid != g.sysid_my_gcs) break;                         // Only accept control from our gcs
        mavlink_rc_channels_override_t packet;
        int16_t v[8];
        mavlink_msg_rc_channels_override_decode(msg, &packet);

        if (mavlink_check_target(packet.target_system,packet.target_component))
            break;

        v[0] = packet.chan1_raw;
        v[1] = packet.chan2_raw;
        v[2] = packet.chan3_raw;
        v[3] = packet.chan4_raw;
        v[4] = packet.chan5_raw;
        v[5] = packet.chan6_raw;
        v[6] = packet.chan7_raw;
        v[7] = packet.chan8_raw;

        hal.rcin->set_overrides(v, 8);

        // a RC override message is consiered to be a 'heartbeat' from
        // the ground station for failsafe purposes
        plane_failsafe.last_heartbeat_ms = millis();
        break;
    }

    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        // We keep track of the last time we received a heartbeat from
        // our GCS for failsafe purposes
        if (msg->sysid != g.sysid_my_gcs) break;
        plane_failsafe.last_heartbeat_ms = millis();
        break;
    }

#if HIL_MODE != HIL_MODE_DISABLED
    case MAVLINK_MSG_ID_HIL_STATE:
    {
        mavlink_hil_state_t packet;
        mavlink_msg_hil_state_decode(msg, &packet);

        last_hil_state = packet;

        float vel = pythagorous2(packet.vx, packet.vy);
        float cog = wrap_360_cd(ToDeg(atan2f(packet.vy, packet.vx)) * 100);

        if (g_gps != NULL) {
            // set gps hil sensor
            g_gps->setHIL(packet.time_usec/1000,
                          packet.lat*1.0e-7, packet.lon*1.0e-7, packet.alt*1.0e-3,
                          vel*1.0e-2, cog*1.0e-2, 0, 10);
        }

        // rad/sec
        Vector3f gyros;
        gyros.x = packet.rollspeed;
        gyros.y = packet.pitchspeed;
        gyros.z = packet.yawspeed;

        // m/s/s
        Vector3f accels;
        accels.x = packet.xacc * (GRAVITY_MSS/1000.0);
        accels.y = packet.yacc * (GRAVITY_MSS/1000.0);
        accels.z = packet.zacc * (GRAVITY_MSS/1000.0);

        ins.set_gyro(gyros);
        ins.set_accel(accels);

        barometer.setHIL(packet.alt*0.001f);
        compass.setHIL(packet.roll, packet.pitch, packet.yaw);
        airspeed.disable();

        // cope with DCM getting badly off due to HIL lag
        if (g.hil_err_limit > 0 &&
            (fabsf(packet.roll - ahrs.roll) > ToRad(g.hil_err_limit) ||
             fabsf(packet.pitch - ahrs.pitch) > ToRad(g.hil_err_limit) ||
             wrap_PI(fabsf(packet.yaw - ahrs.yaw)) > ToRad(g.hil_err_limit))) {
            ahrs.reset_attitude(packet.roll, packet.pitch, packet.yaw);
        }
        break;
    }
#endif // HIL_MODE

#if CAMERA == ENABLED
    case MAVLINK_MSG_ID_DIGICAM_CONFIGURE:
    {
        camera.configure_msg(msg);
        break;
    }

    case MAVLINK_MSG_ID_DIGICAM_CONTROL:
    {
        camera.control_msg(msg);
        break;
    }
#endif // CAMERA == ENABLED

#if MOUNT == ENABLED
    case MAVLINK_MSG_ID_MOUNT_CONFIGURE:
    {
        camera_mount.configure_msg(msg);
        break;
    }

    case MAVLINK_MSG_ID_MOUNT_CONTROL:
    {
        camera_mount.control_msg(msg);
        break;
    }

    case MAVLINK_MSG_ID_MOUNT_STATUS:
    {
        camera_mount.status_msg(msg);
        break;
    }
#endif // MOUNT == ENABLED

    case MAVLINK_MSG_ID_RADIO:
    case MAVLINK_MSG_ID_RADIO_STATUS:
    {
        mavlink_radio_t packet;
        mavlink_msg_radio_decode(msg, &packet);

        // record if the GCS has been receiving radio messages from
        // the aircraft
        if (packet.remrssi != 0) {
            plane_failsafe.last_radio_status_remrssi_ms = hal.scheduler->millis();
        }

        // use the state of the transmit buffer in the radio to
        // control the stream rate, giving us adaptive software
        // flow control
        if (packet.txbuf < 20 && stream_slowdown < 100) {
            // we are very low on space - slow down a lot
            stream_slowdown += 3;
        } else if (packet.txbuf < 50 && stream_slowdown < 100) {
            // we are a bit low on space, slow down slightly
            stream_slowdown += 1;
        } else if (packet.txbuf > 95 && stream_slowdown > 10) {
            // the buffer has plenty of space, speed up a lot
            stream_slowdown -= 2;
        } else if (packet.txbuf > 90 && stream_slowdown != 0) {
            // the buffer has enough space, speed up a bit
            stream_slowdown--;
        }
        break;
    }

    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
    case MAVLINK_MSG_ID_LOG_ERASE:
        in_log_download = true;
        // fallthru
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
        if (!in_mavlink_delay) {
            handle_log_message(msg, DataFlash);
        }
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
        in_log_download = false;
        if (!in_mavlink_delay) {
            handle_log_message(msg, DataFlash);
        }
        break;

    default:
        // forward unknown messages to the other link if there is one
        for (uint8_t i=0; i<num_gcs; i++) {
            if (gcs[i].initialised && i != (uint8_t)chan) {
                mavlink_channel_t out_chan = (mavlink_channel_t)i;
                // only forward if it would fit in the transmit buffer
                if (comm_get_txspace(out_chan) > ((uint16_t)msg->len) + MAVLINK_NUM_NON_PAYLOAD_BYTES) {
                    _mavlink_resend_uart(out_chan, msg);
                }
            }
        }
        break;

    } // end switch
}
} // end handle mavlink

/*
 *  a delay() callback that processes MAVLink packets. We set this as the
 *  callback in long running library initialisation routines to allow
 *  MAVLink to process packets while waiting for the initialisation to
 *  complete
 */
static void mavlink_delay_cb()
{
    static uint32_t last_1hz, last_50hz, last_5s;
    if (!gcs[0].initialised || in_mavlink_delay) return;

    in_mavlink_delay = true;

    uint32_t tnow = millis();
    if (tnow - last_1hz > 1000) {
        last_1hz = tnow;
        gcs_send_heartbeat();
        gcs_send_message(MSG_EXTENDED_STATUS1);
    }
    if (tnow - last_50hz > 20) {
        last_50hz = tnow;
        gcs_check_input();
        gcs_data_stream_send();
        gcs_send_deferred();
        notify.update();
    }
    if (tnow - last_5s > 5000) {
        last_5s = tnow;
        gcs_send_text_P(SEVERITY_LOW, PSTR("Initialising APM..."));
    }
    check_usb_mux();

    in_mavlink_delay = false;
}

/*
 *  send a message on both GCS links
 */
static void gcs_send_message(enum ap_message id)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].send_message(id);
        }
    }
}

/*
 *  send data streams in the given rate range on both links
 */
static void gcs_data_stream_send(void)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].data_stream_send();
        }
    }
}

/*
 *  look for incoming commands on the GCS links
 */
static void gcs_check_input(void)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].update();
        }
    }
}

static void gcs_send_text_P(gcs_severity severity, const prog_char_t *str)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].send_text_P(severity, str);
        }
    }
#if LOGGING_ENABLED == ENABLED
    DataFlash.Log_Write_Message_P(str);
#endif
}

/*
 *  send a low priority formatted message to the GCS
 *  only one fits in the queue, so if you send more than one before the
 *  last one gets into the serial buffer then the old one will be lost
 */
void gcs_send_text_fmt(const prog_char_t *fmt, ...)
{
    va_list arg_list;
    gcs[0].pending_status.severity = (uint8_t)SEVERITY_LOW;
    va_start(arg_list, fmt);
    hal.util->vsnprintf_P((char *)gcs[0].pending_status.text,
            sizeof(gcs[0].pending_status.text), fmt, arg_list);
    va_end(arg_list);
    mavlink_send_message(MAVLINK_COMM_0, MSG_STATUSTEXT, 0);
    for (uint8_t i=1; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].pending_status = gcs[0].pending_status;
            mavlink_send_message((mavlink_channel_t)i, MSG_STATUSTEXT, 0);
        }
    }
}

void plane_gcs_send_text_fmt(const prog_char_t *fmt, ...)
{
    va_list arg_list;
    gcs[0].pending_status.severity = (uint8_t)SEVERITY_LOW;
    va_start(arg_list, fmt);
    hal.util->vsnprintf_P((char *)gcs[0].pending_status.text,
            sizeof(gcs[0].pending_status.text), fmt, arg_list);
    va_end(arg_list);
#if LOGGING_ENABLED == ENABLED
    DataFlash.Log_Write_Message(gcs[0].pending_status.text);
#endif
    plane_mavlink_send_message(MAVLINK_COMM_0, MSG_STATUSTEXT);
    for (uint8_t i=1; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].pending_status = gcs[0].pending_status;
            plane_mavlink_send_message((mavlink_channel_t)i, MSG_STATUSTEXT);
        }
    }
}

/*
  send airspeed calibration data
 */
static void gcs_send_airspeed_calibration(const Vector3f &vg)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (comm_get_txspace((mavlink_channel_t)i) - MAVLINK_NUM_NON_PAYLOAD_BYTES >= 
            MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN) {
            airspeed.log_mavlink_send((mavlink_channel_t)i, vg);
        }
    }
}

/**
   retry any deferred messages
 */
static void gcs_retry_deferred(void)
{
    gcs_send_message(MSG_RETRY_DEFERRED);
}

#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/Log.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from DataFlash log memory
// Code to interact with the user to dump or erase logs

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static bool     print_log_menu(void);
static int8_t   dump_log(uint8_t argc,                  const Menu::arg *argv);
static int8_t   erase_logs(uint8_t argc,                const Menu::arg *argv);
static int8_t   select_logs(uint8_t argc,               const Menu::arg *argv);

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
const struct Menu::command log_menu_commands[] PROGMEM = {
    {"dump",        dump_log},
    {"erase",       erase_logs},
    {"enable",      select_logs},
    {"disable",     select_logs}
};

// A Macro to create the Menu
MENU2(log_menu, "Log", log_menu_commands, print_log_menu);

static bool
print_log_menu(void)
{
    cliSerial->printf_P(PSTR("logs enabled: "));

    if (0 == g.log_bitmask) {
        cliSerial->printf_P(PSTR("none"));
    }else{
        // Macro to make the following code a bit easier on the eye.
        // Pass it the capitalised name of the log option, as defined
        // in defines.h but without the LOG_ prefix.  It will check for
        // the bit being set and print the name of the log option to suit.
 #define PLOG(_s) if (g.log_bitmask & MASK_LOG_ ## _s) cliSerial->printf_P(PSTR(" %S"), PSTR(# _s))
        PLOG(ATTITUDE_FAST);
        PLOG(ATTITUDE_MED);
        PLOG(GPS);
        PLOG(PM);
        PLOG(CTUN);
        PLOG(NTUN);
        PLOG(RCIN);
        PLOG(IMU);
        PLOG(CMD);
        PLOG(CURRENT);
        PLOG(RCOUT);
        PLOG(OPTFLOW);
        PLOG(COMPASS);
        PLOG(TECS);
        PLOG(CAMERA);
 #undef PLOG
    }

    cliSerial->println();

    DataFlash.ListAvailableLogs(cliSerial);
    return(true);
}

static int8_t
dump_log(uint8_t argc, const Menu::arg *argv)
{
    int16_t dump_log;
    uint16_t dump_log_start;
    uint16_t dump_log_end;
    uint16_t last_log_num;

    // check that the requested log number can be read
    dump_log = argv[1].i;
    last_log_num = DataFlash.find_last_log();

    if (dump_log == -2) {
        DataFlash.DumpPageInfo(cliSerial);
        return(-1);
    } else if (dump_log <= 0) {
        cliSerial->printf_P(PSTR("dumping all\n"));
        Log_Read(0, 1, 0);
        return(-1);
    } else if ((argc != 2) || ((uint16_t)dump_log <= (last_log_num - DataFlash.get_num_logs())) || (static_cast<uint16_t>(dump_log) > last_log_num)) {
        cliSerial->printf_P(PSTR("bad log number\n"));
        return(-1);
    }

    DataFlash.get_log_boundaries(dump_log, dump_log_start, dump_log_end);
    Log_Read((uint16_t)dump_log, dump_log_start, dump_log_end);
    return (0);
}

static void do_erase_logs(void)
{
	gcs_send_text_P(SEVERITY_LOW, PSTR("Erasing logs\n"));
    DataFlash.EraseAll();
	gcs_send_text_P(SEVERITY_LOW, PSTR("Log erase complete\n"));
}

static int8_t
erase_logs(uint8_t argc, const Menu::arg *argv)
{
    in_mavlink_delay = true;
    do_erase_logs();
    in_mavlink_delay = false;
    return 0;
}

static int8_t
select_logs(uint8_t argc, const Menu::arg *argv)
{
    uint16_t bits;

    if (argc != 2) {
        cliSerial->printf_P(PSTR("missing log type\n"));
        return(-1);
    }

    bits = 0;

    // Macro to make the following code a bit easier on the eye.
    // Pass it the capitalised name of the log option, as defined
    // in defines.h but without the LOG_ prefix.  It will check for
    // that name as the argument to the command, and set the bit in
    // bits accordingly.
    //
    if (!strcasecmp_P(argv[1].str, PSTR("all"))) {
        bits = ~0;
    } else {
 #define TARG(_s)        if (!strcasecmp_P(argv[1].str, PSTR(# _s))) bits |= MASK_LOG_ ## _s
        TARG(ATTITUDE_FAST);
        TARG(ATTITUDE_MED);
        TARG(GPS);
        TARG(PM);
        TARG(CTUN);
        TARG(NTUN);
        TARG(RCIN);
        TARG(IMU);
        TARG(CMD);
        TARG(CURRENT);
        TARG(RCOUT);
        TARG(OPTFLOW);
        TARG(COMPASS);
        TARG(TECS);
        TARG(CAMERA);
 #undef TARG
    }

    if (!strcasecmp_P(argv[0].str, PSTR("enable"))) {
        g.log_bitmask.set_and_save(g.log_bitmask | bits);
    }else{
        g.log_bitmask.set_and_save(g.log_bitmask & ~bits);
    }

    return(0);
}

static int8_t
process_logs(uint8_t argc, const Menu::arg *argv)
{
    log_menu.run();
    return 0;
}

#if AUTOTUNE == ENABLED
struct PACKED log_AutoTune {
    LOG_PACKET_HEADER;
    uint8_t axis;           // roll or pitch
    uint8_t tune_step;      // tuning PI or D up or down
    float   rate_min;       // maximum achieved rotation rate
    float   rate_max;       // maximum achieved rotation rate
    float   new_gain_rp;       // newly calculated gain
    float   new_gain_rd;       // newly calculated gain
    float   new_gain_sp;       // newly calculated gain
};

// Write an Current data packet
static void Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float rate_min, float rate_max, float new_gain_rp, float new_gain_rd, float new_gain_sp)
{
    struct log_AutoTune pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AUTOTUNE_MSG),
        axis        : axis,
        tune_step   : tune_step,
        rate_min    : rate_min,
        rate_max    : rate_max,
        new_gain_rp  : new_gain_rp,
        new_gain_rd  : new_gain_rd,
        new_gain_sp  : new_gain_sp
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_AutoTuneDetails {
    LOG_PACKET_HEADER;
    int16_t angle_cd;       // lean angle in centi-degrees
    float   rate_cds;       // current rotation rate in centi-degrees / second
};

// Write an Current data packet
static void Log_Write_AutoTuneDetails(int16_t angle_cd, float rate_cds)
{
    struct log_AutoTuneDetails pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AUTOTUNEDETAILS_MSG),
        angle_cd    : angle_cd,
        rate_cds    : rate_cds
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}
#endif

struct PACKED log_Current {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t  throttle_out;
    uint32_t throttle_integrator;
    int16_t  battery_voltage;
    int16_t  current_amps;
    uint16_t board_voltage;
    float    current_total;
};

// Write a Current data packet
static void Log_Write_Current()
{
    struct log_Current pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CURRENT_MSG),
        time_ms             : hal.scheduler->millis(),
        throttle_out        : g.rc_3.servo_out,
        throttle_integrator : throttle_integrator,
        battery_voltage     : (int16_t) (battery.voltage() * 100.0f),
        current_amps        : (int16_t) (battery.current_amps() * 100.0f),
        board_voltage       : board_voltage(),
        current_total       : battery.current_total_mah()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Optflow {
    LOG_PACKET_HEADER;
    int16_t dx;
    int16_t dy;
    uint8_t surface_quality;
    int16_t x_cm;
    int16_t y_cm;
    int32_t roll;
    int32_t pitch;
};

// Write an optical flow packet
static void Log_Write_Optflow()
{
 #if OPTFLOW == ENABLED
    struct log_Optflow pkt = {
        LOG_PACKET_HEADER_INIT(LOG_OPTFLOW_MSG),
        dx              : optflow.dx,
        dy              : optflow.dy,
        surface_quality : optflow.surface_quality,
        x_cm            : (int16_t) optflow.x_cm,
        y_cm            : (int16_t) optflow.y_cm,
        roll            : of_roll,
        pitch           : of_pitch
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
 #endif     // OPTFLOW == ENABLED
}

struct PACKED log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    float    desired_pos_x;
    float    desired_pos_y;
    float    pos_x;
    float    pos_y;
    float    desired_vel_x;
    float    desired_vel_y;
    float    vel_x;
    float    vel_y;
    float    desired_accel_x;
    float    desired_accel_y;
};

// Write a navigation tuning packet
static void Log_Write_Nav_Tuning()
{
    const Vector3f &desired_position = wp_nav.get_loiter_target();
    const Vector3f &position = inertial_nav.get_position();
    const Vector3f &velocity = inertial_nav.get_velocity();

    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NAV_TUNING_MSG),
        time_ms         : hal.scheduler->millis(),
        desired_pos_x   : desired_position.x,
        desired_pos_y   : desired_position.y,
        pos_x           : position.x,
        pos_y           : position.y,
        desired_vel_x   : wp_nav.desired_vel.x,
        desired_vel_y   : wp_nav.desired_vel.y,
        vel_x           : velocity.x,
        vel_y           : velocity.y,
        desired_accel_x : wp_nav.desired_accel.x,
        desired_accel_y : wp_nav.desired_accel.y
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED plane_log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint16_t yaw;
    uint32_t plane_wp_distance;
    uint16_t target_bearing_cd;
    uint16_t nav_bearing_cd;
    int16_t altitude_error_cm;
    int16_t airspeed_cm;
    float   altitude;
    uint32_t groundspeed_cm;
};

// Write a navigation tuning packet
static void plane_Log_Write_Nav_Tuning()
{
    struct plane_log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NTUN_MSG),
        time_ms             : hal.scheduler->millis(),
        yaw                 : (uint16_t)ahrs.yaw_sensor,
        plane_wp_distance         : plane_wp_distance,
        target_bearing_cd   : (uint16_t)nav_controller->target_bearing_cd(),
        nav_bearing_cd      : (uint16_t)nav_controller->nav_bearing_cd(),
        altitude_error_cm   : (int16_t)altitude_error_cm,
        airspeed_cm         : (int16_t)airspeed.get_airspeed_cm(),
        altitude            : barometer.get_altitude(),
        groundspeed_cm      : g_gps->ground_speed_cm
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t  throttle_in;
    int16_t  angle_boost;
    int16_t  throttle_out;
    float    desired_alt;
    float    inav_alt;
    int32_t  baro_alt;
    int16_t  desired_sonar_alt;
    int16_t  sonar_alt;
    int16_t  desired_climb_rate;
    int16_t  climb_rate;
};

// Write a control tuning packet
static void Log_Write_Control_Tuning()
{
    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        time_ms             : hal.scheduler->millis(),
        throttle_in         : g.rc_3.control_in,
        angle_boost         : angle_boost,
        throttle_out        : g.rc_3.servo_out,
        desired_alt         : get_target_alt_for_reporting() / 100.0f,
        inav_alt            : current_loc.alt / 100.0f,
        baro_alt            : baro_alt,
        desired_sonar_alt   : (int16_t)target_sonar_alt,
        sonar_alt           : sonar_alt,
        desired_climb_rate  : desired_climb_rate,
        climb_rate          : climb_rate
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED plane_log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t nav_roll_cd;
    int16_t roll;
    int16_t nav_pitch_cd;
    int16_t pitch;
    int16_t throttle_out;
    int16_t rudder_out;
    float   accel_y;
};

// Write a control tuning packet
static void plane_Log_Write_Control_Tuning()
{
    Vector3f accel = ins.get_accel();
    struct plane_log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CTUN_MSG),
        time_ms         : hal.scheduler->millis(),
        nav_roll_cd     : (int16_t)nav_roll_cd,
        roll            : (int16_t)ahrs.roll_sensor,
        nav_pitch_cd    : (int16_t)nav_pitch_cd,
        pitch           : (int16_t)ahrs.pitch_sensor,
        throttle_out    : (int16_t)channel_throttle->servo_out,
        rudder_out      : (int16_t)channel_rudder->servo_out,
        accel_y         : accel.y
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Compass {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t  mag_x;
    int16_t  mag_y;
    int16_t  mag_z;
    int16_t  offset_x;
    int16_t  offset_y;
    int16_t  offset_z;
    int16_t  motor_offset_x;
    int16_t  motor_offset_y;
    int16_t  motor_offset_z;
};

// Write a Compass packet
static void Log_Write_Compass()
{
    const Vector3f &mag_offsets = compass.get_offsets(0);
    const Vector3f &mag_motor_offsets = compass.get_motor_offsets(0);
    const Vector3f &mag = compass.get_field(0);
    struct log_Compass pkt = {
        LOG_PACKET_HEADER_INIT(LOG_COMPASS_MSG),
        time_ms         : hal.scheduler->millis(),
        mag_x           : (int16_t)mag.x,
        mag_y           : (int16_t)mag.y,
        mag_z           : (int16_t)mag.z,
        offset_x        : (int16_t)mag_offsets.x,
        offset_y        : (int16_t)mag_offsets.y,
        offset_z        : (int16_t)mag_offsets.z,
        motor_offset_x  : (int16_t)mag_motor_offsets.x,
        motor_offset_y  : (int16_t)mag_motor_offsets.y,
        motor_offset_z  : (int16_t)mag_motor_offsets.z
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
#if COMPASS_MAX_INSTANCES > 1
    if (compass.get_count() > 1) {
        const Vector3f &mag2_offsets = compass.get_offsets(1);
        const Vector3f &mag2_motor_offsets = compass.get_motor_offsets(1);
        const Vector3f &mag2 = compass.get_field(1);
        struct log_Compass pkt2 = {
            LOG_PACKET_HEADER_INIT(LOG_COMPASS2_MSG),
            time_ms         : hal.scheduler->millis(),
            mag_x           : (int16_t)mag2.x,
            mag_y           : (int16_t)mag2.y,
            mag_z           : (int16_t)mag2.z,
            offset_x        : (int16_t)mag2_offsets.x,
            offset_y        : (int16_t)mag2_offsets.y,
            offset_z        : (int16_t)mag2_offsets.z,
            motor_offset_x  : (int16_t)mag2_motor_offsets.x,
            motor_offset_y  : (int16_t)mag2_motor_offsets.y,
            motor_offset_z  : (int16_t)mag2_motor_offsets.z
        };
        DataFlash.WriteBlock(&pkt2, sizeof(pkt2));
    }
#endif
}

struct PACKED plane_log_Compass {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
    int16_t offset_x;
    int16_t offset_y;
    int16_t offset_z;
};

// Write a Compass packet
static void plane_Log_Write_Compass()
{
    const Vector3f &mag_offsets = compass.get_offsets();
    const Vector3f &mag = compass.get_field();
    struct plane_log_Compass pkt = {
        LOG_PACKET_HEADER_INIT(LOG_COMPASS_MSG),
        time_ms         : hal.scheduler->millis(),
        mag_x           : (int16_t)mag.x,
        mag_y           : (int16_t)mag.y,
        mag_z           : (int16_t)mag.z,
        offset_x        : (int16_t)mag_offsets.x,
        offset_y        : (int16_t)mag_offsets.y,
        offset_z        : (int16_t)mag_offsets.z
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
#if COMPASS_MAX_INSTANCES > 1
    if (compass.get_count() > 1) {
        const Vector3f &mag2_offsets = compass.get_offsets(1);
        const Vector3f &mag2 = compass.get_field(1);
        struct plane_log_Compass pkt2 = {
            LOG_PACKET_HEADER_INIT(LOG_COMPASS2_MSG),
            time_ms         : hal.scheduler->millis(),
            mag_x           : (int16_t)mag2.x,
            mag_y           : (int16_t)mag2.y,
            mag_z           : (int16_t)mag2.z,
            offset_x        : (int16_t)mag2_offsets.x,
            offset_y        : (int16_t)mag2_offsets.y,
            offset_z        : (int16_t)mag2_offsets.z
        };
        DataFlash.WriteBlock(&pkt2, sizeof(pkt2));
    }
#endif
}

struct PACKED log_Performance {
    LOG_PACKET_HEADER;
    uint8_t renorm_count;
    uint8_t renorm_blowup;
    uint16_t num_long_running;
    uint16_t num_loops;
    uint32_t max_time;
    int16_t  pm_test;
    uint8_t i2c_lockup_count;
    uint16_t ins_error_count;
    uint8_t inav_error_count;
};

// Write a performance monitoring packet
static void Log_Write_Performance()
{
    struct log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        renorm_count     : ahrs.renorm_range_count,
        renorm_blowup    : ahrs.renorm_blowup_count,
        num_long_running : perf_info_get_num_long_running(),
        num_loops        : perf_info_get_num_loops(),
        max_time         : perf_info_get_max_time(),
        pm_test          : pmTest1,
        i2c_lockup_count : hal.i2c->lockup_count(),
        ins_error_count  : ins.error_count(),
        inav_error_count : inertial_nav.error_count()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED plane_log_Performance {
    LOG_PACKET_HEADER;
    uint32_t loop_time;
    uint16_t main_loop_count;
    uint32_t perf_info_max_time;
    uint8_t  renorm_count;
    uint8_t  renorm_blowup;
    int16_t  gyro_drift_x;
    int16_t  gyro_drift_y;
    int16_t  gyro_drift_z;
    uint8_t  i2c_lockup_count;
    uint16_t ins_error_count;
};

// Write a performance monitoring packet
static void plane_Log_Write_Performance()
{
    struct plane_log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        loop_time       : millis() - perf_mon_timer,
        main_loop_count : mainLoop_count,
        perf_info_max_time        : perf_info_max_time,
        renorm_count    : ahrs.renorm_range_count,
        renorm_blowup   : ahrs.renorm_blowup_count,
        gyro_drift_x    : (int16_t)(ahrs.get_gyro_drift().x * 1000),
        gyro_drift_y    : (int16_t)(ahrs.get_gyro_drift().y * 1000),
        gyro_drift_z    : (int16_t)(ahrs.get_gyro_drift().z * 1000),
        i2c_lockup_count: hal.i2c->lockup_count(),
        ins_error_count  : ins.error_count()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Cmd {
    LOG_PACKET_HEADER;
    uint8_t command_total;
    uint8_t command_number;
    uint8_t waypoint_id;
    uint8_t waypoint_options;
    uint8_t waypoint_param1;
    int32_t waypoint_altitude;
    int32_t waypoint_latitude;
    int32_t waypoint_longitude;
};

// Write a command processing packet
static void Log_Write_Cmd(uint8_t num, const struct Location *wp)
{
    struct log_Cmd pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CMD_MSG),
        command_total       : g.command_total,
        command_number      : num,
        waypoint_id         : wp->id,
        waypoint_options    : wp->options,
        waypoint_param1     : wp->p1,
        waypoint_altitude   : wp->alt,
        waypoint_latitude   : wp->lat,
        waypoint_longitude  : wp->lng
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Attitude {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t  control_roll;
    int16_t  roll;
    int16_t  control_pitch;
    int16_t  pitch;
    uint16_t control_yaw;
    uint16_t yaw;
};

// Write an attitude packet
static void Log_Write_Attitude()
{
    struct log_Attitude pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        time_ms         : hal.scheduler->millis(),
        control_roll    : (int16_t)control_roll,
        roll            : (int16_t)ahrs.roll_sensor,
        control_pitch   : (int16_t)control_pitch,
        pitch           : (int16_t)ahrs.pitch_sensor,
        control_yaw     : (uint16_t)control_yaw,
        yaw             : (uint16_t)ahrs.yaw_sensor
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED plane_log_Attitude {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t roll;
    int16_t pitch;
    uint16_t yaw;
    uint16_t error_rp;
    uint16_t error_yaw;
};

// Write an attitude packet
static void plane_Log_Write_Attitude(void)
{
    struct plane_log_Attitude pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        time_ms : hal.scheduler->millis(),
        roll  : (int16_t)ahrs.roll_sensor,
        pitch : (int16_t)ahrs.pitch_sensor,
        yaw   : (uint16_t)ahrs.yaw_sensor,
        error_rp  : (uint16_t)(ahrs.get_error_rp() * 100),
        error_yaw : (uint16_t)(ahrs.get_error_yaw() * 100)
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Mode {
    LOG_PACKET_HEADER;
    uint8_t mode;
    int16_t throttle_cruise;
};

// Write a mode packet
static void Log_Write_Mode(uint8_t mode)
{
    struct log_Mode pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MODE_MSG),
        mode            : mode,
        throttle_cruise : g.throttle_cruise,
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED plane_log_Mode {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint8_t mode;
    uint8_t mode_num;
};

// Write a mode packet
static void plane_Log_Write_Mode(uint8_t mode)
{
    struct plane_log_Mode pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MODE_MSG),
        time_ms  : hal.scheduler->millis(),
        mode     : mode,
        mode_num : mode
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Startup {
    LOG_PACKET_HEADER;
    uint8_t startup_type;
    uint8_t command_total;
};

// Write Startup packet
static void Log_Write_Startup()
{
    struct log_Startup pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STARTUP_MSG)
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Camera {
    LOG_PACKET_HEADER;
    uint32_t gps_time;
    uint16_t gps_week;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  altitude;
    int16_t  roll;
    int16_t  pitch;
    uint16_t yaw;
};

// Write a Camera packet
static void Log_Write_Camera()
{
#if CAMERA == ENABLED
    struct log_Camera pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CAMERA_MSG),
        gps_time    : g_gps->time_week_ms,
        gps_week    : g_gps->time_week,
        latitude    : current_loc.lat,
        longitude   : current_loc.lng,
        altitude    : current_loc.alt,
        roll        : (int16_t)ahrs.roll_sensor,
        pitch       : (int16_t)ahrs.pitch_sensor,
        yaw         : (uint16_t)ahrs.yaw_sensor
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
#endif
}

// Write a TECS tuning packet
static void Log_Write_TECS_Tuning(void)
{
    SpdHgt_Controller->log_data(DataFlash, LOG_TECS_MSG);
}

struct PACKED log_Arm_Disarm {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint8_t  arm_state;
    uint16_t arm_checks;
};

static void Log_Arm_Disarm() {
    struct log_Arm_Disarm pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ARM_DISARM_MSG),
        time_ms                 : hal.scheduler->millis(),
        arm_state               : arming.is_armed(),
        arm_checks              : arming.get_enabled_checks()      
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

static void Log_Write_GPS(void)
{
    DataFlash.Log_Write_GPS(g_gps, current_loc.alt);
}

static void Log_Write_IMU() 
{
    DataFlash.Log_Write_IMU(ins);
}

static void Log_Write_Baro(void)
{
    DataFlash.Log_Write_Baro(barometer);
}

struct PACKED log_AIRSPEED {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    float   airspeed;
    float   diffpressure;
    int16_t temperature;
};

// Write a AIRSPEED packet
static void Log_Write_Airspeed(void)
{
    float temperature;
    if (!airspeed.get_temperature(temperature)) {
        temperature = 0;
    }
    struct log_AIRSPEED pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AIRSPEED_MSG),
        timestamp     : hal.scheduler->millis(),
        airspeed      : airspeed.get_raw_airspeed(),
        diffpressure  : airspeed.get_differential_pressure(),
        temperature   : (int16_t)(temperature * 100.0f)
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Event {
    LOG_PACKET_HEADER;
    uint8_t id;
};

// Wrote an event packet
static void Log_Write_Event(uint8_t id)
{
    if (g.log_bitmask != 0) {
        struct log_Event pkt = {
            LOG_PACKET_HEADER_INIT(LOG_EVENT_MSG),
            id  : id
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int16t {
    LOG_PACKET_HEADER;
    uint8_t id;
    int16_t data_value;
};

// Write an int16_t data packet
static void Log_Write_Data(uint8_t id, int16_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_Int16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT16_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt16t {
    LOG_PACKET_HEADER;
    uint8_t id;
    uint16_t data_value;
};

// Write an uint16_t data packet
static void Log_Write_Data(uint8_t id, uint16_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_UInt16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT16_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int32t {
    LOG_PACKET_HEADER;
    uint8_t id;
    int32_t data_value;
};

// Write an int32_t data packet
static void Log_Write_Data(uint8_t id, int32_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_Int32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT32_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt32t {
    LOG_PACKET_HEADER;
    uint8_t id;
    uint32_t data_value;
};

// Write a uint32_t data packet
static void Log_Write_Data(uint8_t id, uint32_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_UInt32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT32_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Float {
    LOG_PACKET_HEADER;
    uint8_t id;
    float data_value;
};

// Write a float data packet
static void Log_Write_Data(uint8_t id, float value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_Float pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_FLOAT_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Error {
    LOG_PACKET_HEADER;
    uint8_t sub_system;
    uint8_t error_code;
};

// Write an error packet
static void Log_Write_Error(uint8_t sub_system, uint8_t error_code)
{
    struct log_Error pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ERROR_MSG),
        sub_system    : sub_system,
        error_code    : error_code,
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

static const struct LogStructure log_structure[] PROGMEM = {
    LOG_COMMON_STRUCTURES,
#if AUTOTUNE == ENABLED
    { LOG_AUTOTUNE_MSG, sizeof(log_AutoTune),
      "ATUN", "BBfffff",       "Axis,TuneStep,RateMin,RateMax,RPGain,RDGain,SPGain" },
    { LOG_AUTOTUNEDETAILS_MSG, sizeof(log_AutoTuneDetails),
      "ATDE", "cf",          "Angle,Rate" },
#endif
    { LOG_CURRENT_MSG, sizeof(log_Current),             
      "CURR", "IhIhhhf",     "TimeMS,ThrOut,ThrInt,Volt,Curr,Vcc,CurrTot" },
    { LOG_OPTFLOW_MSG, sizeof(log_Optflow),       
      "OF",   "hhBccee",   "Dx,Dy,SQual,X,Y,Roll,Pitch" },
    { LOG_NAV_TUNING_MSG, sizeof(log_Nav_Tuning),       
      "NTUN", "Iffffffffff", "TimeMS,DPosX,DPosY,PosX,PosY,DVelX,DVelY,VelX,VelY,DAccX,DAccY" },
    { LOG_CONTROL_TUNING_MSG, sizeof(log_Control_Tuning),
      "CTUN", "Ihhhffecchh", "TimeMS,ThrIn,AngBst,ThrOut,DAlt,Alt,BarAlt,DSAlt,SAlt,DCRt,CRt" },
    { LOG_COMPASS_MSG, sizeof(log_Compass),             
      "MAG", "Ihhhhhhhhh",    "TimeMS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ" },
    { LOG_COMPASS2_MSG, sizeof(log_Compass),             
      "MAG2","Ihhhhhhhhh",    "TimeMS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ" },
    { LOG_PERFORMANCE_MSG, sizeof(log_Performance), 
      "PM",  "BBHHIhBHB",    "RenCnt,RenBlw,NLon,NLoop,MaxT,PMT,I2CErr,INSErr,INAVErr" },
    { LOG_CMD_MSG, sizeof(log_Cmd),                 
      "CMD", "BBBBBeLL",     "CTot,CNum,CId,COpt,Prm1,Alt,Lat,Lng" },
    { LOG_ATTITUDE_MSG, sizeof(log_Attitude),       
      "ATT", "IccccCC",      "TimeMS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw" },
    { LOG_MODE_MSG, sizeof(log_Mode),
      "MODE", "Mh",          "Mode,ThrCrs" },
    { LOG_STARTUP_MSG, sizeof(log_Startup),         
      "STRT", "",            "" },
    { LOG_EVENT_MSG, sizeof(log_Event),         
      "EV",   "B",           "Id" },
    { LOG_DATA_INT16_MSG, sizeof(log_Data_Int16t),         
      "D16",   "Bh",         "Id,Value" },
    { LOG_DATA_UINT16_MSG, sizeof(log_Data_UInt16t),         
      "DU16",  "BH",         "Id,Value" },
    { LOG_DATA_INT32_MSG, sizeof(log_Data_Int32t),         
      "D32",   "Bi",         "Id,Value" },
    { LOG_DATA_UINT32_MSG, sizeof(log_Data_UInt32t),         
      "DU32",  "BI",         "Id,Value" },
    { LOG_DATA_FLOAT_MSG, sizeof(log_Data_Float),         
      "DFLT",  "Bf",         "Id,Value" },
    { LOG_CAMERA_MSG, sizeof(log_Camera),                 
      "CAM",   "IHLLeccC",   "GPSTime,GPSWeek,Lat,Lng,Alt,Roll,Pitch,Yaw" },
    { LOG_ERROR_MSG, sizeof(log_Error),         
      "ERR",   "BB",         "Subsys,ECode" },
    { LOG_CTUN_MSG, sizeof(plane_log_Control_Tuning),     
      "CTUN", "Icccchhf",    "TimeMS,NavRoll,Roll,NavPitch,Pitch,ThrOut,RdrOut,AccY" },
    { LOG_NTUN_MSG, sizeof(plane_log_Nav_Tuning),         
      "NTUN", "ICICCccfI",   "TimeMS,Yaw,WpDist,TargBrg,NavBrg,AltErr,Arspd,Alt,GSpdCM" },
    { LOG_MODE_MSG, sizeof(plane_log_Mode),             
      "MODE", "IMB",         "TimeMS,Mode,ModeNum" },
    { LOG_ARM_DISARM_MSG, sizeof(log_Arm_Disarm),
      "ARM", "IHB", "TimeMS,ArmState,ArmChecks" },
    { LOG_AIRSPEED_MSG, sizeof(log_AIRSPEED),
      "ARSP",  "Iffc",     "TimeMS,Airspeed,DiffPress,Temp" },
    TECS_LOG_FORMAT(LOG_TECS_MSG),
};

// Read the DataFlash log memory
static void Log_Read(uint16_t log_num, uint16_t start_page, uint16_t end_page)
{
 #ifdef AIRFRAME_NAME
    cliSerial->printf_P(PSTR((AIRFRAME_NAME)));
 #endif

    cliSerial->printf_P(PSTR("\n" FIRMWARE_STRING
                             "\nFree RAM: %u\n"),
                        (unsigned) hal.util->available_memory());

    cliSerial->println_P(PSTR(HAL_BOARD_NAME));

	DataFlash.LogReadProcess(log_num, start_page, end_page, 
                             print_flight_mode,
                             cliSerial);
}

// start a new log
static void start_logging() 
{
    if (g.log_bitmask != 0) {
        if (!ap.logging_started) {
            ap.logging_started = true;
            in_mavlink_delay = true;
            DataFlash.StartNewLog();
            in_mavlink_delay = false;
            DataFlash.Log_Write_Message_P(PSTR(FIRMWARE_STRING));

#if defined(PX4_GIT_VERSION) && defined(NUTTX_GIT_VERSION)
            DataFlash.Log_Write_Message_P(PSTR("PX4: " PX4_GIT_VERSION " NuttX: " NUTTX_GIT_VERSION));
#endif

            // write system identifier as well if available
            char sysid[40];
            if (hal.util->get_system_id(sysid)) {
                DataFlash.Log_Write_Message(sysid);
            }

            // log the flight mode
            Log_Write_Mode(control_mode);
        }
        // enable writes
        DataFlash.EnableWrites(true);
    }
}

#else // LOGGING_ENABLED

// dummy functions
static void Log_Write_Startup() {}
static void Log_Write_Cmd(uint8_t num, const struct Location *wp) {}
static void Log_Write_Mode(uint8_t mode) {}
static void plane_Log_Write_Mode(uint8_t mode) {}

static void Log_Write_IMU() {}
static void Log_Write_GPS() {}
#if AUTOTUNE == ENABLED
static void Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float rate_min, float rate_max, float new_gain_rp, float new_gain_rd, float new_gain_sp) {}
static void Log_Write_AutoTuneDetails(int16_t angle_cd, float rate_cds) {}
#endif
static void Log_Write_Current() {}
static void Log_Write_Compass() {}
static void Log_Write_Attitude() {}
static void plane_Log_Write_Compass() {}
static void plane_Log_Write_Attitude() {}
static void Log_Write_Data(uint8_t id, int16_t value){}
static void Log_Write_Data(uint8_t id, uint16_t value){}
static void Log_Write_Data(uint8_t id, int32_t value){}
static void Log_Write_Data(uint8_t id, uint32_t value){}
static void Log_Write_Data(uint8_t id, float value){}
static void Log_Write_Event(uint8_t id){}
static void Log_Write_Optflow() {}
static void Log_Write_Nav_Tuning() {}
static void plane_Log_Write_Nav_Tuning() {}

static void Log_Write_Control_Tuning() {}
static void plane_Log_Write_Control_Tuning() {}
static void Log_Write_TECS_Tuning() {}

static void Log_Write_Performance() {}
static void plane_Log_Write_Performance() {}
static void Log_Write_Camera() {}
static void Log_Write_Error(uint8_t sub_system, uint8_t error_code) {}
static void Log_Write_Airspeed(void) {}
static void Log_Write_Baro(void) {}
static int8_t process_logs(uint8_t argc, const Menu::arg *argv) {
    return 0;
}

#endif // LOGGING_DISABLED
#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/Parameters.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *  ArduCopter parameter definitions
 *
 */

#define GSCALAR(v, name, def) { g.v.vtype, name, Parameters::k_param_ ## v, &g.v, {def_value : def} }
#define ASCALAR(v, name, def) { aparm.v.vtype, name, Parameters::k_param_ ## v, &aparm.v, {def_value : def} }	// Plane
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &g.v, {group_info : class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &v, {group_info : class::var_info} }
#define GOBJECTN(v, pname, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## pname, &v, {group_info : class::var_info} }

const AP_Param::Info var_info[] PROGMEM = {
    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version, "FORMAT_VERSION",   0),

    // @Param: SYSID_SW_TYPE
    // @DisplayName: Software Type
    // @Description: This is used by the ground station to recognise the software type (eg ArduPlane vs ArduCopter)
    // @User: Advanced
    GSCALAR(software_type,  "SYSID_SW_TYPE",   Parameters::k_software_type),

    // @Param: SYSID_THISMAV
    // @DisplayName: Mavlink version
    // @Description: Allows reconising the mavlink version
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(sysid_this_mav, "SYSID_THISMAV",   MAV_SYSTEM_ID),

    // @Param: SYSID_MYGCS
    // @DisplayName: My ground station number
    // @Description: The identifier of the ground station in the MAVLink protocol. Don't change this unless you also modify the ground station to match.
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(sysid_my_gcs,   "SYSID_MYGCS",     255),

    // @Param: SERIAL0_BAUD
    // @DisplayName: USB Console Baud Rate
    // @Description: The baud rate used on the USB console
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200
    // @User: Standard
    GSCALAR(serial0_baud,           "SERIAL0_BAUD",   SERIAL0_BAUD/1000),

    // @Param: SERIAL1_BAUD
    // @DisplayName: Telemetry Baud Rate
    // @Description: The baud rate used on the first telemetry port
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200
    // @User: Standard
    GSCALAR(serial1_baud,   "SERIAL1_BAUD",     SERIAL1_BAUD/1000),

#if MAVLINK_COMM_NUM_BUFFERS > 2
    // @Param: SERIAL2_BAUD
    // @DisplayName: Telemetry Baud Rate
    // @Description: The baud rate used on the seconds telemetry port
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200
    // @User: Standard
    GSCALAR(serial2_baud,   "SERIAL2_BAUD",     SERIAL2_BAUD/1000),
#endif

    // @Param: TELEM_DELAY
    // @DisplayName: Telemetry startup delay
    // @Description: The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up
    // @User: Advanced
    // @Units: seconds
    // @Range: 0 10
    // @Increment: 1
    GSCALAR(telem_delay,            "TELEM_DELAY",     0),

    // @Param: KFF_RDDRMIX
    // @DisplayName: Rudder Mix
    // @Description: The amount of rudder mix to apply during aileron movement 0 = 0 %, 1 = 100%
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(kff_rudder_mix,         "KFF_RDDRMIX",    RUDDER_MIX),

    // @Param: KFF_THR2PTCH
    // @DisplayName: Throttle to Pitch Mix
    // @Description: Throttle to pitch feed-forward gain.
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Advanced
    GSCALAR(kff_throttle_to_pitch,  "KFF_THR2PTCH",   0),

    // @Param: STICK_MIXING
    // @DisplayName: Stick Mixing
    // @Description: When enabled, this adds user stick input to the control surfaces in auto modes, allowing the user to have some degree of flight control without changing modes.  There are two types of stick mixing available. If you set STICK_MIXING to 1 then it will use "fly by wire" mixing, which controls the roll and pitch in the same way that the FBWA mode does. This is the safest option if you usually fly ArduPlane in FBWA or FBWB mode. If you set STICK_MIXING to 2 then it will enable direct mixing mode, which is what the PLANE_STABILIZE mode uses. That will allow for much more extreme maneuvers while in PLANE_AUTO mode.
    // @Values: 0:Disabled,1:FBWMixing,2:DirectMixing
    // @User: Advanced
    GSCALAR(stick_mixing,           "STICK_MIXING",   STICK_MIXING_FBW),

    // @Param: SKIP_GYRO_CAL
    // @DisplayName: Skip gyro calibration
    // @Description: When enabled this tells the APM to skip the normal gyroscope calibration at startup, and instead use the saved gyro calibration from the last flight. You should only enable this if you are careful to check that your aircraft has good attitude control before flying, as some boards may have significantly different gyro calibration between boots, especially if the temperature changes a lot. If gyro calibration is skipped then APM relies on using the gyro drift detection code to get the right gyro calibration in the few minutes after it boots. This option is mostly useful where the requirement to hold the plane still while it is booting is a significant problem.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    GSCALAR(skip_gyro_cal,           "SKIP_GYRO_CAL",   0),

    // @Param: AUTO_FBW_STEER
    // @DisplayName: Use FBWA steering in PLANE_AUTO
    // @Description: When enabled this option gives FBWA navigation and steering in PLANE_AUTO mode. This can be used to allow manual stabilised piloting with waypoint logic for triggering payloads. With this enabled the pilot has the same control over the plane as in FBWA mode, and the normal PLANE_AUTO navigation is completely disabled. This option is not recommended for normal use.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    GSCALAR(auto_fbw_steer,          "AUTO_FBW_STEER",   0),

    // @Param: TKOFF_THR_MINSPD
    // @DisplayName: Takeoff throttle min speed
    // @Description: Minimum GPS ground speed in m/s used by the speed check that un-suppresses throttle in auto-takeoff. This can be be used for catapult launches where you want the motor to engage only after the plane leaves the catapult, but it is preferable to use the TKOFF_THR_MINACC and TKOFF_THR_DELAY parameters for cvatapult launches due to the errors associated with GPS measurements. For hand launches with a pusher prop it is strongly advised that this parameter be set to a value no less than 4 m/s to provide additional protection against premature motor start. Note that the GPS velocity will lag the real velocity by about 0.5 seconds. The ground speed check is delayed by the TKOFF_THR_DELAY parameter.
    // @Units: m/s
    // @Range: 0 30
    // @Increment: 0.1
    // @User: User
    GSCALAR(takeoff_throttle_min_speed,     "TKOFF_THR_MINSPD",  0),

    // @Param: TKOFF_THR_MINACC
    // @DisplayName: Takeoff throttle min acceleration
    // @Description: Minimum forward acceleration in m/s/s before arming the ground speed check in auto-takeoff. This is meant to be used for hand launches. Setting this value to 0 disables the acceleration test which means the ground speed check will always be armed which could allow GPS velocity jumps to start the engine. For hand launches this should be set to 15.
    // @Units: m/s/s
    // @Range: 0 30
    // @Increment: 0.1
    // @User: User
    GSCALAR(takeoff_throttle_min_accel,     "TKOFF_THR_MINACC",  0),

    // @Param: TKOFF_THR_DELAY
    // @DisplayName: Takeoff throttle delay
    // @Description: This parameter sets the time delay (in 1/10ths of a second) that the ground speed check is delayed after the forward acceleration check controlled by TKOFF_THR_MINACC has passed. For hand launches with pusher propellers it is essential that this is set to a value of no less than 2 (0.2 seconds) to ensure that the aircraft is safely clear of the throwers arm before the motor can start. 
    // @Units: 0.1 seconds
    // @Range: 0 15
    // @Increment: 1
    // @User: User
    GSCALAR(takeoff_throttle_delay,     "TKOFF_THR_DELAY",  2),

    // @Param: LEVEL_ROLL_LIMIT
    // @DisplayName: Level flight roll limit
    // @Description: This controls the maximum bank angle in degrees during flight modes where level flight is desired, such as in the final stages of landing, and during auto takeoff. This should be a small angle (such as 5 degrees) to prevent a wing hitting the runway during takeoff or landing. Setting this to zero will completely disable heading hold on auto takeoff and final landing approach.
    // @Units: degrees
    // @Range: 0 45
    // @Increment: 1
    // @User: User
    GSCALAR(level_roll_limit,              "LEVEL_ROLL_LIMIT",   5),

    // @Param: LAND_PITCH_CD
    // @DisplayName: Landing Pitch
    // @Description: Used in autoland for planes without airspeed sensors in hundredths of a degree
    // @Units: centi-Degrees
    // @User: Advanced
    GSCALAR(land_pitch_cd,          "LAND_PITCH_CD",  0),

    // @Param: LAND_FLARE_ALT
    // @DisplayName: Landing flare altitude
    // @Description: Altitude in autoland at which to lock heading and flare to the LAND_PITCH_CD pitch
    // @Units: meters
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(land_flare_alt,          "LAND_FLARE_ALT",  3.0),

    // @Param: LAND_FLARE_SEC
    // @DisplayName: Landing flare time
    // @Description: Time before landing point at which to lock heading and flare to the LAND_PITCH_CD pitch
    // @Units: seconds
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(land_flare_sec,          "LAND_FLARE_SEC",  2.0),

	// @Param: NAV_CONTROLLER
	// @DisplayName: Navigation controller selection
	// @Description: Which navigation controller to enable. Currently the only navigation controller available is L1. From time to time other experimental conrtrollers will be added which are selected using this parameter.
	// @Values: 0:Default,1:L1Controller
	// @User: Standard
	GSCALAR(nav_controller,          "NAV_CONTROLLER",   AP_Navigation::CONTROLLER_L1),

    // @Param: ALT_MIX
    // @DisplayName: GPS to Baro Mix
    // @Description: The percent of mixing between GPS altitude and baro altitude. 0 = 100% gps, 1 = 100% baro. It is highly recommend that you not change this from the default of 1, as GPS altitude is notoriously unreliable. The only time I would recommend changing this is if you have a high altitude enabled GPS, and you are dropping a plane from a high altitude baloon many kilometers off the ground.
    // @Units: Percent
    // @Range: 0 1
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(altitude_mix,           "ALT_MIX",        ALTITUDE_MIX),

    // @Param: ALT_CTRL_ALG
    // @DisplayName: Altitude control algorithm
    // @Description: This sets what algorithm will be used for altitude control. The default is zero, which selects the most appropriate algorithm for your airframe. Currently the default is to use TECS (total energy control system). From time to time we will add other experimental altitude control algorithms which will be seleted using this parameter.
    // @Values: 0:Automatic
    // @User: Advanced
    GSCALAR(alt_control_algorithm, "ALT_CTRL_ALG",    ALT_CONTROL_DEFAULT),

    // @Param: ALT_OFFSET
    // @DisplayName: Altitude offset
    // @Description: This is added to the target altitude in automatic flight. It can be used to add a global altitude offset to a mission
    // @Units: Meters
    // @Range: -32767 32767
    // @Increment: 1
    // @User: Advanced
    GSCALAR(alt_offset, "ALT_OFFSET",                 0),

    // @Param: WP_RADIUS
    // @DisplayName: Waypoint Radius
    // @Description: Defines the distance from a waypoint that when crossed indicates the waypoint has been completed. To avoid the aircraft looping around the waypoint in case it misses by more than the WP_RADIUS an additional check is made to see if the aircraft has crossed a "finish line" passing through the waypoint and perpendicular to the flight path from the previous waypoint. If that finish line is crossed then the waypoint is considered complete.
    // @Units: Meters
    // @Range: 1 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(waypoint_radius,        "WP_RADIUS",      WP_RADIUS_DEFAULT),

    // @Param: WP_MAX_RADIUS
    // @DisplayName: Waypoint Maximum Radius
    // @Description: Sets the maximum distance to a waypoint for the waypoint to be considered complete. This overrides the "cross the finish line" logic that is normally used to consider a waypoint complete. For normal PLANE_AUTO behaviour this parameter should be set to zero. Using a non-zero value is only recommended when it is critical that the aircraft does approach within the given radius, and should loop around until it has done so. This can cause the aircraft to loop forever if its turn radius is greater than the maximum radius set.
    // @Units: Meters
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(waypoint_max_radius,        "WP_MAX_RADIUS",      0),

    // @Param: WP_LOITER_RAD
    // @DisplayName: Waypoint Loiter Radius
    // @Description: Defines the distance from the waypoint center, the plane will maintain during a loiter. If you set this value to a negative number then the default loiter direction will be counter-clockwise instead of clockwise.
    // @Units: Meters
    // @Range: 1 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(loiter_radius,          "WP_LOITER_RAD",  LOITER_RADIUS_DEFAULT),

#if AC_FENCE == ENABLED
    // @Param: FENCE_ACTION
    // @DisplayName: Action on geofence breach
    // @Description: What to do on fence breach. If this is set to 0 then no action is taken, and geofencing is disabled. If this is set to 1 then the plane will enter PLANE_GUIDED mode, with the target waypoint as the fence return point. If this is set to 2 then the fence breach is reported to the ground station, but no other action is taken. If set to 3 then the plane enters guided mode but the pilot retains manual throttle control.
    // @Values: 0:None,1:GuidedMode,2:ReportOnly,3:GuidedModeThrPass
    // @User: Standard
    GSCALAR(fence_action,           "FENCE_ACTION",   0),

    // @Param: FENCE_TOTAL
    // @DisplayName: Fence Total
    // @Description: Number of geofence points currently loaded
    // @User: Advanced
    GSCALAR(fence_total,            "FENCE_TOTAL",    0),

    // @Param: FENCE_CHANNEL
    // @DisplayName: Fence Channel
    // @Description: RC Channel to use to enable geofence. PWM input above 1750 enables the geofence
    // @User: Standard
    GSCALAR(fence_channel,          "FENCE_CHANNEL",  0),

    // @Param: FENCE_MINALT
    // @DisplayName: Fence Minimum Altitude
    // @Description: Minimum altitude allowed before geofence triggers
    // @Units: meters
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(fence_minalt,           "FENCE_MINALT",   0),

    // @Param: FENCE_MAXALT
    // @DisplayName: Fence Maximum Altitude
    // @Description: Maximum altitude allowed before geofence triggers
    // @Units: meters
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(fence_maxalt,           "FENCE_MAXALT",   0),

    // @Param: FENCE_RETALT
    // @DisplayName: Fence Return Altitude
    // @Description: Altitude the aircraft will transit to when a fence breach occurs.  If FENCE_RETALT is <= 0 then the midpoint between FENCE_MAXALT and FENCE_MINALT is used, unless FENCE_MAXALT < FENCE_MINALT.  If FENCE_MAXALT < FENCE_MINALT AND FENCE_RETALT is <= 0 then ALT_HOLD_RTL is the altitude used on a fence breach.
    // @Units: meters
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(fence_retalt,           "FENCE_RETALT",   0),
#endif

    // @Param: RALLY_TOTAL
    // @DisplayName: Rally Total
    // @Description: Number of rally points currently loaded
    // @User: Advanced
    GSCALAR(rally_total,            "RALLY_TOTAL",    0),

    // @Param: RALLY_LIMIT_KM
    // @DisplayName: Rally Limit
    // @Description: Maximum distance to rally point. If the closest rally point is more than this number of kilometers from the current position and the home location is closer than any of the rally points from the current position then do PLANE_RTL to home rather than to the closest rally point. This prevents a leftover rally point from a different airfield being used accidentally. If this is set to 0 then the closest rally point is always used.
    // @User: Advanced
    // @Units: kilometers
    // @Increment: 0.1
    GSCALAR(rally_limit_km,          "RALLY_LIMIT_KM",    5),

    // @Param: ARSPD_FBW_MIN
    // @DisplayName: Fly By Wire Minimum Airspeed
    // @Description: Airspeed corresponding to minimum throttle in auto throttle modes (FBWB, PLANE_CRUISE, PLANE_AUTO, PLANE_GUIDED, PLANE_LOITER, PLANE_CIRCLE and PLANE_RTL). This is a calibrated (apparent) airspeed.
    // @Units: m/s
    // @Range: 5 50
    // @Increment: 1
    // @User: Standard
    ASCALAR(airspeed_min, "ARSPD_FBW_MIN",  AIRSPEED_FBW_MIN),

    // @Param: ARSPD_FBW_MAX
    // @DisplayName: Fly By Wire Maximum Airspeed
    // @Description: Airspeed corresponding to maximum throttle in auto throttle modes (FBWB, PLANE_CRUISE, PLANE_AUTO, PLANE_GUIDED, PLANE_LOITER, PLANE_CIRCLE and PLANE_RTL). This is a calibrated (apparent) airspeed.
    // @Units: m/s
    // @Range: 5 50
    // @Increment: 1
    // @User: Standard
    ASCALAR(airspeed_max, "ARSPD_FBW_MAX",  AIRSPEED_FBW_MAX),

    // @Param: FBWB_ELEV_REV
    // @DisplayName: Fly By Wire elevator reverse
    // @Description: Reverse sense of elevator in FBWB and PLANE_CRUISE modes. When set to 0 up elevator (pulling back on the stick) means to lower altitude. When set to 1, up elevator means to raise altitude.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(flybywire_elev_reverse, "FBWB_ELEV_REV",  0),

    // @Param: FBWB_CLIMB_RATE
    // @DisplayName: Fly By Wire B altitude change rate
    // @Description: This sets the rate in m/s at which FBWB and PLANE_CRUISE modes will change its target altitude for full elevator deflection. Note that the actual climb rate of the aircraft can be lower than this, depending on your airspeed and throttle control settings. If you have this parameter set to the default value of 2.0, then holding the elevator at maximum deflection for 10 seconds would change the target altitude by 20 meters.
    // @Range: 1-10
	// @Increment: 0.1
    // @User: Standard
    GSCALAR(flybywire_climb_rate, "FBWB_CLIMB_RATE",  2.0f),

    // @Param: PLTHR_MIN
    // @DisplayName: Minimum Throttle
    // @Description: The minimum throttle setting to which the autopilot will apply.
    // @Units: Percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(plthr_min,           "PLTHR_MIN",        PLTHR_MIN),

    // @Param: PLTHR_MAX
    // @DisplayName: Maximum Throttle
    // @Description: The maximum throttle setting to which the autopilot will apply.
    // @Units: Percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(plthr_max,           "PLTHR_MAX",        PLTHR_MAX),

    // @Param: THR_SLEWRATE
    // @DisplayName: Throttle slew rate
    // @Description: maximum percentage change in throttle per second. A setting of 10 means to not change the throttle by more than 10% of the full throttle range in one second.
    // @Units: Percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(throttle_slewrate,      "THR_SLEWRATE",   100),

    // @Param: THR_SUPP_MAN
    // @DisplayName: Throttle suppress manual passthru
    // @Description: When throttle is supressed in auto mode it is normally forced to zero. If you enable this option, then while suppressed it will be manual throttle. This is useful on petrol engines to hold the idle throttle manually while waiting for takeoff
	// @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    GSCALAR(throttle_suppress_manual,"THR_SUPP_MAN",   0),

    // @Param: THR_PASS_STAB
    // @DisplayName: Throttle passthru in stabilize
    // @Description: If this is set then when in PLANE_STABILIZE, FBWA or PLANE_ACRO modes the throttle is a direct passthru from the transmitter. This means the PL_THR_MIN and PL_THR_MAX settings are not used in these modes. This is useful for petrol engines where you setup a throttle cut switch that suppresses the throttle below the normal minimum.
	// @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    GSCALAR(throttle_passthru_stabilize,"THR_PASS_STAB",   0),

    // @Param: THR_FAILSAFE
    // @DisplayName: Throttle Failsafe Enable
    // @Description: The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(throttle_fs_enabled,    "THR_FAILSAFE",   THROTTLE_FAILSAFE),


    // @Param: THR_FS_VALUE
    // @DisplayName: Throttle Failsafe Value
    // @Description: The PWM level on channel 3 below which throttle sailsafe triggers
    // @Range: 925 1100
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_fs_value,      "THR_FS_VALUE",   THROTTLE_FS_VALUE),

    // @Param: PLTRIM_THROTTLE
    // @DisplayName: Throttle cruise percentage
    // @Description: The target percentage of throttle to apply for normal flight
    // @Units: Percent
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(plthr_cruise,        "PLTRIM_THROTTLE",  PLTHR_CRUISE),

    // @Param: THROTTLE_NUDGE
    // @DisplayName: Throttle nudge enable
    // @Description: When enabled, this uses the throttle input in auto-throttle modes to 'nudge' the throttle or airspeed to higher or lower values. When you have an airspeed sensor the nudge affects the target airspeed, so that throttle inputs above 50% will increase the target airspeed from TRIM_ARSPD_CM up to a maximum of ARSPD_FBW_MAX. When no airspeed sensor is enabled the throttle nudge will push up the target throttle for throttle inputs above 50%.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    // @User: Standard
    GSCALAR(throttle_nudge,         "THROTTLE_NUDGE",  1),

    // @Param: FS_SHORT_ACTN
    // @DisplayName: Short failsafe action
    // @Description: The action to take on a short (FS_SHORT_TIMEOUT) failsafe event in PLANE_AUTO, PLANE_GUIDED or PLANE_LOITER modes. A short failsafe event in stabilization modes will always cause an immediate change to PLANE_CIRCLE mode. In PLANE_AUTO mode you can choose whether it will enter PLANE_CIRCLE mode or continue with the mission. If FS_SHORT_ACTN is 0 then it will continue with the mission, if it is 1 then it will enter PLANE_CIRCLE mode, and then enter PLANE_RTL if the failsafe condition persists for FS_LONG_TIMEOUT seconds. If it is set to 2 then the plane will enter FBWA mode with zero throttle and level attitude to glide in.
    // @Values: 0:Continue,1:Circle/ReturnToLaunch,2:Glide
    // @User: Standard
    GSCALAR(short_fs_action,        "FS_SHORT_ACTN",  SHORT_FAILSAFE_ACTION),

    // @Param: FS_SHORT_TIMEOUT
    // @DisplayName: Short failsafe timeout
    // @Description: The time in seconds that a failsafe condition has to persist before a short failsafe event will occor. This defaults to 1.5 seconds
    // @Units: seconds
    // @Range: 1 100
    // @Increment: 0.5
    // @User: Standard
    GSCALAR(short_fs_timeout,        "FS_SHORT_TIMEOUT", 1.5f),

    // @Param: FS_LONG_ACTN
    // @DisplayName: Long failsafe action
    // @Description: The action to take on a long (FS_LONG_TIMEOUT seconds) failsafe event in PLANE_AUTO, PLANE_GUIDED or PLANE_LOITER modes. A long failsafe event in stabilization modes will always cause an PLANE_RTL (ReturnToLaunch). In PLANE_AUTO modes you can choose whether it will PLANE_RTL or continue with the mission. If FS_LONG_ACTN is 0 then it will continue with the mission, if it is 1 then it will enter PLANE_RTL mode. Note that if FS_SHORT_ACTN is 1, then the aircraft will enter PLANE_CIRCLE mode after FS_SHORT_TIMEOUT seconds of failsafe, and will always enter PLANE_RTL after FS_LONG_TIMEOUT seconds of failsafe, regardless of the FS_LONG_ACTN setting. If FS_LONG_ACTN is set to 2 then instead of using PLANE_RTL it will enter a FBWA glide with zero throttle.
    // @Values: 0:Continue,1:ReturnToLaunch,2:Glide
    // @User: Standard
    GSCALAR(long_fs_action,         "FS_LONG_ACTN",   LONG_FAILSAFE_ACTION),

    // @Param: FS_LONG_TIMEOUT
    // @DisplayName: Long failsafe timeout
    // @Description: The time in seconds that a failsafe condition has to persist before a long failsafe event will occor. This defaults to 20 seconds
    // @Units: seconds
    // @Range: 1 300
    // @Increment: 0.5
    // @User: Standard
    GSCALAR(long_fs_timeout,        "FS_LONG_TIMEOUT", 20),
	
    // @Param: RTL_ALT
    // @DisplayName: RTL Altitude
    // @Description: The minimum altitude the model will move to before Returning to Launch.  Set to zero to return at current altitude.
    // @Units: Centimeters
    // @Range: 0 8000
    // @Increment: 1
    // @User: Standard
    GSCALAR(rtl_altitude,   "RTL_ALT",     RTL_ALT),

    // @Param: SONAR_ENABLE
    // @DisplayName: Sonar enable/disable
    // @Description: Setting this to Enabled(1) will enable the sonar. Setting this to Disabled(0) will disable the sonar
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(sonar_enabled,  "SONAR_ENABLE",     DISABLED),

    // @Param: SONAR_TYPE
    // @DisplayName: Sonar type
    // @Description: Used to adjust scaling to match the sonar used (only Maxbotix sonars are supported at this time)
    // @Values: 0:XL-EZ0 / XL-EZ4,1:LV-EZ0,2:XLL-EZ0,3:HRLV
    // @User: Standard
    GSCALAR(sonar_type,     "SONAR_TYPE",           AP_RANGEFINDER_MAXSONARXL),

    // @Param: SONAR_GAIN
    // @DisplayName: Sonar gain
    // @Description: Used to adjust the speed with which the target altitude is changed when objects are sensed below the copter
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(sonar_gain,     "SONAR_GAIN",           SONAR_GAIN_DEFAULT),

    // @Param: FS_BATT_ENABLE
    // @DisplayName: Battery Failsafe Enable
    // @Description: Controls whether failsafe will be invoked when battery voltage or current runs low
    // @Values: 0:Disabled,1:Land,2:RTL
    // @User: Standard
    GSCALAR(failsafe_battery_enabled, "FS_BATT_ENABLE", FS_BATT_DISABLED),


    // @Param: FS_BATT_VOLTAGE
    // @DisplayName: Failsafe battery voltage
    // @Description: Battery voltage to trigger failsafe. Set to 0 to disable battery voltage failsafe. If the battery voltage drops below this voltage then the copter will RTL
    // @Units: Volts
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(fs_batt_voltage,        "FS_BATT_VOLTAGE", FS_BATT_VOLTAGE_DEFAULT),

    // @Param: FS_BATT_MAH
    // @DisplayName: Failsafe battery milliAmpHours
    // @Description: Battery capacity remaining to trigger failsafe. Set to 0 to disable battery remaining failsafe. If the battery remaining drops below this level then the copter will RTL
    // @Units: mAh
    // @Increment: 50
    // @User: Standard
    GSCALAR(fs_batt_mah,            "FS_BATT_MAH", FS_BATT_MAH_DEFAULT),

    // @Param: FS_GCS_ENABL
    // @DisplayName: GCS failsafe enable
    // @Description: Enable ground control station telemetry failsafe. Failsafe will trigger after FS_LONG_TIMEOUT seconds of no MAVLink heartbeat messages. There are two possible enabled settings. Seeing FS_GCS_ENABL to 1 means that GCS failsafe will be triggered when the aircraft has not received a MAVLink HEARTBEAT message. Setting FS_GCS_ENABL to 2 means that GCS failsafe will be triggerded on either a loss of HEARTBEAT messages, or a RADIO_STATUS message from a MAVLink enabled 3DR radio indicating that the ground station is not receiving status updates from the aircraft, which is indicated by the RADIO_STATUS.remrssi field being zero (this may happen if you have a one way link due to asymmetric noise on the ground station and aircraft radios). WARNING: Enabling this option opens up the possibility of your plane going into failsafe mode and running the motor on the ground it it loses contact with your ground station. If this option is enabled on an electric plane then you should enable ARMING_REQUIRED. 
    // @Values: 0:Disabled,1:Heartbeat,2:HeartbeatAndREMRSSI
    // @User: Standard
    GSCALAR(gcs_heartbeat_fs_enabled, "FS_GCS_ENABL", GCS_FAILSAFE_OFF),

    // @Param: FS_GPS_ENABLE
    // @DisplayName: GPS Failsafe Enable
    // @Description: Controls what action will be taken if GPS signal is lost for at least 5 seconds
    // @Values: 0:Disabled,1:Land,2:AltHold,3:Land even from Stabilize
    // @User: Standard
    GSCALAR(failsafe_gps_enabled, "FS_GPS_ENABLE", FS_GPS_LAND),

    // @Param: FS_GCS_ENABLE
    // @DisplayName: Ground Station Failsafe Enable
    // @Description: Controls whether failsafe will be invoked (and what action to take) when connection with Ground station is lost for at least 5 seconds
    // @Values: 0:Disabled,1:Enabled always RTL,2:Enabled Continue with Mission in Auto Mode
    // @User: Standard
    GSCALAR(failsafe_gcs, "FS_GCS_ENABLE", FS_GCS_DISABLED),

    // @Param: GPS_HDOP_GOOD
    // @DisplayName: GPS Hdop Good
    // @Description: GPS Hdop value at or below this value represent a good position.  Used for pre-arm checks
    // @Range: 100 900
    // @User: Advanced
    GSCALAR(gps_hdop_good, "GPS_HDOP_GOOD", GPS_HDOP_GOOD_DEFAULT),

    // @Param: MAG_ENABLE
    // @DisplayName: Compass enable/disable
    // @Description: Setting this to Enabled(1) will enable the compass. Setting this to Disabled(0) will disable the compass
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(compass_enabled,        "MAG_ENABLE",   MAGNETOMETER),

    // @Param: FLOW_ENABLE
    // @DisplayName: Optical Flow enable/disable
    // @Description: Setting this to Enabled(1) will enable optical flow. Setting this to Disabled(0) will disable optical flow
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(optflow_enabled,        "FLOW_ENABLE",  DISABLED),

    // @Param: SUPER_SIMPLE
    // @DisplayName: Super Simple Mode
    // @Description: Bitmask to enable Super Simple mode for some flight modes. Setting this to Disabled(0) will disable Super Simple Mode
    // @Values: 0:Disabled,1:Mode1,2:Mode2,3:Mode1+2,4:Mode3,5:Mode1+3,6:Mode2+3,7:Mode1+2+3,8:Mode4,9:Mode1+4,10:Mode2+4,11:Mode1+2+4,12:Mode3+4,13:Mode1+3+4,14:Mode2+3+4,15:Mode1+2+3+4,16:Mode5,17:Mode1+5,18:Mode2+5,19:Mode1+2+5,20:Mode3+5,21:Mode1+3+5,22:Mode2+3+5,23:Mode1+2+3+5,24:Mode4+5,25:Mode1+4+5,26:Mode2+4+5,27:Mode1+2+4+5,28:Mode3+4+5,29:Mode1+3+4+5,30:Mode2+3+4+5,31:Mode1+2+3+4+5,32:Mode6,33:Mode1+6,34:Mode2+6,35:Mode1+2+6,36:Mode3+6,37:Mode1+3+6,38:Mode2+3+6,39:Mode1+2+3+6,40:Mode4+6,41:Mode1+4+6,42:Mode2+4+6,43:Mode1+2+4+6,44:Mode3+4+6,45:Mode1+3+4+6,46:Mode2+3+4+6,47:Mode1+2+3+4+6,48:Mode5+6,49:Mode1+5+6,50:Mode2+5+6,51:Mode1+2+5+6,52:Mode3+5+6,53:Mode1+3+5+6,54:Mode2+3+5+6,55:Mode1+2+3+5+6,56:Mode4+5+6,57:Mode1+4+5+6,58:Mode2+4+5+6,59:Mode1+2+4+5+6,60:Mode3+4+5+6,61:Mode1+3+4+5+6,62:Mode2+3+4+5+6,63:Mode1+2+3+4+5+6
    // @User: Standard
    GSCALAR(super_simple,   "SUPER_SIMPLE",     SUPER_SIMPLE),

    // @Param: RTL_ALT_FINAL
    // @DisplayName: RTL Final Altitude
    // @Description: This is the altitude the vehicle will move to as the final stage of Returning to Launch or after completing a mission.  Set to zero to land.
    // @Units: Centimeters
    // @Range: -1 1000
    // @Increment: 1
    // @User: Standard
    GSCALAR(rtl_alt_final,  "RTL_ALT_FINAL", RTL_ALT_FINAL),

    // @Param: RSSI_PIN
    // @DisplayName: Receiver RSSI sensing pin
    // @Description: This selects an analog pin for the receiver RSSI voltage. It assumes the voltage is 5V for max rssi, 0V for minimum
    // @Values: -1:Disabled, 0:A0, 1:A1, 13:A13, 103:Pixhawk
    // @User: Standard
    GSCALAR(rssi_pin,            "RSSI_PIN",         -1),

    // @Param: RSSI_RANGE
    // @DisplayName: Receiver RSSI voltage range
    // @Description: Receiver RSSI voltage range
    // @Units: Volt
    // @Values: 3.3:3.3V, 5.0:5V
    // @User: Standard
    GSCALAR(rssi_range,          "RSSI_RANGE",         5.0),

    // @Param: WP_YAW_BEHAVIOR
    // @DisplayName: Yaw behaviour during missions
    // @Description: Determines how the autopilot controls the yaw during missions and RTL
    // @Values: 0:Never change yaw, 1:Face next waypoint, 2:Face next waypoint except RTL, 3:Face along GPS course
    // @User: Standard
    GSCALAR(wp_yaw_behavior,  "WP_YAW_BEHAVIOR",    WP_YAW_BEHAVIOR_DEFAULT),

    // @Param: WP_TOTAL
    // @DisplayName: Number of loaded mission items
    // @Description: The number of mission mission items that has been loaded by the ground station. Do not change this manually.
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(command_total,          "WP_TOTAL",      0),

    // @Param: WP_INDEX
    // @DisplayName: Current mission command index
    // @Description: The index of the currently running mission item. Do not change this manually.
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(command_index,          "WP_INDEX",      0),
	
    // @Param: INVERTEDFLT_CH
    // @DisplayName: Inverted flight channel
    // @Description: A RC input channel number to enable inverted flight. If this is non-zero then the APM will monitor the correcponding RC input channel and will enable inverted flight when the channel goes above 1750.
    // @Values: 0:Disabled,1:Channel1,2:Channel2,3:Channel3,4:Channel4,5:Channel5,6:Channel6,7:Channel7,8:Channel8
    // @User: Standard
    GSCALAR(inverted_flight_ch,     "INVERTEDFLT_CH", 0),

#if HIL_MODE != HIL_MODE_DISABLED
    // @Param: HIL_SERVOS
    // @DisplayName: HIL Servos enable
    // @Description: This controls whether real servo controls are used in HIL mode. If you enable this then the APM will control the real servos in HIL mode. If disabled it will report servo values, but will not output to the real servos. Be careful that your motor and propeller are not connected if you enable this option.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    GSCALAR(hil_servos,            "HIL_SERVOS",      0),

    // @Param: HIL_ERR_LIMIT
    // @DisplayName: Limit of error in HIL attitude before reset
    // @Description: This controls the maximum error in degrees on any axis before HIL will reset the DCM attitude to match the HIL_STATE attitude. This limit will prevent poor timing on HIL from causing a major attitude error. If the value is zero then no limit applies.
    // @Units: degrees
    // @Range: 0 90
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(hil_err_limit,         "HIL_ERR_LIMIT",   5),
#endif

    // @Param: CIRCLE_RADIUS
    // @DisplayName: Circle radius
    // @Description: Defines the radius of the circle the vehicle will fly when in Circle flight mode
    // @Units: Meters
    // @Range: 1 127
    // @Increment: 1
    // @User: Standard
    GSCALAR(circle_radius,  "CIRCLE_RADIUS",    CIRCLE_RADIUS),

    // @Param: CIRCLE_RATE
    // @DisplayName: Circle rate
    // @Description: Circle mode's turn rate in degrees / second.  Positive to turn clockwise, negative for counter clockwise
    // @Units: deg/s
    // @Range: -90 90
    // @Increment: 1
    // @User: Standard
    GSCALAR(circle_rate,  "CIRCLE_RATE",        CIRCLE_RATE),

    // @Param: RTL_LOIT_TIME
    // @DisplayName: RTL loiter time
    // @Description: Time (in milliseconds) to loiter above home before begining final descent
    // @Units: ms
    // @Range: 0 60000
    // @Increment: 1000
    // @User: Standard
    GSCALAR(rtl_loiter_time,      "RTL_LOIT_TIME",    RTL_LOITER_TIME),

    // @Param: LAND_SPEED
    // @DisplayName: Land speed
    // @Description: The descent speed for the final stage of landing in cm/s
    // @Units: cm/s
    // @Range: 30 200
    // @Increment: 10
    // @User: Standard
    GSCALAR(land_speed,             "LAND_SPEED",   LAND_SPEED),

    // @Param: PILOT_VELZ_MAX
    // @DisplayName: Pilot maximum vertical speed
    // @Description: The maximum vertical velocity the pilot may request in cm/s
    // @Units: Centimeters/Second
    // @Range: 10 500
    // @Increment: 10
    // @User: Standard
    GSCALAR(pilot_velocity_z_max,     "PILOT_VELZ_MAX",   PILOT_VELZ_MAX),

    // @Param: THR_MIN
    // @DisplayName: Throttle Minimum
    // @Description: The minimum throttle that will be sent to the motors to keep them spinning
    // @Units: Percent*10
    // @Range: 0 300
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_min,   "THR_MIN",          THR_MIN_DEFAULT),

    // @Param: THR_MAX
    // @DisplayName: Throttle Maximum
    // @Description: The maximum throttle that will be sent to the motors.  This should normally be left as 1000.
    // @Units: Percent*10
    // @Range: 800 1000
    // @Increment: 1
    // @User: Advanced
    GSCALAR(throttle_max,   "THR_MAX",          THR_MAX_DEFAULT),

    // @Param: FS_THR_ENABLE
    // @DisplayName: Throttle Failsafe Enable
    // @Description: The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
    // @Values: 0:Disabled,1:Enabled always RTL,2:Enabled Continue with Mission in Auto Mode,3:Enabled always LAND
    // @User: Standard
    GSCALAR(failsafe_throttle,  "FS_THR_ENABLE",   FS_THR_DISABLED),

    // @Param: FS_THR_VALUE
    // @DisplayName: Throttle Failsafe Value
    // @Description: The PWM level on channel 3 below which throttle sailsafe triggers
    // @Range: 925 1100
    // @Units: pwm
    // @Increment: 1
    // @User: Standard
    GSCALAR(failsafe_throttle_value, "FS_THR_VALUE",      FS_THR_VALUE_DEFAULT),

    // @Param: TRIM_THROTTLE
    // @DisplayName: Throttle Trim
    // @Description: The autopilot's estimate of the throttle required to maintain a level hover.  Calculated automatically from the pilot's throttle input while in stabilize mode
    // @Range: 0 1000
    // @Units: Percent*10
    // @User: Advanced
    GSCALAR(throttle_cruise,        "TRIM_THROTTLE",    THROTTLE_CRUISE),

    // @Param: THR_MID
    // @DisplayName: Throttle Mid Position
    // @Description: The throttle output (0 ~ 1000) when throttle stick is in mid position.  Used to scale the manual throttle so that the mid throttle stick position is close to the throttle required to hover
    // @User: Standard
    // @Range: 300 700
    // @Units: Percent*10
    // @Increment: 1
    GSCALAR(throttle_mid,        "THR_MID",    THR_MID_DEFAULT),

    // @Param: FLTMODE_CH
    // @DisplayName: Flightmode channel
    // @Description: RC Channel to use for flight mode control
    // @User: Advanced
    GSCALAR(flight_mode_channel,    "FLTMODE_CH",     FLIGHT_MODE_CHANNEL),

    // @Param: FLTMODE1
    // @DisplayName: Flight Mode 1
    // @Description: Flight mode when Channel 5 pwm is <= 1230
    // @Values: 0:Stabilize/Manual,1:Acro/Circle,2:AltHold/Stabilize,3:Auto/Training,4:Guided/Acro,5:Loiter/FBWA,6:RTL/FBWB,7:Circle/Cruise,8:Position,9:Land,10:OF_Loiter/Auto,11:Drift/RTL,12:PlaneLoiter,13:Sport,15:PlaneGuided
    // @User: Standard
    GSCALAR(flight_mode1, "FLTMODE1",               FLIGHT_MODE_1),

    // @Param: FLTMODE2
    // @DisplayName: Flight Mode 2
    // @Description: Flight mode when Channel 5 pwm is >1230, <= 1360
    // @Values: 0:Stabilize/Manual,1:Acro/Circle,2:AltHold/Stabilize,3:Auto/Training,4:Guided/Acro,5:Loiter/FBWA,6:RTL/FBWB,7:Circle/Cruise,8:Position,9:Land,10:OF_Loiter/Auto,11:Drift/RTL,12:PlaneLoiter,13:Sport,15:PlaneGuided
    // @User: Standard
    GSCALAR(flight_mode2, "FLTMODE2",               FLIGHT_MODE_2),

    // @Param: FLTMODE3
    // @DisplayName: Flight Mode 3
    // @Description: Flight mode when Channel 5 pwm is >1360, <= 1490
    // @Values: 0:Stabilize/Manual,1:Acro/Circle,2:AltHold/Stabilize,3:Auto/Training,4:Guided/Acro,5:Loiter/FBWA,6:RTL/FBWB,7:Circle/Cruise,8:Position,9:Land,10:OF_Loiter/Auto,11:Drift/RTL,12:PlaneLoiter,13:Sport,15:PlaneGuided
    // @User: Standard
    GSCALAR(flight_mode3, "FLTMODE3",               FLIGHT_MODE_3),

    // @Param: FLTMODE4
    // @DisplayName: Flight Mode 4
    // @Description: Flight mode when Channel 5 pwm is >1490, <= 1620
    // @Values: 0:Stabilize/Manual,1:Acro/Circle,2:AltHold/Stabilize,3:Auto/Training,4:Guided/Acro,5:Loiter/FBWA,6:RTL/FBWB,7:Circle/Cruise,8:Position,9:Land,10:OF_Loiter/Auto,11:Drift/RTL,12:PlaneLoiter,13:Sport,15:PlaneGuided
    // @User: Standard
    GSCALAR(flight_mode4, "FLTMODE4",               FLIGHT_MODE_4),

    // @Param: FLTMODE5
    // @DisplayName: Flight Mode 5
    // @Description: Flight mode when Channel 5 pwm is >1620, <= 1749
    // @Values: 0:Stabilize/Manual,1:Acro/Circle,2:AltHold/Stabilize,3:Auto/Training,4:Guided/Acro,5:Loiter/FBWA,6:RTL/FBWB,7:Circle/Cruise,8:Position,9:Land,10:OF_Loiter/Auto,11:Drift/RTL,12:PlaneLoiter,13:Sport,15:PlaneGuided
    // @User: Standard
    GSCALAR(flight_mode5, "FLTMODE5",               FLIGHT_MODE_5),

    // @Param: FLTMODE6
    // @DisplayName: Flight Mode 6
    // @Description: Flight mode when Channel 5 pwm is >=1750
    // @Values: 0:Stabilize/Manual,1:Acro/Circle,2:AltHold/Stabilize,3:Auto/Training,4:Guided/Acro,5:Loiter/FBWA,6:RTL/FBWB,7:Circle/Cruise,8:Position,9:Land,10:OF_Loiter/Auto,11:Drift/RTL,12:PlaneLoiter,13:Sport,15:PlaneGuided
    // @User: Standard
    GSCALAR(flight_mode6, "FLTMODE6",               FLIGHT_MODE_6),

    // @Param: PLFLTMODE1
    // @DisplayName: Flight Mode 1
    // @Description: Flight mode when Channel 5 pwm is <= 1230
    // @Values: 20:Manual,21:CIRCLE,22:STABILIZE,23:TRAINING,24:ACRO,25:FBWA,26:FBWB,27:CRUISE,30:Auto,31:RTL,32:Loiter,35:Guided
    // @User: Standard
    GSCALAR(plane_flight_mode1, "PLFLTMODE1",               PLANE_FLIGHT_MODE_1),

    // @Param: PLFLTMODE2
    // @DisplayName: Flight Mode 2
    // @Description: Flight mode when Channel 5 pwm is >1230, <= 1360
    // @Values: 20:Manual,21:CIRCLE,22:STABILIZE,23:TRAINING,24:ACRO,25:FBWA,26:FBWB,27:CRUISE,30:Auto,31:RTL,32:Loiter,35:Guided
    // @User: Standard
    GSCALAR(plane_flight_mode2, "PLFLTMODE2",               PLANE_FLIGHT_MODE_2),

    // @Param: PLFLTMODE3
    // @DisplayName: Flight Mode 3
    // @Description: Flight mode when Channel 5 pwm is >1360, <= 1490
    // @Values: 20:Manual,21:CIRCLE,22:STABILIZE,23:TRAINING,24:ACRO,25:FBWA,26:FBWB,27:CRUISE,30:Auto,31:RTL,32:Loiter,35:Guided
    // @User: Standard
    GSCALAR(plane_flight_mode3, "PLFLTMODE3",               PLANE_FLIGHT_MODE_3),

    // @Param: PLFLTMODE4
    // @DisplayName: Flight Mode 4
    // @Description: Flight mode when Channel 5 pwm is >1490, <= 1620
    // @Values: 20:Manual,21:CIRCLE,22:STABILIZE,23:TRAINING,24:ACRO,25:FBWA,26:FBWB,27:CRUISE,30:Auto,31:RTL,32:Loiter,35:Guided
    // @User: Standard
    GSCALAR(plane_flight_mode4, "PLFLTMODE4",               PLANE_FLIGHT_MODE_4),

    // @Param: PLFLTMODE5
    // @DisplayName: Flight Mode 5
    // @Description: Flight mode when Channel 5 pwm is >1620, <= 1749
    // @Values: 20:Manual,21:CIRCLE,22:STABILIZE,23:TRAINING,24:ACRO,25:FBWA,26:FBWB,27:CRUISE,30:Auto,31:RTL,32:Loiter,35:Guided
    // @User: Standard
    GSCALAR(plane_flight_mode5, "PLFLTMODE5",               PLANE_FLIGHT_MODE_5),

    // @Param: PLFLTMODE6
    // @DisplayName: Flight Mode 6
    // @Description: Flight mode when Channel 5 pwm is >=1750
    // @Values: 20:Manual,21:CIRCLE,22:STABILIZE,23:TRAINING,24:ACRO,25:FBWA,26:FBWB,27:CRUISE,30:Auto,31:RTL,32:Loiter,35:Guided
    // @User: Standard
    GSCALAR(plane_flight_mode6, "PLFLTMODE6",               PLANE_FLIGHT_MODE_6),

    // @Param: hybridservo_copter_value
    // @DisplayName: hybridservo_copter_value
    // @Description: 
    // @Values: 800-2000
    // @User: Standard
    GSCALAR(hybridservo_copter, "HYBCOPVAL",               HYBRIDSERVO_COPTER),

    // @Param: hybridservo_transition_value
    // @DisplayName: hybridservo_transition_value
    // @Description: 
    // @Values: 800-2000
    // @User: Standard
    GSCALAR(hybridservo_transition, "HYBTRANSVAL",               HYBRIDSERVO_TRANSITION),
	
    // @Param: hybridservo_plane_value
    // @DisplayName: hybridservo_plane_value
    // @Description: 
    // @Values: 800-2000
    // @User: Standard
    GSCALAR(hybridservo_plane, "HYBPLANEVAL",               HYBRIDSERVO_PLANE),

	// @Param: Hybrid transitionspeed
    // @DisplayName: Hybrid transitionspeed
    // @Description: 
    // @Values:
    // @User: Standard
    GSCALAR(transitionspeed, "TRANSSPEED",               TRANSITIONSPEED),
	
    // @Param: LIM_ROLL_CD
    // @DisplayName: Maximum Bank Angle
    // @Description: The maximum commanded bank angle in either direction
    // @Units: centi-Degrees
    // @Range: 0 9000
    // @Increment: 1
    // @User: Standard
    GSCALAR(roll_limit_cd,          "LIM_ROLL_CD",    HEAD_MAX_CENTIDEGREE),

    // @Param: LIM_PITCH_MAX
    // @DisplayName: Maximum Pitch Angle
    // @Description: The maximum commanded pitch up angle
    // @Units: centi-Degrees
    // @Range: 0 9000
    // @Increment: 1
    // @User: Standard
    ASCALAR(pitch_limit_max_cd,     "LIM_PITCH_MAX",  PITCH_MAX_CENTIDEGREE),

    // @Param: LIM_PITCH_MIN
    // @DisplayName: Minimum Pitch Angle
    // @Description: The minimum commanded pitch down angle
    // @Units: centi-Degrees
    // @Range: -9000 0
    // @Increment: 1
    // @User: Standard
    ASCALAR(pitch_limit_min_cd,     "LIM_PITCH_MIN",  PITCH_MIN_CENTIDEGREE),

    // @Param: ACRO_ROLL_RATEP
    // @DisplayName: PLANE_ACRO mode roll rate
    // @Description: The maximum roll rate at full stick deflection in PLANE_ACRO mode
    // @Units: degrees/second
    // @Range: 10 500
    // @Increment: 1
    // @User: Standard
    GSCALAR(acro_roll_ratep,          "ACRO_ROLL_RATEP",    180),

    // @Param: ACRO_PITCH_RATEP
    // @DisplayName: PLANE_ACRO mode pitch rate
    // @Description: The maximum pitch rate at full stick deflection in PLANE_ACRO mode
    // @Units: degrees/second
    // @Range: 10 500
    // @Increment: 1
    // @User: Standard
    GSCALAR(acro_pitch_ratep,          "ACRO_PITCH_RATEP",  180),

    // @Param: ACRO_LOCKING
    // @DisplayName: PLANE_ACRO mode attitude locking
    // @Description: Enable attitude locking when sticks are released
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(acro_locking,             "ACRO_LOCKING",     0),

    // @Param: GROUND_STEER_ALT
    // @DisplayName: Ground steer altitude
    // @Description: Altitude at which to use the ground steering controller on the rudder. If non-zero then the STEER2SRV controller will be used to control the rudder for altitudes within this limit of the home altitude.
    // @Units: Meters
    // @Range: -100 100
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(ground_steer_alt,         "GROUND_STEER_ALT",   0),

    // @Param: GROUND_STEER_DPS
    // @DisplayName: Ground steer rate
    // @Description: Ground steering rate in degrees per second for full rudder stick deflection
    // @Units: Meters
    // @Range: 10 360
    // @Increment: 1
    // @User: Advanced
    GSCALAR(ground_steer_dps,         "GROUND_STEER_DPS",  90),

    // @Param: TRIM_AUTO
    // @DisplayName: Automatic trim adjustment
    // @Description: Set RC trim PWM levels to current levels when switching away from manual mode. When this option is enabled and you change from PLANE_MANUAL to any other mode then the APM will take the current position of the control sticks as the trim values for aileron, elevator and rudder. It will use those to set RC1_TRIM, RC2_TRIM and RC4_TRIM. This option is disabled by default as if a pilot is not aware of this option and changes from PLANE_MANUAL to another mode while control inputs are not centered then the trim could be changed to a dangerously bad value. You can enable this option to assist with trimming your plane, by enabling it before takeoff then switching briefly to PLANE_MANUAL in flight, and seeing how the plane reacts. You can then switch back to FBWA, trim the surfaces then again test PLANE_MANUAL mode. Each time you switch from PLANE_MANUAL the APM will take your control inputs as the new trim. After you have good trim on your aircraft you can disable TRIM_AUTO for future flights.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(auto_trim,              "TRIM_AUTO",      AUTO_TRIM),

    // @Param: ELEVON_MIXING
    // @DisplayName: Elevon mixing
    // @Description: Enable elevon mixing  on both input and output. To enable just output mixing see the ELEVON_OUTPUT option.
    // @Values: 0:Disabled,1:Enabled
    // @User: User
    GSCALAR(mix_mode,               "ELEVON_MIXING",  ELEVON_MIXING),

    // @Param: ELEVON_REVERSE
    // @DisplayName: Elevon reverse
    // @Description: Reverse elevon mixing
    // @Values: 0:Disabled,1:Enabled
    // @User: User
    GSCALAR(reverse_elevons,        "ELEVON_REVERSE", ELEVON_REVERSE),


    // @Param: ELEVON_CH1_REV
    // @DisplayName: Elevon reverse
    // @Description: Reverse elevon channel 1
    // @Values: -1:Disabled,1:Enabled
    // @User: User
    GSCALAR(reverse_ch1_elevon,     "ELEVON_CH1_REV", ELEVON_CH1_REVERSE),

    // @Param: ELEVON_CH2_REV
    // @DisplayName: Elevon reverse
    // @Description: Reverse elevon channel 2
    // @Values: -1:Disabled,1:Enabled
    // @User: User
    GSCALAR(reverse_ch2_elevon,     "ELEVON_CH2_REV", ELEVON_CH2_REVERSE),

    // @Param: VTAIL_OUTPUT
    // @DisplayName: VTail output
    // @Description: Enable VTail output in software. If enabled then the APM will provide software VTail mixing on the elevator and rudder channels. There are 4 different mixing modes available, which refer to the 4 ways the elevator can be mapped to the two VTail servos. Note that you must not use VTail output mixing with hardware pass-through of RC values, such as with channel 8 manual control on an APM1. So if you use an APM1 then set FLTMODE_CH to something other than 8 before you enable VTAIL_OUTPUT. Please also see the MIXING_GAIN parameter for the output gain of the mixer.
    // @Values: 0:Disabled,1:UpUp,2:UpDown,3:DownUp,4:DownDown
    // @User: User
    GSCALAR(vtail_output,           "VTAIL_OUTPUT",  0),

    // @Param: ELEVON_OUTPUT
    // @DisplayName: Elevon output
    // @Description: Enable software elevon output mixer. If enabled then the APM will provide software elevon mixing on the aileron and elevator channels. There are 4 different mixing modes available, which refer to the 4 ways the elevator can be mapped to the two elevon servos. Note that you must not use elevon output mixing with hardware pass-through of RC values, such as with channel 8 manual control on an APM1. So if you use an APM1 then set FLTMODE_CH to something other than 8 before you enable ELEVON_OUTPUT. Please also see the MIXING_GAIN parameter for the output gain of the mixer.
    // @Values: 0:Disabled,1:UpUp,2:UpDown,3:DownUp,4:DownDown
    // @User: User
    GSCALAR(elevon_output,           "ELEVON_OUTPUT",  0),

    // @Param: MIXING_GAIN
    // @DisplayName: Mixing Gain
    // @Description: The gain for the Vtail and elevon output mixers. The default is 0.5, which ensures that the mixer doesn't saturate, allowing both input channels to go to extremes while retaining control over the output. Hardware mixers often have a 1.0 gain, which gives more servo throw, but can saturate. If you don't have enough throw on your servos with VTAIL_OUTPUT or ELEVON_OUTPUT enabled then you can raise the gain using MIXING_GAIN. The mixer allows outputs in the range 900 to 2100 microseconds.
    // @Range: 0.5 1.2
    // @User: User
    GSCALAR(mixing_gain,            "MIXING_GAIN",    0.5f),

    // @Param: SYS_NUM_RESETS
    // @DisplayName: Num Resets
    // @Description: Number of APM board resets
    // @User: Advanced
    GSCALAR(num_resets,             "SYS_NUM_RESETS", 0),

    // @Param: SIMPLE
    // @DisplayName: Simple mode bitmask
    // @Description: Bitmask which holds which flight modes use simple heading mode (eg bit 0 = 1 means Flight Mode 0 uses simple mode)
    // @User: Advanced
    GSCALAR(simple_modes, "SIMPLE",                 0),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: 2 byte bitmap of log types to enable
    // @Values: 0:Disabled,830:Default,894:Default+RCIN,958:Default+IMU,1854:Default+Motors,-6146:NearlyAll
    // @User: Standard
    GSCALAR(log_bitmask,    "LOG_BITMASK",          DEFAULT_LOG_BITMASK),

    // @Param: RST_SWITCH_CH
    // @DisplayName: Reset Switch Channel
    // @Description: RC channel to use to reset to last flight mode	after geofence takeover.
    // @User: Advanced
    GSCALAR(reset_switch_chan,      "RST_SWITCH_CH",  0),

    // @Param: RST_MISSION_CH
    // @DisplayName: Reset Mission Channel
    // @Description: RC channel to use to reset the mission to the first waypoint. When this channel goes above 1750 the mission is reset. Set RST_MISSION_CH to 0 to disable.
    // @User: Advanced
    GSCALAR(reset_mission_chan,      "RST_MISSION_CH",  0),

    // @Param: TRIM_ARSPD_CM
    // @DisplayName: Target airspeed
    // @Description: Airspeed in cm/s to aim for when airspeed is enabled in auto mode. This is a calibrated (apparent) airspeed.
    // @Units: cm/s
    // @User: User
    GSCALAR(airspeed_cruise_cm,     "TRIM_ARSPD_CM",  AIRSPEED_CRUISE_CM),

    // @Param: SCALING_SPEED
    // @DisplayName: speed used for speed scaling calculations
    // @Description: Airspeed in m/s to use when calculating surface speed scaling. Note that changing this value will affect all PID values
    // @Units: m/s
    // @User: Advanced
    GSCALAR(scaling_speed,        "SCALING_SPEED",    SCALING_SPEED),

    // @Param: MIN_GNDSPD_CM
    // @DisplayName: Minimum ground speed
    // @Description: Minimum ground speed in cm/s when under airspeed control
    // @Units: cm/s
    // @User: Advanced
    GSCALAR(min_gndspeed_cm,      "MIN_GNDSPD_CM",  MIN_GNDSPEED_CM),

    // @Param: TRIM_PITCH_CD
    // @DisplayName: Pitch angle offset
    // @Description: offset to add to pitch - used for in-flight pitch trimming. It is recommended that instead of using this parameter you level your plane correctly on the ground for good flight attitude.
    // @Units: centi-Degrees
    // @User: Advanced
    GSCALAR(pitch_trim_cd,        "TRIM_PITCH_CD",  0),

    // @Param: ALT_HOLD_RTL
    // @DisplayName: PLANE_RTL altitude
    // @Description: Return to launch target altitude. This is the altitude the plane will aim for and loiter at when returning home. If this is negative (usually -1) then the plane will use the current altitude at the time of entering PLANE_RTL. Note that when transiting to a Rally Point the alitude of the Rally Point is used instead of ALT_HOLD_RTL.
    // @Units: centimeters
    // @User: User
    GSCALAR(RTL_altitude_cm,        "ALT_HOLD_RTL",   ALT_HOLD_HOME_CM),

    // @Param: ALT_HOLD_FBWCM
    // @DisplayName: Minimum altitude for FBWB mode
    // @Description: This is the minimum altitude in centimeters that FBWB and PLANE_CRUISE modes will allow. If you attempt to descend below this altitude then the plane will level off. A value of zero means no limit.
    // @Units: centimeters
    // @User: User
    GSCALAR(FBWB_min_altitude_cm,   "ALT_HOLD_FBWCM", ALT_HOLD_FBW_CM),

    // @Param: FLAP_1_PERCNT
    // @DisplayName: Flap 1 percentage
    // @Description: The percentage change in flap position when FLAP_1_SPEED is reached. Use zero to disable flaps
    // @Range: 0 100
    // @Units: Percent
    // @User: Advanced
    GSCALAR(flap_1_percent,         "FLAP_1_PERCNT",  FLAP_1_PERCENT),

    // @Param: FLAP_1_SPEED
    // @DisplayName: Flap 1 speed
    // @Description: The speed in meters per second at which to engage FLAP_1_PERCENT of flaps. Note that FLAP_1_SPEED should be greater than or equal to FLAP_2_SPEED
    // @Range: 0 100
	// @Increment: 1
    // @Units: m/s
    // @User: Advanced
    GSCALAR(flap_1_speed,           "FLAP_1_SPEED",   FLAP_1_SPEED),

    // @Param: FLAP_2_PERCNT
    // @DisplayName: Flap 2 percentage
    // @Description: The percentage change in flap position when FLAP_2_SPEED is reached. Use zero to disable flaps
    // @Range: 0 100
	// @Units: Percent
    // @User: Advanced
    GSCALAR(flap_2_percent,         "FLAP_2_PERCNT",  FLAP_2_PERCENT),

    // @Param: FLAP_2_SPEED
    // @DisplayName: Flap 2 speed
    // @Description: The speed in meters per second at which to engage FLAP_2_PERCENT of flaps. Note that FLAP_1_SPEED should be greater than or equal to FLAP_2_SPEED
    // @Range: 0 100
	// @Units: m/s
	// @Increment: 1
    // @User: Advanced
    GSCALAR(flap_2_speed,           "FLAP_2_SPEED",   FLAP_2_SPEED),

    // @Param: ESC
    // @DisplayName: ESC Calibration
    // @Description: Controls whether ArduCopter will enter ESC calibration on the next restart.  Do not adjust this parameter manually.
    // @User: Advanced
    // @Values: 0:Normal Start-up,1:Start-up in ESC Calibration mode
    GSCALAR(esc_calibrate, "ESC",                   0),

    // @Param: TUNE
    // @DisplayName: Channel 6 Tuning
    // @Description: Controls which parameters (normally PID gains) are being tuned with transmitter's channel 6 knob
    // @User: Standard
    // @Values: 0:None,1:Stab Roll/Pitch kP,4:Rate Roll/Pitch kP,5:Rate Roll/Pitch kI,21:Rate Roll/Pitch kD,3:Stab Yaw kP,6:Rate Yaw kP,26:Rate Yaw kD,14:Altitude Hold kP,7:Throttle Rate kP,37:Throttle Rate kD,34:Throttle Accel kP,35:Throttle Accel kI,36:Throttle Accel kD,42:Loiter Speed,12:Loiter Pos kP,22:Loiter Rate kP,28:Loiter Rate kI,23:Loiter Rate kD,10:WP Speed,25:Acro RollPitch kP,40:Acro Yaw kP,9:Relay On/Off,13:Heli Ext Gyro,17:OF Loiter kP,18:OF Loiter kI,19:OF Loiter kD,30:AHRS Yaw kP,31:AHRS kP,32:INAV_TC,38:Declination,39:Circle Rate,41:Sonar Gain
    GSCALAR(radio_tuning, "TUNE",                   0),

    // @Param: TUNE_LOW
    // @DisplayName: Tuning minimum
    // @Description: The minimum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
    // @User: Standard
    // @Range: 0 32767
    GSCALAR(radio_tuning_low, "TUNE_LOW",           0),

    // @Param: TUNE_HIGH
    // @DisplayName: Tuning maximum
    // @Description: The maximum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
    // @User: Standard
    // @Range: 0 32767
    GSCALAR(radio_tuning_high, "TUNE_HIGH",         1000),

    // @Param: FRAME
    // @DisplayName: Frame Orientation (+, X or V)
    // @Description: Controls motor mixing for multicopters.  Not used for Tri or Traditional Helicopters.
    // @Values: 0:Plus, 1:X, 2:V, 3:H, 10:Y6B (New)
    // @User: Standard
    GSCALAR(frame_orientation, "FRAME",             AP_MOTORS_X_FRAME),

    // @Param: CH7_OPT
    // @DisplayName: Channel 7 option
    // @Description: Select which function if performed when CH7 is above 1800 pwm
    // @Values: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 8:Multi Mode, 9:Camera Trigger, 10:Sonar, 11:Fence, 12:ResetToArmedYaw, 13:Super Simple Mode, 14:Acro Trainer, 16:Auto, 17:AutoTune, 18:Land, 19:EPM
    // @User: Standard
    GSCALAR(ch7_option, "CH7_OPT",                  CH7_OPTION),

    // @Param: CH8_OPT
    // @DisplayName: Channel 8 option
    // @Description: Select which function if performed when CH8 is above 1800 pwm
    // @Values: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 8:Multi Mode, 9:Camera Trigger, 10:Sonar, 11:Fence, 12:ResetToArmedYaw, 13:Super Simple Mode, 14:Acro Trainer, 16:Auto, 17:AutoTune, 18:Land, 19:EPM
    // @User: Standard
    GSCALAR(ch8_option, "CH8_OPT",                  CH8_OPTION),

    // @Param: ARMING_CHECK
    // @DisplayName: Arming check
    // @Description: Allows enabling or disabling of pre-arming checks of receiver, accelerometer, barometer, compass and GPS
    // @Values: 0:Disabled, 1:Enabled, -3:Skip Baro, -5:Skip Compass, -9:Skip GPS, -17:Skip INS, -33:Skip Parameters, -65:Skip RC, 127:Skip Voltage
    // @User: Standard
    GSCALAR(arming_check, "ARMING_CHECK",           ARMING_CHECK_ALL),

    // @Param: ANGLE_MAX
    // @DisplayName: Angle Max
    // @Description: Maximum lean angle in all flight modes
    // @Range 1000 8000
    // @User: Advanced
    GSCALAR(angle_max, "ANGLE_MAX",                 DEFAULT_ANGLE_MAX),

    // @Param: ANGLE_RATE_MAX
    // @DisplayName: Angle Rate max
    // @Description: maximum rotation rate in roll/pitch axis requested by angle controller used in stabilize, loiter, rtl, auto flight modes
    // @Range 90000 250000
    // @User: Advanced
    GSCALAR(angle_rate_max, "ANGLE_RATE_MAX",  ANGLE_RATE_MAX),

    // @Param: RC_FEEL_RP
    // @DisplayName: RC Feel Roll/Pitch
    // @Description: RC feel for roll/pitch which controls vehicle response to user input with 0 being extremely soft and 100 begin crisp
    // @User: Advanced
    // @Values: 0:Very Soft, 25:Soft, 50:Medium, 75:Crisp, 100:Very Crisp
    GSCALAR(rc_feel_rp, "RC_FEEL_RP",  RC_FEEL_RP_VERY_CRISP),

#if FRAME_CONFIG ==     HELI_FRAME
    // @Group: HS1_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(heli_servo_1,    "HS1_", RC_Channel),
    // @Group: HS2_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(heli_servo_2,    "HS2_", RC_Channel),
    // @Group: HS3_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(heli_servo_3,    "HS3_", RC_Channel),
    // @Group: HS4_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(heli_servo_4,    "HS4_", RC_Channel),

    // @Param: RATE_PIT_FF
    // @DisplayName: Rate Pitch Feed Forward
    // @Description: Rate Pitch Feed Forward (for TradHeli Only)
    // @Range: 0 10
    // @Increment: 0.01
    // @User: Standard
	GSCALAR(heli_pitch_ff, "RATE_PIT_FF",            HELI_PITCH_FF),

    // @Param: RATE_RLL_FF
    // @DisplayName: Rate Roll Feed Forward
    // @Description: Rate Roll Feed Forward (for TradHeli Only)
    // @Range: 0 10
    // @Increment: 0.01
    // @User: Standard
	GSCALAR(heli_roll_ff, "RATE_RLL_FF",            HELI_ROLL_FF),

    // @Param: RATE_YAW_FF
    // @DisplayName: Rate Yaw Feed Forward
    // @Description: Rate Yaw Feed Forward (for TradHeli Only)
    // @Range: 0 10
    // @Increment: 0.01
    // @User: Standard
	GSCALAR(heli_yaw_ff, "RATE_YAW_FF",            HELI_YAW_FF),

    // @Param: H_STAB_COL_MIN
    // @DisplayName: Heli Stabilize Throttle Collective Minimum
    // @Description: Helicopter's minimum collective position while pilot directly controls collective in stabilize mode
    // @Range: 0 500
    // @Units: Percent*10
    // @Increment: 1
    // @User: Standard
    GSCALAR(heli_stab_col_min, "H_STAB_COL_MIN", HELI_STAB_COLLECTIVE_MIN_DEFAULT),

    // @Param: H_STAB_COL_MAX
    // @DisplayName: Stabilize Throttle Maximum
    // @Description: Helicopter's maximum collective position while pilot directly controls collective in stabilize mode
    // @Range: 500 1000
    // @Units: Percent*10
    // @Increment: 1
    // @User: Standard
    GSCALAR(heli_stab_col_max, "H_STAB_COL_MAX", HELI_STAB_COLLECTIVE_MAX_DEFAULT),
#endif

#if FRAME_CONFIG ==     SINGLE_FRAME
    // @Group: SS1_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(single_servo_1,    "SS1_", RC_Channel),
    // @Group: SS2_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(single_servo_2,    "SS2_", RC_Channel),
    // @Group: SS3_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(single_servo_3,    "SS3_", RC_Channel),
    // @Group: SS4_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(single_servo_4,    "SS4_", RC_Channel),
#endif

    // RC channel
    //-----------
    // @Group: RC1_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_1,    "RC1_", RC_Channel),
    // @Group: RC2_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_2,    "RC2_", RC_Channel),
    // @Group: RC3_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_3,    "RC3_", RC_Channel),
    // @Group: RC4_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_4,    "RC4_", RC_Channel),
    // @Group: RC5_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_5,    "RC5_", RC_Channel_aux),
    // @Group: RC6_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_6,    "RC6_", RC_Channel_aux),
    // @Group: RC7_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_7,    "RC7_", RC_Channel_aux),
    // @Group: RC8_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_8,    "RC8_", RC_Channel_aux),

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    // @Group: RC9_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_9,                    "RC9_", RC_Channel_aux),
#endif

    // @Group: RC10_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_10,                    "RC10_", RC_Channel_aux),
    // @Group: RC11_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_11,                    "RC11_", RC_Channel_aux),

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    // @Group: RC12_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_12,                   "RC12_", RC_Channel_aux),
#endif

    // @Group: RLL2SRV_
    // @Path: ../libraries/APM_Control/AP_RollController.cpp
	GOBJECT(rollController,         "RLL2SRV_",   AP_RollController),

    // @Group: PTCH2SRV_
    // @Path: ../libraries/APM_Control/AP_PitchController.cpp
	GOBJECT(pitchController,        "PTCH2SRV_",  AP_PitchController),

    // @Group: YAW2SRV_
    // @Path: ../libraries/APM_Control/AP_YawController.cpp
	GOBJECT(yawController,          "YAW2SRV_",   AP_YawController),

    // @Group: STEER2SRV_
    // @Path: ../libraries/APM_Control/AP_SteerController.cpp
	GOBJECT(steerController,        "STEER2SRV_",   AP_SteerController),

    // @Param: RC_SPEED
    // @DisplayName: ESC Update Speed
    // @Description: This is the speed in Hertz that your ESCs will receive updates
    // @Units: Hz
    // @Range: 50 490
    // @Increment: 1
    // @User: Advanced
    GSCALAR(rc_speed, "RC_SPEED",              RC_FAST_SPEED),

    // @Param: ACRO_RP_P
    // @DisplayName: Acro Roll and Pitch P gain
    // @Description: Converts pilot roll and pitch into a desired rate of rotation in ACRO and SPORT mode.  Higher values mean faster rate of rotation.
    // @Range: 1 10
    // @User: Standard
    GSCALAR(acro_rp_p,                 "ACRO_RP_P",           ACRO_RP_P),

    // @Param: ACRO_YAW_P
    // @DisplayName: Acro Yaw P gain
    // @Description: Converts pilot yaw input into a desired rate of rotation in ACRO, Stabilize and SPORT modes.  Higher values mean faster rate of rotation.
    // @Range: 1 10
    // @User: Standard
    GSCALAR(acro_yaw_p,                 "ACRO_YAW_P",           ACRO_YAW_P),

    // @Param: ACRO_BAL_ROLL
    // @DisplayName: Acro Balance Roll
    // @Description: rate at which roll angle returns to level in acro mode
    // @Range: 0 3
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(acro_balance_roll,      "ACRO_BAL_ROLL",    ACRO_BALANCE_ROLL),

    // @Param: ACRO_BAL_PITCH
    // @DisplayName: Acro Balance Pitch
    // @Description: rate at which pitch angle returns to level in acro mode
    // @Range: 0 3
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(acro_balance_pitch,     "ACRO_BAL_PITCH",   ACRO_BALANCE_PITCH),

    // @Param: ACRO_TRAINER
    // @DisplayName: Acro Trainer
    // @Description: Type of trainer used in acro mode
    // @Values: 0:Disabled,1:Leveling,2:Leveling and Limited
    // @User: Advanced
    GSCALAR(acro_trainer,   "ACRO_TRAINER",     ACRO_TRAINER_LIMITED),

    // PID controller
    //---------------
    // @Param: RATE_RLL_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output
    // @Range: 0.08 0.20
    // @Increment: 0.005
    // @User: Standard

    // @Param: RATE_RLL_I
    // @DisplayName: Roll axis rate controller I gain
    // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.01 0.5
    // @Increment: 0.01
    // @User: Standard

    // @Param: RATE_RLL_IMAX
    // @DisplayName: Roll axis rate controller I gain maximum
    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 500
    // @Increment: 10
    // @Units: Percent*10
    // @User: Standard

    // @Param: RATE_RLL_D
    // @DisplayName: Roll axis rate controller D gain
    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.001 0.02
    // @Increment: 0.001
    // @User: Standard
    GGROUP(pid_rate_roll,     "RATE_RLL_", AC_PID),

    // @Param: RATE_PIT_P
    // @DisplayName: Pitch axis rate controller P gain
    // @Description: Pitch axis rate controller P gain.  Converts the difference between desired pitch rate and actual pitch rate into a motor speed output
    // @Range: 0.08 0.20
    // @Increment: 0.005
    // @User: Standard

    // @Param: RATE_PIT_I
    // @DisplayName: Pitch axis rate controller I gain
    // @Description: Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
    // @Range: 0.01 0.5
    // @Increment: 0.01
    // @User: Standard

    // @Param: RATE_PIT_IMAX
    // @DisplayName: Pitch axis rate controller I gain maximum
    // @Description: Pitch axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 500
    // @Increment: 10
    // @Units: Percent*10
    // @User: Standard

    // @Param: RATE_PIT_D
    // @DisplayName: Pitch axis rate controller D gain
    // @Description: Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
    // @Range: 0.001 0.02
    // @Increment: 0.001
    // @User: Standard
    GGROUP(pid_rate_pitch,    "RATE_PIT_", AC_PID),

    // @Param: RATE_YAW_P
    // @DisplayName: Yaw axis rate controller P gain
    // @Description: Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output
    // @Range: 0.150 0.250
    // @Increment: 0.005
    // @User: Standard

    // @Param: RATE_YAW_I
    // @DisplayName: Yaw axis rate controller I gain
    // @Description: Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
    // @Range: 0.010 0.020
    // @Increment: 0.01
    // @User: Standard

    // @Param: RATE_YAW_IMAX
    // @DisplayName: Yaw axis rate controller I gain maximum
    // @Description: Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 800
    // @Increment: 10
    // @Units: Percent*10
    // @User: Standard

    // @Param: RATE_YAW_D
    // @DisplayName: Yaw axis rate controller D gain
    // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
    // @Range: 0.000 0.02
    // @Increment: 0.001
    // @User: Standard
    GGROUP(pid_rate_yaw,      "RATE_YAW_", AC_PID),

    // @Param: LOITER_LAT_P
    // @DisplayName: Loiter latitude rate controller P gain
    // @Description: Loiter latitude rate controller P gain.  Converts the difference between desired speed and actual speed into a lean angle in the latitude direction
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: LOITER_LAT_I
    // @DisplayName: Loiter latitude rate controller I gain
    // @Description: Loiter latitude rate controller I gain.  Corrects long-term difference in desired speed and actual speed in the latitude direction
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: LOITER_LAT_IMAX
    // @DisplayName: Loiter rate controller I gain maximum
    // @Description: Loiter rate controller I gain maximum.  Constrains the lean angle that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: Centi-Degrees
    // @User: Advanced

    // @Param: LOITER_LAT_D
    // @DisplayName: Loiter latitude rate controller D gain
    // @Description: Loiter latitude rate controller D gain.  Compensates for short-term change in desired speed vs actual speed
    // @Range: 0.0 0.6
    // @Increment: 0.01
    // @User: Advanced
    GGROUP(pid_loiter_rate_lat,      "LOITER_LAT_",  AC_PID),

    // @Param: LOITER_LON_P
    // @DisplayName: Loiter longitude rate controller P gain
    // @Description: Loiter longitude rate controller P gain.  Converts the difference between desired speed and actual speed into a lean angle in the longitude direction
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: LOITER_LON_I
    // @DisplayName: Loiter longitude rate controller I gain
    // @Description: Loiter longitude rate controller I gain.  Corrects long-term difference in desired speed and actual speed in the longitude direction
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: LOITER_LON_IMAX
    // @DisplayName: Loiter longitude rate controller I gain maximum
    // @Description: Loiter longitude rate controller I gain maximum.  Constrains the lean angle that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: Centi-Degrees
    // @User: Advanced

    // @Param: LOITER_LON_D
    // @DisplayName: Loiter longituderate controller D gain
    // @Description: Loiter longitude rate controller D gain.  Compensates for short-term change in desired speed vs actual speed
    // @Range: 0.0 0.6
    // @Increment: 0.01
    // @User: Advanced
    GGROUP(pid_loiter_rate_lon,      "LOITER_LON_",  AC_PID),

    // @Param: THR_RATE_P
    // @DisplayName: Throttle rate controller P gain
    // @Description: Throttle rate controller P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
    // @Range: 1.000 8.000
    // @User: Standard

    // @Param: THR_RATE_I
    // @DisplayName: Throttle rate controller I gain
    // @Description: Throttle rate controller I gain.  Corrects long-term difference in desired vertical speed and actual speed
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: THR_RATE_IMAX
    // @DisplayName: Throttle rate controller I gain maximum
    // @Description: Throttle rate controller I gain maximum.  Constrains the desired acceleration that the I gain will generate
    // @Range: 0 500
    // @Units: cm/s/s
    // @User: Standard


    // @Param: THR_RATE_D
    // @DisplayName: Throttle rate controller D gain
    // @Description: Throttle rate controller D gain.  Compensates for short-term change in desired vertical speed vs actual speed
    // @Range: 0.000 0.400
    // @User: Standard
    GGROUP(pid_throttle_rate, "THR_RATE_", AC_PID),

    // @Param: THR_ACCEL_P
    // @DisplayName: Throttle acceleration controller P gain
    // @Description: Throttle acceleration controller P gain.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output
    // @Range: 0.500 1.500
    // @User: Standard

    // @Param: THR_ACCEL_I
    // @DisplayName: Throttle acceleration controller I gain
    // @Description: Throttle acceleration controller I gain.  Corrects long-term difference in desired vertical acceleration and actual acceleration
    // @Range: 0.000 3.000
    // @User: Standard

    // @Param: THR_ACCEL_IMAX
    // @DisplayName: Throttle acceleration controller I gain maximum
    // @Description: Throttle acceleration controller I gain maximum.  Constrains the maximum pwm that the I term will generate
    // @Range: 0 500
    // @Units: Percent*10
    // @User: Standard

    // @Param: THR_ACCEL_D
    // @DisplayName: Throttle acceleration controller D gain
    // @Description: Throttle acceleration controller D gain.  Compensates for short-term change in desired vertical acceleration vs actual acceleration
    // @Range: 0.000 0.400
    // @User: Standard
    GGROUP(pid_throttle_accel,"THR_ACCEL_", AC_PID),

    // @Param: OF_RLL_P
    // @DisplayName: Optical Flow based loiter controller roll axis P gain
    // @Description: Optical Flow based loiter controller roll axis P gain.  Converts the position error from the target point to a roll angle
    // @Range: 2.000 3.000
    // @User: Standard

    // @Param: OF_RLL_I
    // @DisplayName: Optical Flow based loiter controller roll axis I gain
    // @Description: Optical Flow based loiter controller roll axis I gain.  Corrects long-term position error by more persistently rolling left or right
    // @Range: 0.250 0.750
    // @User: Standard


    // @Param: OF_RLL_IMAX
    // @DisplayName: Optical Flow based loiter controller roll axis I gain maximum
    // @Description: Optical Flow based loiter controller roll axis I gain maximum.  Constrains the maximum roll angle that the I term will generate
    // @Range: 0 4500
    // @Units: Centi-Degrees
    // @User: Standard

    // @Param: OF_RLL_D
    // @DisplayName: Optical Flow based loiter controller roll axis D gain
    // @Description: Optical Flow based loiter controller roll axis D gain.  Compensates for short-term change in speed in the roll direction
    // @Range: 0.100 0.140
    // @User: Standard
    GGROUP(pid_optflow_roll,  "OF_RLL_",   AC_PID),


    // @Param: OF_PIT_P
    // @DisplayName: Optical Flow based loiter controller pitch axis P gain
    // @Description: Optical Flow based loiter controller pitch axis P gain.  Converts the position error from the target point to a pitch angle
    // @Range: 2.000 3.000
    // @User: Standard


    // @Param: OF_PIT_I
    // @DisplayName: Optical Flow based loiter controller pitch axis I gain
    // @Description: Optical Flow based loiter controller pitch axis I gain.  Corrects long-term position error by more persistently pitching left or right
    // @Range: 0.250 0.750
    // @User: Standard


    // @Param: OF_PIT_IMAX
    // @DisplayName: Optical Flow based loiter controller pitch axis I gain maximum
    // @Description: Optical Flow based loiter controller pitch axis I gain maximum.  Constrains the maximum pitch angle that the I term will generate
    // @Range: 0 4500
    // @Units: Centi-Degrees
    // @User: Standard

    // @Param: OF_PIT_D
    // @DisplayName: Optical Flow based loiter controller pitch axis D gain
    // @Description: Optical Flow based loiter controller pitch axis D gain.  Compensates for short-term change in speed in the pitch direction
    // @Range: 0.100 0.140
    // @User: Standard
    GGROUP(pid_optflow_pitch, "OF_PIT_",   AC_PID),

    // PI controller
    //--------------
    // @Param: STB_RLL_P
    // @DisplayName: Roll axis stabilize controller P gain
    // @Description: Roll axis stabilize (i.e. angle) controller P gain.  Converts the error between the desired roll angle and actual angle to a desired roll rate
    // @Range: 3.000 6.000
    // @User: Standard

    // @Param: STB_RLL_I
    // @DisplayName: Roll axis stabilize controller I gain
    // @Description: Roll axis stabilize (i.e. angle) controller I gain.  Corrects for longer-term difference in desired roll angle and actual angle
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: STB_RLL_IMAX
    // @DisplayName: Roll axis stabilize controller I gain maximum
    // @Description: Roll axis stabilize (i.e. angle) controller I gain maximum.  Constrains the maximum roll rate that the I term will generate
    // @Range: 0 4500
    // @Units: Centi-Degrees/Sec
    // @User: Standard
    GGROUP(pi_stabilize_roll,       "STB_RLL_", APM_PI),

    // @Param: STB_PIT_P
    // @DisplayName: Pitch axis stabilize controller P gain
    // @Description: Pitch axis stabilize (i.e. angle) controller P gain.  Converts the error between the desired pitch angle and actual angle to a desired pitch rate
    // @Range: 3.000 6.000
    // @User: Standard

    // @Param: STB_PIT_I
    // @DisplayName: Pitch axis stabilize controller I gain
    // @Description: Pitch axis stabilize (i.e. angle) controller I gain.  Corrects for longer-term difference in desired pitch angle and actual angle
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: STB_PIT_IMAX
    // @DisplayName: Pitch axis stabilize controller I gain maximum
    // @Description: Pitch axis stabilize (i.e. angle) controller I gain maximum.  Constrains the maximum pitch rate that the I term will generate
    // @Range: 0 4500
    // @Units: Centi-Degrees/Sec
    // @User: Standard
    GGROUP(pi_stabilize_pitch,      "STB_PIT_", APM_PI),

    // @Param: STB_YAW_P
    // @DisplayName: Yaw axis stabilize controller P gain
    // @Description: Yaw axis stabilize (i.e. angle) controller P gain.  Converts the error between the desired yaw angle and actual angle to a desired yaw rate
    // @Range: 3.000 6.000
    // @User: Standard

    // @Param: STB_YAW_I
    // @DisplayName: Yaw axis stabilize controller I gain
    // @Description: Yaw axis stabilize (i.e. angle) controller I gain.  Corrects for longer-term difference in desired yaw angle and actual angle
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: STB_YAW_IMAX
    // @DisplayName: Yaw axis stabilize controller I gain maximum
    // @Description: Yaw axis stabilize (i.e. angle) controller I gain maximum.  Constrains the maximum yaw rate that the I term will generate
    // @Range: 0 4500
    // @Units: Centi-Degrees/Sec
    // @User: Standard
    GGROUP(pi_stabilize_yaw,        "STB_YAW_", APM_PI),

    // @Param: THR_ALT_P
    // @DisplayName: Altitude controller P gain
    // @Description: Altitude controller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller
    // @Range: 1.000 3.000
    // @User: Standard

    // @Param: THR_ALT_I
    // @DisplayName: Altitude controller I gain
    // @Description: Altitude controller I gain.  Corrects for longer-term difference in desired altitude and actual altitude
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: THR_ALT_IMAX
    // @DisplayName: Altitude controller I gain maximum
    // @Description: Altitude controller I gain maximum.  Constrains the maximum climb rate rate that the I term will generate
    // @Range: 0 500
    // @Units: cm/s
    // @User: Standard
    GGROUP(pi_alt_hold,     "THR_ALT_", APM_PI),

    // @Param: HLD_LAT_P
    // @DisplayName: Loiter latitude position controller P gain
    // @Description: Loiter latitude position controller P gain.  Converts the distance (in the latitude direction) to the target location into a desired speed which is then passed to the loiter latitude rate controller
    // @Range: 0.500 2.000
    // @User: Standard

    // @Param: HLD_LAT_I
    // @DisplayName: Loiter latitude position controller I gain
    // @Description: Loiter latitude position controller I gain.  Corrects for longer-term distance (in latitude) to the target location
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: HLD_LAT_IMAX
    // @DisplayName: Loiter latitude position controller I gain maximum
    // @Description: Loiter latitude position controller I gain maximum.  Constrains the maximum desired speed that the I term will generate
    // @Range: 0 3000
    // @Units: cm/s
    // @User: Standard
    GGROUP(pi_loiter_lat,   "HLD_LAT_", APM_PI),

    // @Param: HLD_LON_P
    // @DisplayName: Loiter longitude position controller P gain
    // @Description: Loiter longitude position controller P gain.  Converts the distance (in the longitude direction) to the target location into a desired speed which is then passed to the loiter longitude rate controller
    // @Range: 0.500 2.000
    // @User: Standard

    // @Param: HLD_LON_I
    // @DisplayName: Loiter longitude position controller I gain
    // @Description: Loiter longitude position controller I gain.  Corrects for longer-term distance (in longitude direction) to the target location
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: HLD_LON_IMAX
    // @DisplayName: Loiter longitudeposition controller I gain maximum
    // @Description: Loiter  longitudeposition controller I gain maximum.  Constrains the maximum desired speed that the I term will generate
    // @Range: 0 3000
    // @Units: cm/s
    // @User: Standard
    GGROUP(pi_loiter_lon,   "HLD_LON_", APM_PI),

    // variables not in the g class which contain EEPROM saved variables

#if CAMERA == ENABLED
    // @Group: CAM_
    // @Path: ../libraries/AP_Camera/AP_Camera.cpp
    GOBJECT(camera,           "CAM_", AP_Camera),
#endif

    // @Group: ARMING_
    // @Path: ../libraries/AP_Arming/AP_Arming.cpp
    GOBJECT(arming,                 "ARMING_", AP_Arming),

    // @Group: RELAY_
    // @Path: ../libraries/AP_Relay/AP_Relay.cpp
    GOBJECT(relay,                  "RELAY_", AP_Relay),

#if EPM_ENABLED == ENABLED
	// @Group: EPM_
    // @Path: ../libraries/AP_EPM/AP_EPM.cpp
    GOBJECT(epm,            "EPM_", AP_EPM),
#endif

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/Compass.cpp
    GOBJECT(compass,        "COMPASS_", Compass),

    // @Group: INS_
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
#if HIL_MODE != HIL_MODE_ATTITUDE
    GOBJECT(ins,            "INS_", AP_InertialSensor),
#endif

    // @Group: INAV_
    // @Path: ../libraries/AP_InertialNav/AP_InertialNav.cpp
    GOBJECT(inertial_nav,           "INAV_",    AP_InertialNav),

    // @Group: WPNAV_
    // @Path: ../libraries/AC_WPNav/AC_WPNav.cpp
    GOBJECT(wp_nav, "WPNAV_",       AC_WPNav),

    // @Group: SR0_
    // @Path: GCS_Mavlink.pde
    GOBJECTN(gcs[0],  gcs0,       "SR0_",     GCS_MAVLINK),

    // @Group: SR1_
    // @Path: GCS_Mavlink.pde
    GOBJECTN(gcs[1],  gcs1,       "SR1_",     GCS_MAVLINK),

#if MAVLINK_COMM_NUM_BUFFERS > 2
    // @Group: SR2_
    // @Path: GCS_Mavlink.pde
    GOBJECTN(gcs[2],  gcs2,       "SR2_",     GCS_MAVLINK),
#endif

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

    // @Group: ARSPD_
    // @Path: ../libraries/AP_Airspeed/AP_Airspeed.cpp
    GOBJECT(airspeed,                               "ARSPD_",   AP_Airspeed),

    // @Group: NAVL1_
    // @Path: ../libraries/AP_L1_Control/AP_L1_Control.cpp
    GOBJECT(L1_controller,         "NAVL1_",   AP_L1_Control),

    // @Group: TECS_
    // @Path: ../libraries/AP_TECS/AP_TECS.cpp
    GOBJECT(TECS_controller,         "TECS_",   AP_TECS),

#if MOUNT == ENABLED
    // @Group: MNT_
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount,           "MNT_", AP_Mount),
#endif

#if MOUNT2 == ENABLED
    // @Group: MNT2_
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount2,           "MNT2_",       AP_Mount),
#endif

    // @Group: BATT_
    // @Path: ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    GOBJECT(battery,                "BATT_",       AP_BattMonitor),

    // @Group: BRD_
    // @Path: ../libraries/AP_BoardConfig/AP_BoardConfig.cpp
    GOBJECT(BoardConfig,            "BRD_",       AP_BoardConfig),    

#if SPRAYER == ENABLED
    // @Group: SPRAYER_
    // @Path: ../libraries/AC_Sprayer/AC_Sprayer.cpp
    GOBJECT(sprayer,                "SPRAY_",       AC_Sprayer),
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    GOBJECT(sitl, "SIM_", SITL),
#endif

#if OBC_FAILSAFE == ENABLED
    GOBJECT(obc,  "FS_", APM_OBC),
#endif

    // @Group: GND_
    // @Path: ../libraries/AP_Baro/AP_Baro.cpp
    GOBJECT(barometer, "GND_", AP_Baro),

    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

#if AC_FENCE == ENABLED
    // @Group: FENCE_
    // @Path: ../libraries/AC_Fence/AC_Fence.cpp
    GOBJECT(fence,      "FENCE_",   AC_Fence),
#endif

    // @Group: GPSGLITCH_
    // @Path: ../libraries/AP_GPS/AP_GPS_Glitch.cpp
    GOBJECT(gps_glitch,      "GPSGLITCH_",   GPS_Glitch),

#if FRAME_CONFIG ==     HELI_FRAME
    // @Group: H_
    // @Path: ../libraries/AP_Motors/AP_MotorsHeli.cpp
    GOBJECT(motors, "H_",           AP_MotorsHeli),

#elif FRAME_CONFIG == SINGLE_FRAME
    // @Group: SS1_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(single_servo_1,    "SS1_", RC_Channel),
    // @Group: SS2_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(single_servo_2,    "SS2_", RC_Channel),
    // @Group: SS3_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(single_servo_3,    "SS3_", RC_Channel),
    // @Group: SS4_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(single_servo_4,    "SS4_", RC_Channel),
    // @Group: MOT_
    // @Path: ../libraries/AP_Motors/AP_MotorsSingle.cpp
    GOBJECT(motors, "MOT_",           AP_MotorsSingle),

#else
    // @Group: MOT_
    // @Path: ../libraries/AP_Motors/AP_Motors_Class.cpp
    GOBJECT(motors, "MOT_",         AP_Motors),
#endif

    // @Group: RCMAP_
    // @Path: ../libraries/AP_RCMapper/AP_RCMapper.cpp
    GOBJECT(rcmap, "RCMAP_",        RCMapper),

    AP_VAREND
};

/*
  This is a conversion table from old parameter values to new
  parameter names. The startup code looks for saved values of the old
  parameters and will copy them across to the new parameters if the
  new parameter does not yet have a saved value. It then saves the new
  value.

  Note that this works even if the old parameter has been removed. It
  relies on the old k_param index not being removed

  The second column below is the index in the var_info[] table for the
  old object. This should be zero for top level parameters.
 */

static void load_parameters(void)
{
    // change the default for the AHRS_GPS_GAIN for ArduCopter
    // if it hasn't been set by the user
    if (!ahrs.gps_gain.load()) {
        ahrs.gps_gain.set_and_save(1.0);
    }
    // disable centrifugal force correction, it will be enabled as part of the arming process
    ahrs.set_correct_centrifugal(false);

    // setup different AHRS gains for ArduCopter than the default
    // but allow users to override in their config
    if (!ahrs._kp.load()) {
        ahrs._kp.set_and_save(0.1);
    }
    if (!ahrs._kp_yaw.load()) {
        ahrs._kp_yaw.set_and_save(0.1);
    }

    // setup different Compass learn setting for ArduCopter than the default
    // but allow users to override in their config
    if (!compass._learn.load()) {
        compass._learn.set_and_save(0);
    }

    if (!g.format_version.load() ||
        g.format_version != Parameters::k_format_version) {

        // erase all parameters
        cliSerial->printf_P(PSTR("Firmware change: erasing EEPROM...\n"));
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
        cliSerial->println_P(PSTR("done."));
    } else {
        uint32_t before = micros();
        // Load all auto-loaded EEPROM variables
        AP_Param::load_all();
        cliSerial->printf_P(PSTR("load_all took %luus\n"), micros() - before);
    }
}
#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/UserCode.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifdef USERHOOK_INIT
void userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif
#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/auto_tune.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if AUTOTUNE == ENABLED
/*
    Auto tuning works in this way:
        i) set up 3-position ch7 or ch8 switch to "AutoTune"
        2) take-off in stabilize mode, put the copter into a level hover and switch ch7/ch8 to high position to start tuning
        3) auto tuner brings roll and pitch level
        4) the following procedure is run for roll and then pitch
            a) invokes 90 deg/sec rate request
            b) records maximum "forward" roll rate and bounce back rate
            c) when copter reaches 20 degrees or 1 second has passed, it commands level
            d) tries to keep max rotation rate between 80% ~ 100% of requested rate (90deg/sec) by adjusting rate P
            e) increases rate D until the bounce back becomes greater than 10% of requested rate (90deg/sec)
            f) decreases rate D until the bounce back becomes less than 10% of requested rate (90deg/sec)
            g) increases rate P until the max rotate rate becomes greater than the requeste rate (90deg/sec)
            h) invokes a 20deg angle request on roll or pitch
            i) increases stab P until the maximum angle becomes greater than 110% of the requested angle (20deg)
            j) decreases stab P by 25%
    If pilot inputs any stick inputs these becomes the desired roll, pitch angles sent to the stabilize controller and the tuner is disabled until the sticks are put back into the middle for 1 second
*/

#define AUTO_TUNE_PILOT_OVERRIDE_TIMEOUT_MS  500    // restart tuning if pilot has left sticks in middle for 2 seconds
#define AUTO_TUNE_TARGET_RATE_TIMEOUT_MS     500    // timeout for rate test step
#define AUTO_TUNE_TARGET_RATE_CDS           9000    // target roll/pitch rate during AUTO_TUNE_STEP_TESTING step
#define AUTO_TUNE_LEVEL_ANGLE_CD             300    // angle which qualifies as level
#define AUTO_TUNE_TARGET_ANGLE_CD           2000    // target angle during TESTING_RATE step that will cause us to move to next step
#define AUTO_TUNE_REQUIRED_LEVEL_TIME_MS     250    // time we require the copter to be level
#define AUTO_TUNE_AGGRESSIVENESS            0.1f    // tuning for 10% overshoot
#define AUTO_TUNE_RD_STEP                0.0005f    // minimum increment when increasing/decreasing Rate D term
#define AUTO_TUNE_RP_STEP                 0.005f    // minimum increment when increasing/decreasing Rate P term
#define AUTO_TUNE_SP_STEP                   0.5f    // minimum increment when increasing/decreasing Stab P term
#define AUTO_TUNE_SP_BACKOFF               0.75f    // back off on the Stab P tune
#define AUTO_TUNE_PI_RATIO_FOR_TESTING      0.1f    // I is set 10x smaller than P during testing
#define AUTO_TUNE_RP_RATIO_FINAL            1.0f    // I is set 1x P after testing
#define AUTO_TUNE_RD_MIN                  0.002f    // minimum Rate D value
#define AUTO_TUNE_RD_MAX                  0.015f    // maximum Rate D value
#define AUTO_TUNE_RP_MIN                   0.01f    // minimum Rate P value
#define AUTO_TUNE_RP_MAX                   0.25f    // maximum Rate P value
#define AUTO_TUNE_SP_MAX                   15.0f    // maximum Stab P value
#define AUTO_TUNE_SUCCESS_COUNT                4    // how many successful iterations we need to freeze at current gains

// Auto Tune message ids for ground station
#define AUTO_TUNE_MESSAGE_STARTED 0
#define AUTO_TUNE_MESSAGE_SUCCESS 1
#define AUTO_TUNE_MESSAGE_FAILED 2

enum AutoTuneTuneMode {
    AUTO_TUNE_MODE_UNINITIALISED = 0,
    AUTO_TUNE_MODE_TUNING = 1,
    AUTO_TUNE_MODE_TESTING = 2,
    AUTO_TUNE_MODE_FAILED = 3
};

// things that can be tuned
enum AutoTuneAxisType {
    AUTO_TUNE_AXIS_ROLL = 0,
    AUTO_TUNE_AXIS_PITCH = 1
};

// steps performed during tuning
enum AutoTuneStepType {
    AUTO_TUNE_STEP_WAITING_FOR_LEVEL = 0,
    AUTO_TUNE_STEP_TESTING = 1,
    AUTO_TUNE_STEP_UPDATE_GAINS = 2
};

// steps performed during tuning
enum AutoTuneTuneType {
    AUTO_TUNE_TYPE_RD_UP = 0,
    AUTO_TUNE_TYPE_RD_DOWN = 1,
    AUTO_TUNE_TYPE_RP_UP = 2,
    AUTO_TUNE_TYPE_SP_UP = 3
};

// state
struct auto_tune_state_struct {
    AutoTuneTuneMode mode       : 2;    // see AutoTuneTuneMode for what modes are allowed
    uint8_t pilot_override      : 1;    // 1 = pilot is overriding controls so we suspend tuning temporarily
    AutoTuneAxisType axis       : 1;    // see AutoTuneAxisType for which things can be tuned
    uint8_t positive_direction  : 1;    // 0 = tuning in negative direction (i.e. left for roll), 1 = positive direction (i.e. right for roll)
    AutoTuneStepType step       : 2;    // see AutoTuneStepType for what steps are performed
    AutoTuneTuneType tune_type  : 2;    // see AutoTuneTuneType
} auto_tune_state;

// variables
static uint32_t auto_tune_override_time;   // the last time the pilot overrode the controls
static float auto_tune_test_min;           // the minimum angular rate achieved during TESTING_RATE step
static float auto_tune_test_max;           // the maximum angular rate achieved during TESTING_RATE step
static uint32_t auto_tune_timer;           // generic timer variable
static int8_t auto_tune_counter;           // counter for tuning gains
static float orig_roll_rp, orig_roll_ri, orig_roll_rd, orig_roll_sp;     // backup of currently being tuned parameter values
static float orig_pitch_rp, orig_pitch_ri, orig_pitch_rd, orig_pitch_sp; // backup of currently being tuned parameter values
static float tune_roll_rp, tune_roll_rd, tune_roll_sp;     // currently being tuned parameter values
static float tune_pitch_rp, tune_pitch_rd, tune_pitch_sp;  // currently being tuned parameter values

// store current pids as originals
static void auto_tune_initialise()
{
    // initialise gains and axis because this is our first time
    auto_tune_state.axis = AUTO_TUNE_AXIS_ROLL;
    auto_tune_state.positive_direction = false;
    auto_tune_state.step = AUTO_TUNE_STEP_WAITING_FOR_LEVEL;
    auto_tune_timer = millis();
    auto_tune_state.tune_type = AUTO_TUNE_TYPE_RD_UP;

    // backup original pids
    orig_roll_rp = g.pid_rate_roll.kP();
    orig_roll_ri = g.pid_rate_roll.kI();
    orig_roll_rd = g.pid_rate_roll.kD();
    orig_roll_sp = g.pi_stabilize_roll.kP();
    orig_pitch_rp = g.pid_rate_pitch.kP();
    orig_pitch_ri = g.pid_rate_pitch.kI();
    orig_pitch_rd = g.pid_rate_pitch.kD();
    orig_pitch_sp = g.pi_stabilize_pitch.kP();

    // initialise tuned pid values
    tune_roll_rp = g.pid_rate_roll.kP();
    tune_roll_rd = g.pid_rate_roll.kD();
    tune_roll_sp = g.pi_stabilize_roll.kP();
    tune_pitch_rp = g.pid_rate_pitch.kP();
    tune_pitch_rd = g.pid_rate_pitch.kD();
    tune_pitch_sp = g.pi_stabilize_pitch.kP();

    Log_Write_Event(DATA_AUTOTUNE_INITIALISED);
}

// auto_tune_intra_test_gains - gains used between tests
static void auto_tune_intra_test_gains()
{
    // we are restarting tuning so reset gains to tuning-start gains (i.e. low I term)
    g.pid_rate_roll.kP(orig_roll_rp);
    g.pid_rate_roll.kI(orig_roll_rp*AUTO_TUNE_PI_RATIO_FOR_TESTING);
    g.pid_rate_roll.kD(orig_roll_rd);
    g.pi_stabilize_roll.kP(orig_roll_sp);
    g.pid_rate_pitch.kP(orig_pitch_rp);
    g.pid_rate_pitch.kI(orig_pitch_rp*AUTO_TUNE_PI_RATIO_FOR_TESTING);
    g.pid_rate_pitch.kD(orig_pitch_rd);
    g.pi_stabilize_pitch.kP(orig_pitch_sp);

    // re-enable the rate limits
    ap.disable_stab_rate_limit = false;
}

// auto_tune_restore_orig_gains - restore pids to their original values
static void auto_tune_restore_orig_gains()
{
    g.pid_rate_roll.kP(orig_roll_rp);
    g.pid_rate_roll.kI(orig_roll_ri);
    g.pid_rate_roll.kD(orig_roll_rd);
    g.pi_stabilize_roll.kP(orig_roll_sp);
    g.pid_rate_pitch.kP(orig_pitch_rp);
    g.pid_rate_pitch.kI(orig_pitch_ri);
    g.pid_rate_pitch.kD(orig_pitch_rd);
    g.pi_stabilize_pitch.kP(orig_pitch_sp);
    ap.disable_stab_rate_limit = false;
}

// auto_tune_load_tuned_pids - restore pids to their tuned values
static void auto_tune_load_tuned_gains()
{
    if (tune_roll_rp != 0 && tune_pitch_rp != 0) {
        g.pid_rate_roll.kP(tune_roll_rp);
        g.pid_rate_roll.kI(tune_roll_rp*AUTO_TUNE_RP_RATIO_FINAL);
        g.pid_rate_roll.kD(tune_roll_rd);
        g.pi_stabilize_roll.kP(tune_roll_sp);
        g.pid_rate_pitch.kP(tune_pitch_rp);
        g.pid_rate_pitch.kI(tune_pitch_rp*AUTO_TUNE_RP_RATIO_FINAL);
        g.pid_rate_pitch.kD(tune_pitch_rd);
        g.pi_stabilize_pitch.kP(tune_pitch_sp);
        ap.disable_stab_rate_limit = false;
    }
}

// auto_tune_load_test_gains - load the to-be-tested gains for a single axis
static void auto_tune_load_test_gains()
{
    // restore pids to their tuning values
    if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
        g.pid_rate_roll.kP(tune_roll_rp);
        g.pid_rate_roll.kI(tune_roll_rp*0.01f);
        g.pid_rate_roll.kD(tune_roll_rd);
        g.pi_stabilize_roll.kP(tune_roll_sp);
    }else{
        g.pid_rate_pitch.kP(tune_pitch_rp);
        g.pid_rate_pitch.kI(tune_pitch_rp*0.01f);
        g.pid_rate_pitch.kD(tune_pitch_rd);
        g.pi_stabilize_pitch.kP(tune_pitch_sp);
    }
    ap.disable_stab_rate_limit = true;  // disable rate limits
}

// start an auto tuning session
// returns true if we should change the roll-pitch mode to ROLL_PITCH_AUTOTUNE
// To-Do: make autotune a flight mode so that this slightly non-intuitive returning of a flag is not required
static bool auto_tune_start()
{
    bool requires_autotune_rp_mode = false;

    switch (auto_tune_state.mode) {
        case AUTO_TUNE_MODE_UNINITIALISED:
            // initialise gains and axis because this is our first time
            auto_tune_initialise();
            auto_tune_update_gcs(AUTO_TUNE_MESSAGE_STARTED);
            auto_tune_state.mode = AUTO_TUNE_MODE_TUNING;
            requires_autotune_rp_mode = true;
            break;
        case AUTO_TUNE_MODE_TUNING:
            // we are restarting tuning so reset gains to tuning-start gains (i.e. low I term)
            auto_tune_intra_test_gains();
            Log_Write_Event(DATA_AUTOTUNE_RESTART);
            auto_tune_update_gcs(AUTO_TUNE_MESSAGE_STARTED);
            requires_autotune_rp_mode = true;
            break;
        case AUTO_TUNE_MODE_TESTING:
            // we have completed a tune and are testing the new gains
            auto_tune_load_tuned_gains();
            Log_Write_Event(DATA_AUTOTUNE_TESTING);
            requires_autotune_rp_mode = false;
            break;
        case AUTO_TUNE_MODE_FAILED:
            Log_Write_Event(DATA_AUTOTUNE_ABANDONED);
            requires_autotune_rp_mode = false;
            break;
    }

    // tell caller we require roll-pitch mode to be changed to ROLL_PITCH_AUTOTUNE
    return requires_autotune_rp_mode;
}

// turn off tuning and return to standard pids
static void auto_tune_stop()
{
    ap.disable_stab_rate_limit = false;
    rate_targets_frame = EARTH_FRAME;   // regular stabilize mode frame
    // restore pids to their original values
    auto_tune_restore_orig_gains();
    Log_Write_Event(DATA_AUTOTUNE_OFF);
}

// save discovered gains to eeprom if auto tuner is enabled (i.e. switch is in middle or high position)
static void auto_tune_save_tuning_gains_and_reset()
{
    if (auto_tune_state.mode == AUTO_TUNE_MODE_TESTING) {
        auto_tune_load_tuned_gains();
        g.pid_rate_roll.save_gains();
        g.pid_rate_pitch.save_gains();
        g.pi_stabilize_roll.save_gains();
        g.pi_stabilize_pitch.save_gains();
        orig_roll_rp = g.pid_rate_roll.kP();
        orig_roll_ri = g.pid_rate_roll.kI();
        orig_roll_rd = g.pid_rate_roll.kD();
        orig_roll_sp = g.pi_stabilize_roll.kP();
        orig_pitch_rp = g.pid_rate_pitch.kP();
        orig_pitch_ri = g.pid_rate_pitch.kI();
        orig_pitch_rd = g.pid_rate_pitch.kD();
        orig_pitch_sp = g.pi_stabilize_pitch.kP();
        Log_Write_Event(DATA_AUTOTUNE_SAVEDGAINS);
    }
    // reset state of autotune
    auto_tune_state.mode = AUTO_TUNE_MODE_UNINITIALISED;
}

// send message to ground station
void auto_tune_update_gcs(uint8_t message_id)
{
    switch (message_id) {
        case AUTO_TUNE_MESSAGE_STARTED:
            gcs_send_text_P(SEVERITY_HIGH,PSTR("AutoTune: Started"));
            break;
        case AUTO_TUNE_MESSAGE_SUCCESS:
            gcs_send_text_P(SEVERITY_HIGH,PSTR("AutoTune: Success"));
            break;
        case AUTO_TUNE_MESSAGE_FAILED:
            gcs_send_text_P(SEVERITY_HIGH,PSTR("AutoTune: Failed"));
            break;
    }
}

// Auto tuning roll-pitch controller
static void
get_autotune_roll_pitch_controller(int16_t pilot_roll_angle, int16_t pilot_pitch_angle, int16_t pilot_yaw_command)
{
    int32_t target_roll_rate, target_pitch_rate;
    float rotation_rate;        // rotation rate in radians/second
    int32_t lean_angle;

    // exit immediately if not actively tuning
    if (!auto_tune_state.mode == AUTO_TUNE_MODE_TUNING) {
        return;
    }

    // check for pilot override
    if (pilot_roll_angle != 0 || pilot_pitch_angle != 0 || pilot_yaw_command != 0) {
        if (!auto_tune_state.pilot_override) {
            // restore pids to their original values
            auto_tune_restore_orig_gains();
        }
        auto_tune_state.pilot_override = true;
        auto_tune_override_time = millis();
    }else if (auto_tune_state.pilot_override) {
        // check if we should resume tuning after pilot's override
        if (millis() - auto_tune_override_time > AUTO_TUNE_PILOT_OVERRIDE_TIMEOUT_MS) {
            auto_tune_state.pilot_override = false;             // turn off pilot override
            auto_tune_state.step = AUTO_TUNE_STEP_WAITING_FOR_LEVEL; // set tuning step back from beginning
            auto_tune_timer = millis();
        }
    }

    // check tuning step
    if (!auto_tune_state.pilot_override) {
        switch (auto_tune_state.step) {
            case AUTO_TUNE_STEP_WAITING_FOR_LEVEL:
                // reset counter if we are no longer level
                if ((labs(ahrs.roll_sensor) > AUTO_TUNE_LEVEL_ANGLE_CD) || (labs(ahrs.pitch_sensor) > AUTO_TUNE_LEVEL_ANGLE_CD)) {
                    auto_tune_timer = millis();
                }

                // if we have been level for a sufficient amount of time (0.5 seconds) move onto next step
                if (millis() - auto_tune_timer >= AUTO_TUNE_REQUIRED_LEVEL_TIME_MS) {
                    auto_tune_state.step = AUTO_TUNE_STEP_TESTING;
                    // init variables for next step
                    auto_tune_test_max = 0;
                    auto_tune_test_min = 0;
                    rotation_rate = 0;
                    auto_tune_timer = millis();
                    // initialise rate controller targets
                    acro_roll_rate = roll_rate_target_bf;
                    acro_pitch_rate = pitch_rate_target_bf;
                    acro_yaw_rate = yaw_rate_target_bf;
                    // restore pids to their to-be-tested values
                    auto_tune_load_test_gains();
                    ap.disable_stab_rate_limit = true;  // disable rate limits
                }
                break;

            case AUTO_TUNE_STEP_TESTING:

                // Run the test
                // update rotation targets in body-earth frame
                if(auto_tune_state.tune_type == AUTO_TUNE_TYPE_SP_UP){
                    if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                        // override roll angle
                        if (auto_tune_state.positive_direction) {
                            control_roll = AUTO_TUNE_TARGET_ANGLE_CD;
                        }else{
                            control_roll = -AUTO_TUNE_TARGET_ANGLE_CD;
                        }
                        get_stabilize_roll(control_roll);
                    }else{
                        // override pitch angle
                        if (auto_tune_state.positive_direction) {
                            control_pitch = AUTO_TUNE_TARGET_ANGLE_CD;
                        }else{
                            control_pitch = -AUTO_TUNE_TARGET_ANGLE_CD;
                        }
                        get_stabilize_pitch(control_pitch);
                    }
                } else {
                    if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                        // override roll rate
                        if (auto_tune_state.positive_direction) {
                            target_roll_rate = AUTO_TUNE_TARGET_RATE_CDS;
                        }else{
                            target_roll_rate = -AUTO_TUNE_TARGET_RATE_CDS;
                        }
                        // set body frame targets for rate controller
                        set_roll_rate_target(target_roll_rate, BODY_FRAME);
                        set_pitch_rate_target(acro_pitch_rate, BODY_FRAME);
                        set_yaw_rate_target(acro_yaw_rate, BODY_FRAME);
                    }else{
                        // override pitch rate
                        if (auto_tune_state.positive_direction) {
                            target_pitch_rate = AUTO_TUNE_TARGET_RATE_CDS;
                        }else{
                            target_pitch_rate = -AUTO_TUNE_TARGET_RATE_CDS;
                        }
                        // set body frame targets for rate controller
                        set_pitch_rate_target(target_pitch_rate, BODY_FRAME);
                        set_roll_rate_target(acro_roll_rate, BODY_FRAME);
                        set_yaw_rate_target(acro_yaw_rate, BODY_FRAME);
                    }
                    rate_targets_frame = BODY_EARTH_FRAME;
                }
                
                
                // Get Rate and Angle
                if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                    // 20 Hz filter on rate
                    rotation_rate = ToDeg(fabs(ahrs.get_gyro().x)) * 100.0f;
                    lean_angle = labs(ahrs.roll_sensor);
                }else{
                    // 20 Hz filter on rate
                    // rotation_rate = rotation_rate + 0.55686f*(ToDeg(fabs(ahrs.get_gyro().y))*100.0f-rotation_rate);
                    rotation_rate = ToDeg(fabs(ahrs.get_gyro().y)) * 100.0f;
                    lean_angle = labs(ahrs.pitch_sensor);
                }

                // Make measurements
                if(auto_tune_state.tune_type == AUTO_TUNE_TYPE_SP_UP){
                    // capture max angle
                    if (lean_angle > auto_tune_test_max) {
                        auto_tune_test_max = lean_angle;
                        auto_tune_test_min = lean_angle;
                    }
                    
                    // capture min rotation rate
                    if (lean_angle < auto_tune_test_min && auto_tune_test_max > AUTO_TUNE_TARGET_ANGLE_CD*(1-AUTO_TUNE_AGGRESSIVENESS)) {
                        auto_tune_test_min = lean_angle;
                    }
                }else{
                    // capture max rotation rate
                    if (rotation_rate > auto_tune_test_max) {
                        auto_tune_test_max = rotation_rate;
                        auto_tune_test_min = rotation_rate;
                    }
                    
                    // capture min rotation rate
                    if (rotation_rate < auto_tune_test_min && auto_tune_test_max > AUTO_TUNE_TARGET_RATE_CDS*(1-2*AUTO_TUNE_AGGRESSIVENESS)) {
                        auto_tune_test_min = rotation_rate;
                    }
                }
        
                // check for end of test conditions
                if(millis() - auto_tune_timer >= AUTO_TUNE_TARGET_RATE_TIMEOUT_MS) {
                        auto_tune_state.step = AUTO_TUNE_STEP_UPDATE_GAINS;
                }
                if(auto_tune_state.tune_type == AUTO_TUNE_TYPE_SP_UP){
                    if ((lean_angle >= AUTO_TUNE_TARGET_ANGLE_CD*(1+AUTO_TUNE_AGGRESSIVENESS)) || 
                            (auto_tune_test_max-auto_tune_test_min > AUTO_TUNE_TARGET_ANGLE_CD*AUTO_TUNE_AGGRESSIVENESS)) {
                        auto_tune_state.step = AUTO_TUNE_STEP_UPDATE_GAINS;
                    }
                }else{
                    if (lean_angle >= AUTO_TUNE_TARGET_ANGLE_CD) {
                        auto_tune_state.step = AUTO_TUNE_STEP_UPDATE_GAINS;
                    }
                    if (auto_tune_state.tune_type == AUTO_TUNE_TYPE_RD_UP || auto_tune_state.tune_type == AUTO_TUNE_TYPE_RD_DOWN) {
                        if(auto_tune_test_max-auto_tune_test_min > AUTO_TUNE_TARGET_RATE_CDS*AUTO_TUNE_AGGRESSIVENESS) {
                            auto_tune_state.step = AUTO_TUNE_STEP_UPDATE_GAINS;
                        }
                    }
                }

                // logging
                Log_Write_AutoTuneDetails((int16_t)lean_angle, rotation_rate);
                break;

            case AUTO_TUNE_STEP_UPDATE_GAINS:
                // restore gains to their intra-test values
                auto_tune_intra_test_gains();

                // logging
                if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                    Log_Write_AutoTune(auto_tune_state.axis, auto_tune_state.tune_type, auto_tune_test_min, auto_tune_test_max, tune_roll_rp, tune_roll_rd, tune_roll_sp);
                }else{
                    Log_Write_AutoTune(auto_tune_state.axis, auto_tune_state.tune_type, auto_tune_test_min, auto_tune_test_max, tune_pitch_rp, tune_pitch_rd, tune_pitch_sp);
                }

                // do gain updates
                if (auto_tune_state.tune_type == AUTO_TUNE_TYPE_RD_UP) {
                    if (auto_tune_test_max > AUTO_TUNE_TARGET_RATE_CDS) {
                        if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                            tune_roll_rp -= AUTO_TUNE_RP_STEP;
                        }else{
                            tune_pitch_rp -= AUTO_TUNE_RP_STEP;
                        }
                        // stop the auto tune if we have hit the minimum roll or pitch rate P
                        if(((auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL && tune_roll_rp < AUTO_TUNE_RP_MIN) ||
                            (auto_tune_state.axis == AUTO_TUNE_AXIS_PITCH && tune_pitch_rp < AUTO_TUNE_RP_MIN)) ) {
                            auto_tune_state.mode = AUTO_TUNE_MODE_FAILED;
                            Log_Write_Event(DATA_AUTOTUNE_ABANDONED);
                            set_roll_pitch_mode(ROLL_PITCH_STABLE);
                            auto_tune_update_gcs(AUTO_TUNE_MESSAGE_FAILED);
                            return;
                        }
                    }else if(auto_tune_test_max < AUTO_TUNE_TARGET_RATE_CDS*(1.0f-AUTO_TUNE_AGGRESSIVENESS*2.0f) && 
                            ((auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL && tune_roll_rp <= AUTO_TUNE_RP_MAX) ||
                            (auto_tune_state.axis == AUTO_TUNE_AXIS_PITCH && tune_pitch_rp <= AUTO_TUNE_RP_MAX)) ) {
                        if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                            tune_roll_rp += AUTO_TUNE_RP_STEP*2.0f;
                        }else{
                            tune_pitch_rp += AUTO_TUNE_RP_STEP*2.0f;
                        }
                    }else{
                        if (auto_tune_test_max-auto_tune_test_min > AUTO_TUNE_TARGET_RATE_CDS*AUTO_TUNE_AGGRESSIVENESS) {
                            auto_tune_counter++;
                        }else{
                            if (auto_tune_counter > 0 ) {
                                auto_tune_counter--;
                            }
                            if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                                tune_roll_rd += AUTO_TUNE_RD_STEP*2.0f;
                                // stop tuning if we hit max D
                                if (tune_roll_rd >= AUTO_TUNE_RD_MAX) {
                                    tune_roll_rd = AUTO_TUNE_RD_MAX;
                                    auto_tune_counter = AUTO_TUNE_SUCCESS_COUNT;
                                    Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                                }
                            }else{
                                tune_pitch_rd += AUTO_TUNE_RD_STEP*2.0f;
                                // stop tuning if we hit max D
                                if (tune_pitch_rd >= AUTO_TUNE_RD_MAX) {
                                    tune_pitch_rd = AUTO_TUNE_RD_MAX;
                                    auto_tune_counter = AUTO_TUNE_SUCCESS_COUNT;
                                    Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                                }
                            }
                        }
                    }
                } else if (auto_tune_state.tune_type == AUTO_TUNE_TYPE_RD_DOWN) {
                    if (auto_tune_test_max > AUTO_TUNE_TARGET_RATE_CDS) {
                        if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                            tune_roll_rp -= AUTO_TUNE_RP_STEP;
                        }else{
                            tune_pitch_rp -= AUTO_TUNE_RP_STEP;
                        }
                        // stop the auto tune if we have hit the minimum roll or pitch rate P
                        if(((auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL && tune_roll_rp < AUTO_TUNE_RP_MIN) ||
                            (auto_tune_state.axis == AUTO_TUNE_AXIS_PITCH && tune_pitch_rp < AUTO_TUNE_RP_MIN)) ) {
                            auto_tune_state.mode = AUTO_TUNE_MODE_FAILED;
                            Log_Write_Event(DATA_AUTOTUNE_ABANDONED);
                            set_roll_pitch_mode(ROLL_PITCH_STABLE);
                            auto_tune_update_gcs(AUTO_TUNE_MESSAGE_FAILED);
                            return;
                        }
                    }else if(auto_tune_test_max < AUTO_TUNE_TARGET_RATE_CDS*(1-AUTO_TUNE_AGGRESSIVENESS*2.0f) && 
                            ((auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL && tune_roll_rp <= AUTO_TUNE_RP_MAX) ||
                            (auto_tune_state.axis == AUTO_TUNE_AXIS_PITCH && tune_pitch_rp <= AUTO_TUNE_RP_MAX)) ) {
                        if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                            tune_roll_rp += AUTO_TUNE_RP_STEP;
                        }else{
                            tune_pitch_rp += AUTO_TUNE_RP_STEP;
                        }
                    }else{
                        if (auto_tune_test_max-auto_tune_test_min < AUTO_TUNE_TARGET_RATE_CDS*AUTO_TUNE_AGGRESSIVENESS) {
                            auto_tune_counter++;
                        }else{
                            if (auto_tune_counter > 0 ) {
                                auto_tune_counter--;
                            }
                            if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                                tune_roll_rd -= AUTO_TUNE_RD_STEP;
                                // stop tuning if we hit max D
                                if (tune_roll_rd <= AUTO_TUNE_RD_MIN) {
                                    tune_roll_rd = AUTO_TUNE_RD_MIN;
                                    auto_tune_counter = AUTO_TUNE_SUCCESS_COUNT;
                                    Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                                }
                            }else{
                                tune_pitch_rd -= AUTO_TUNE_RD_STEP;
                                // stop tuning if we hit max D
                                if (tune_pitch_rd <= AUTO_TUNE_RD_MIN) {
                                    tune_pitch_rd = AUTO_TUNE_RD_MIN;
                                    auto_tune_counter = AUTO_TUNE_SUCCESS_COUNT;
                                    Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                                }
                            }
                        }
                    }
                } else if (auto_tune_state.tune_type == AUTO_TUNE_TYPE_RP_UP) {
                    if (auto_tune_test_max > AUTO_TUNE_TARGET_RATE_CDS) {
                        auto_tune_counter++;
                    }else{
                        if (auto_tune_counter > 0 ) {
                            auto_tune_counter--;
                        }
                        // increase P & I or D term
                        // update PI term
                        if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                            tune_roll_rp += AUTO_TUNE_RP_STEP;
                            // stop tuning if we hit max P
                            if (tune_roll_rp >= AUTO_TUNE_RP_MAX) {
                                tune_roll_rp = AUTO_TUNE_RP_MAX;
                                auto_tune_counter = AUTO_TUNE_SUCCESS_COUNT;
                                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                            }
                        }else{
                            tune_pitch_rp += AUTO_TUNE_RP_STEP;
                            // stop tuning if we hit max P
                            if (tune_pitch_rp >= AUTO_TUNE_RP_MAX) {
                                tune_pitch_rp = AUTO_TUNE_RP_MAX;
                                auto_tune_counter = AUTO_TUNE_SUCCESS_COUNT;
                                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                            }
                        }
                    }
                } else if (auto_tune_state.tune_type == AUTO_TUNE_TYPE_SP_UP) {
                    if (auto_tune_test_max > AUTO_TUNE_TARGET_ANGLE_CD*(1+AUTO_TUNE_AGGRESSIVENESS) ||
                            (auto_tune_test_max-auto_tune_test_min > AUTO_TUNE_TARGET_ANGLE_CD*AUTO_TUNE_AGGRESSIVENESS)) {
                        auto_tune_counter++;
                    }else{
                        if (auto_tune_counter > 0 ) {
                            auto_tune_counter--;
                        }
                        // increase P & I or D term
                        // update PI term
                        if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                            tune_roll_sp += AUTO_TUNE_SP_STEP;
                            // stop tuning if we hit max P
                            if (tune_roll_sp >= AUTO_TUNE_SP_MAX) {
                                tune_roll_sp = AUTO_TUNE_SP_MAX;
                                auto_tune_counter = AUTO_TUNE_SUCCESS_COUNT;
                                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                            }
                        }else{
                            tune_pitch_sp += AUTO_TUNE_SP_STEP;
                            // stop tuning if we hit max P
                            if (tune_pitch_sp >= AUTO_TUNE_SP_MAX) {
                                tune_pitch_sp = AUTO_TUNE_SP_MAX;
                                auto_tune_counter = AUTO_TUNE_SUCCESS_COUNT;
                                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                            }
                        }
                    }
                }

                // reverse direction
                auto_tune_state.positive_direction = !auto_tune_state.positive_direction;

                // we've complete this step, finalise pids and move to next step
                if (auto_tune_counter >= AUTO_TUNE_SUCCESS_COUNT) {

                    // reset counter
                    auto_tune_counter = 0;

                    // move to the next tuning type
                    if (auto_tune_state.tune_type < AUTO_TUNE_TYPE_SP_UP) {
                        auto_tune_state.tune_type++;
                    }else{
                        // we've reached the end of a D-up-down PI-up-down tune type cycle
                        auto_tune_state.tune_type = AUTO_TUNE_TYPE_RD_UP;
                        
                        // if we've just completed roll move onto pitch
                        if (auto_tune_state.axis == AUTO_TUNE_AXIS_ROLL) {
                            tune_roll_sp = tune_roll_sp * AUTO_TUNE_SP_BACKOFF;
                            auto_tune_state.axis = AUTO_TUNE_AXIS_PITCH;
                        }else{
                            tune_pitch_sp = tune_pitch_sp * AUTO_TUNE_SP_BACKOFF;
                            tune_roll_sp = min(tune_roll_sp, tune_pitch_sp);
                            tune_pitch_sp = min(tune_roll_sp, tune_pitch_sp);
                            // if we've just completed pitch we are done tuning and are moving onto testing
                            auto_tune_state.mode = AUTO_TUNE_MODE_TESTING;
                            auto_tune_update_gcs(AUTO_TUNE_MESSAGE_SUCCESS);
                            Log_Write_Event(DATA_AUTOTUNE_COMPLETE);
                            set_roll_pitch_mode(ROLL_PITCH_STABLE);
                        }
                    }
                }

                // reset testing step
                auto_tune_state.step = AUTO_TUNE_STEP_WAITING_FOR_LEVEL;
                auto_tune_timer = millis();
                break;
        }
    }
}
#endif  // AUTOTUNE == ENABLED
#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/climb_rate.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if 0 // currently unused

struct DataPoint {
    unsigned long x;
    long y;
};

DataPoint history[ALTITUDE_HISTORY_LENGTH];               // Collection of (x,y) points to regress a rate of change from
unsigned char hindex;   // Index in history for the current data point

unsigned long xoffset;
unsigned char n;

// Intermediate variables for regression calculation
long xi;
long yi;
long xiyi;
unsigned long xi2;

void add_altitude_data(unsigned long xl, long y)
{
    //Reset the regression if our X variable overflowed
    if (xl < xoffset)
        n = 0;

    //To allow calculation of sum(xi*yi), make sure X hasn't exceeded 2^32/2^15/length
    if (xl - xoffset > 131072/ALTITUDE_HISTORY_LENGTH)
        n = 0;

    if (n == ALTITUDE_HISTORY_LENGTH) {
        xi -= history[hindex].x;
        yi -= history[hindex].y;
        xiyi -= (long)history[hindex].x * history[hindex].y;
        xi2 -= history[hindex].x * history[hindex].x;
    } else {
        if (n == 0) {
            xoffset = xl;
            xi = 0;
            yi = 0;
            xiyi = 0;
            xi2 = 0;
        }
        n++;
    }

    history[hindex].x = xl - xoffset;
    history[hindex].y = y;

    xi += history[hindex].x;
    yi += history[hindex].y;
    xiyi += (long)history[hindex].x * history[hindex].y;
    xi2 += history[hindex].x * history[hindex].x;

    if (++hindex >= ALTITUDE_HISTORY_LENGTH)
        hindex = 0;
}
#endif

#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/commands.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void init_commands()
{
    g.command_index         = NO_COMMAND;
    command_nav_index       = NO_COMMAND;
    command_cond_index      = NO_COMMAND;
    prev_nav_index          = NO_COMMAND;
    command_cond_queue.id   = NO_COMMAND;
    command_nav_queue.id    = NO_COMMAND;
}
/*
 *  logic for dealing with the current command in the mission and home location
 */

static void plane_init_commands()
{
    g.command_index.set_and_save(0);
    nav_command_ID  = NO_COMMAND;
    non_nav_command_ID      = NO_COMMAND;
    next_nav_command.id     = CMD_BLANK;
    nav_command_index = 0;
}

static void update_auto() // plane
{
    if (g.command_index >= g.command_total) {
        handle_no_commands();
        if(g.command_total == 0) {
            next_WP.lat             = home.lat + 1000;                  // so we don't have bad calcs
            next_WP.lng             = home.lng + 1000;                  // so we don't have bad calcs
        }
    } else {
        if(g.command_index != 0) {
            g.command_index = nav_command_index;
            nav_command_index--;
        }
        nav_command_ID  = NO_COMMAND;
        non_nav_command_ID      = NO_COMMAND;
        next_nav_command.id     = CMD_BLANK;
        process_next_command();
    }
}

// Getters
// -------
static struct Location get_cmd_with_index(int i)
{
    struct Location temp;

    // Find out proper location in memory by using the start_byte position + the index
    // --------------------------------------------------------------------------------
    if (i >= g.command_total) {
        // we do not have a valid command to load
        // return a WP with a "Blank" id
        temp.id = CMD_BLANK;

        // no reason to carry on
        return temp;

    }else{
        // we can load a command, we don't process it yet
        // read WP position
        uint16_t mem = (WP_START_BYTE) + (i * WP_SIZE);

        temp.id = hal.storage->read_byte(mem);

        mem++;
        temp.options = hal.storage->read_byte(mem);

        mem++;
        temp.p1 = hal.storage->read_byte(mem);

        mem++;
        temp.alt = hal.storage->read_dword(mem);           // alt is stored in CM! Alt is stored relative!

        mem += 4;
        temp.lat = hal.storage->read_dword(mem);         // lat is stored in decimal * 10,000,000

        mem += 4;
        temp.lng = hal.storage->read_dword(mem);         // lon is stored in decimal * 10,000,000
    }

    // Add on home altitude if we are a nav command (or other command with altitude) and stored alt is relative
    //if((temp.id < MAV_CMD_NAV_LAST || temp.id == MAV_CMD_CONDITION_CHANGE_ALT) && (temp.options & MASK_OPTIONS_RELATIVE_ALT)){
    //temp.alt += home.alt;
    //}

    if(temp.options & WP_OPTION_RELATIVE) {
        // If were relative, just offset from home
        temp.lat        +=      home.lat;
        temp.lng        +=      home.lng;
    }

    return temp;
}

/*
  fetch a mission item from EEPROM
*/
static struct Location get_cmd_with_index_raw(int16_t i) // plane
{
    struct Location temp;
    uint16_t mem;

    // Find out proper location in memory by using the start_byte position + the index
    // --------------------------------------------------------------------------------
    if (i > g.command_total) {
        memset(&temp, 0, sizeof(temp));
        temp.id = CMD_BLANK;
    }else{
        // read WP position
        mem = (WP_START_BYTE) + (i * WP_SIZE);
        temp.id = hal.storage->read_byte(mem);

        mem++;
        temp.options = hal.storage->read_byte(mem);

        mem++;
        temp.p1 = hal.storage->read_byte(mem);

        mem++;
        temp.alt = hal.storage->read_dword(mem);

        mem += 4;
        temp.lat = hal.storage->read_dword(mem);

        mem += 4;
        temp.lng = hal.storage->read_dword(mem);
    }

    return temp;
}

/*
  fetch a mission item from EEPROM. Adjust altitude to be absolute
*/
static struct Location plane_get_cmd_with_index(int16_t i)
{
    struct Location temp;

    temp = get_cmd_with_index_raw(i);

    // Add on home altitude if we are a nav command (or other command with altitude) and stored alt is relative
    if ((temp.id < MAV_CMD_NAV_LAST || temp.id == MAV_CMD_CONDITION_CHANGE_ALT) &&
        (temp.options & MASK_OPTIONS_RELATIVE_ALT) &&
        (temp.lat != 0 || temp.lng != 0 || temp.alt != 0)) {
        temp.alt += home.alt;
    }

    return temp;
}

// Setters
// -------
static void set_cmd_with_index(struct Location temp, int i)
{

    i = constrain_int16(i, 0, g.command_total.get());

    // store home as 0 altitude!!!
    // Home is always a MAV_CMD_NAV_WAYPOINT (16)
    if (i == 0) {
        temp.alt = 0;
        temp.id = MAV_CMD_NAV_WAYPOINT;
    }

    uint16_t mem = WP_START_BYTE + (i * WP_SIZE);

    hal.storage->write_byte(mem, temp.id);

    mem++;
    hal.storage->write_byte(mem, temp.options);

    mem++;
    hal.storage->write_byte(mem, temp.p1);

    mem++;
    hal.storage->write_dword(mem, temp.alt);     // Alt is stored in CM!

    mem += 4;
    hal.storage->write_dword(mem, temp.lat);     // Lat is stored in decimal degrees * 10^7

    mem += 4;
    hal.storage->write_dword(mem, temp.lng);     // Long is stored in decimal degrees * 10^7

    // Make sure our WP_total
    if(g.command_total < (i+1))
        g.command_total.set_and_save(i+1);
}

static void plane_set_cmd_with_index(struct Location temp, int16_t i)
{
    i = constrain_int16(i, 0, g.command_total.get());
    uint16_t mem = WP_START_BYTE + (i * WP_SIZE);

    // force home wp to absolute height
    if (i == 0) {
        temp.options &= ~(MASK_OPTIONS_RELATIVE_ALT);
    }
    // zero unused bits
    temp.options &= (MASK_OPTIONS_RELATIVE_ALT | MASK_OPTIONS_LOITER_DIRECTION);

    hal.storage->write_byte(mem, temp.id);

    mem++;
    hal.storage->write_byte(mem, temp.options);

    mem++;
    hal.storage->write_byte(mem, temp.p1);

    mem++;
    hal.storage->write_dword(mem, temp.alt);

    mem += 4;
    hal.storage->write_dword(mem, temp.lat);

    mem += 4;
    hal.storage->write_dword(mem, temp.lng);
}

static int32_t read_alt_to_hold()  // Plane
{
    if (g.RTL_altitude_cm < 0) {
        return current_loc.alt;
    }
    return g.RTL_altitude_cm + home.alt;
}

/*
 *  This function stores waypoint commands
 *  It looks to see what the next command type is and finds the last command.
 */
static void set_next_WP(const struct Location *wp)  // Plane
{
    // copy the current WP into the OldWP slot
    // ---------------------------------------
    prev_WP = next_WP;

    // Load the next_WP slot
    // ---------------------
    next_WP = *wp;

    // if lat and lon is zero, then use current lat/lon
    // this allows a mission to contain a "loiter on the spot"
    // command
    if (next_WP.lat == 0 && next_WP.lng == 0) {
        next_WP.lat = current_loc.lat;
        next_WP.lng = current_loc.lng;
        // additionally treat zero altitude as current altitude
        if (next_WP.alt == 0) {
            next_WP.alt = current_loc.alt;
        }
    }


    // are we already past the waypoint? This happens when we jump
    // waypoints, and it can cause us to skip a waypoint. If we are
    // past the waypoint when we start on a leg, then use the current
    // location as the previous waypoint, to prevent immediately
    // considering the waypoint complete
    if (location_passed_point(current_loc, prev_WP, next_WP)) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("Resetting prev_WP"));
        prev_WP = current_loc;
    }

    // used to control FBW and limit the rate of climb
    // -----------------------------------------------
    target_altitude_cm = current_loc.alt;

    // zero out our loiter vals to watch for missed waypoints
    loiter_angle_reset();

    setup_glide_slope();

    loiter_angle_reset();
}

static void set_guided_WP(void)  // Plane // see also workaround in GCS MAVLINK
{
    if (g.loiter_radius < 0) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }

    // copy the current location into the OldWP slot
    // ---------------------------------------
    prev_WP = current_loc;

    // Load the next_WP slot
    // ---------------------
    next_WP = guided_WP;

    // used to control FBW and limit the rate of climb
    // -----------------------------------------------
    target_altitude_cm = current_loc.alt;

    setup_glide_slope();

    loiter_angle_reset();
}

static int32_t get_RTL_alt()
{
    if(g.rtl_altitude <= 0) {
		return min(current_loc.alt, RTL_ALT_MAX);
    }else if (g.rtl_altitude < current_loc.alt) {
		return min(current_loc.alt, RTL_ALT_MAX);
    }else{
        return g.rtl_altitude;
    }
}

// run this at setup on the ground
// -------------------------------
static void init_home()
{
    set_home_is_set(true);
    home.id         = MAV_CMD_NAV_WAYPOINT;
    home.lng        = g_gps->longitude;                                 // Lon * 10**7
    home.lat        = g_gps->latitude;                                  // Lat * 10**7
    home.alt        = 0;                                                        // Home is always 0


    // Save Home to EEPROM
    // -------------------
    // no need to save this to EPROM
    set_cmd_with_index(home, 0);

    // set inertial nav's home position
    inertial_nav.set_home_position(g_gps->longitude, g_gps->latitude);

    if (g.log_bitmask & MASK_LOG_CMD)
        Log_Write_Cmd(0, &home);

    // update navigation scalers.  used to offset the shrinking longitude as we go towards the poles
    scaleLongDown = longitude_scale(home);
    scaleLongUp   = 1.0f/scaleLongDown;
}

static void plane_init_home()
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("init home"));

    // block until we get a good fix
    // -----------------------------
    while (!g_gps->new_data || !g_gps->fix) {
        g_gps->update();
#if HIL_MODE != HIL_MODE_DISABLED
        // update hil gps so we have new_data
        gcs_check_input();
#endif
    }

    home.id         = MAV_CMD_NAV_WAYPOINT;
    home.lng        = g_gps->longitude;                                 // Lon * 10**7
    home.lat        = g_gps->latitude;                                  // Lat * 10**7
    home.alt        = max(g_gps->altitude_cm, 0);
    ap.home_is_set = true;

    plane_gcs_send_text_fmt(PSTR("gps alt: %lu"), (unsigned long)home.alt);

    // Save Home to EEPROM - Command 0
    // -------------------
    plane_set_cmd_with_index(home, 0);

    // Save prev loc
    // -------------
    next_WP = prev_WP = home;

    // Load home for a default guided_WP
    // -------------
    guided_WP = home;
    guided_WP.alt += g.RTL_altitude_cm;

}

/*
  update home location from GPS
  this is called as long as we have 3D lock and the arming switch is
  not pushed
*/
static void update_home()  // Plane / see also workaround in ARDUHYBRID
{
    home.lng        = g_gps->longitude;                                 // Lon * 10**7
    home.lat        = g_gps->latitude;                                  // Lat * 10**7
    home.alt        = max(g_gps->altitude_cm, 0);
    barometer.update_calibration();
}
#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/commands_logic.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/********************************************************************************/
// Command Event Handlers
/********************************************************************************/
// process_nav_command - main switch statement to initiate the next nav command in the command_nav_queue
static void process_nav_command()
{
    switch(command_nav_queue.id) {

    case MAV_CMD_NAV_TAKEOFF:                   // 22
        do_takeoff();
        break;

    case MAV_CMD_NAV_WAYPOINT:                  // 16  Navigate to Waypoint
        do_nav_wp();
        break;

    case MAV_CMD_NAV_LAND:              // 21 LAND to Waypoint
        do_land(&command_nav_queue);
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:              // 17 Loiter indefinitely
        do_loiter_unlimited();
        break;

    case MAV_CMD_NAV_LOITER_TURNS:              //18 Loiter N Times
        do_circle();
        break;

    case MAV_CMD_NAV_LOITER_TIME:              // 19
        do_loiter_time();
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:             //20
        do_RTL();
        break;

    default:
        break;
    }

}

// process_cond_command - main switch statement to initiate the next conditional command in the command_cond_queue
static void process_cond_command()
{
    switch(command_cond_queue.id) {

    case MAV_CMD_CONDITION_DELAY:             // 112
        do_wait_delay();
        break;

    case MAV_CMD_CONDITION_DISTANCE:             // 114
        do_within_distance();
        break;

    case MAV_CMD_CONDITION_CHANGE_ALT:             // 113
        do_change_alt();
        break;

    case MAV_CMD_CONDITION_YAW:             // 115
        do_yaw();
        break;

    default:
        break;
    }
}

// process_now_command - main switch statement to initiate the next now command in the command_cond_queue
// now commands are conditional commands that are executed immediately so they do not require a corresponding verify to be run later
static void process_now_command()
{
    switch(command_cond_queue.id) {

    case MAV_CMD_DO_JUMP:              // 177
        do_jump();
        break;

    case MAV_CMD_DO_CHANGE_SPEED:             // 178
        do_change_speed();
        break;

    case MAV_CMD_DO_SET_HOME:             // 179
        do_set_home();
        break;

    case MAV_CMD_DO_SET_SERVO:
        ServoRelayEvents.do_set_servo(command_cond_queue.p1, command_cond_queue.alt);
        break;
        
    case MAV_CMD_DO_SET_RELAY:
        ServoRelayEvents.do_set_relay(command_cond_queue.p1, command_cond_queue.alt);
        break;
        
    case MAV_CMD_DO_REPEAT_SERVO:
        ServoRelayEvents.do_repeat_servo(command_cond_queue.p1, command_cond_queue.alt,
                                         command_cond_queue.lat, command_cond_queue.lng);
        break;
        
    case MAV_CMD_DO_REPEAT_RELAY:
        ServoRelayEvents.do_repeat_relay(command_cond_queue.p1, command_cond_queue.alt,
                                         command_cond_queue.lat);
        break;

    case MAV_CMD_DO_SET_ROI:                // 201
        // point the copter and camera at a region of interest (ROI)
        do_roi();
        break;

#if CAMERA == ENABLED
    case MAV_CMD_DO_CONTROL_VIDEO:                      // Control on-board camera capturing. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|
        break;

    case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|
        break;

    case MAV_CMD_DO_DIGICAM_CONTROL:                    // Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|
        do_take_picture();
        break;

    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
        camera.set_trigger_distance(command_cond_queue.alt);
        break;
#endif

#if MOUNT == ENABLED
    case MAV_CMD_DO_MOUNT_CONFIGURE:                    // Mission command to configure a camera mount |Mount operation mode (see MAV_CONFIGURE_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|
        camera_mount.configure_cmd();
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:                      // Mission command to control a camera mount |pitch(deg*100) or lat, depending on mount mode.| roll(deg*100) or lon depending on mount mode| yaw(deg*100) or alt (in cm) depending on mount mode| Empty| Empty| Empty| Empty|
        camera_mount.control_cmd();
        break;
#endif

    default:
        // do nothing with unrecognized MAVLink messages
        break;
    }
}

static void handle_process_nav_cmd()
{
    // set ap.land_complete to false to stop us zeroing the throttle
    ap.land_complete = false;

    // set takeoff_complete to true so we don't add extra elevator
    // except in a takeoff 
    ap.takeoff_complete = true;

    plane_gcs_send_text_fmt(PSTR("Executing nav command ID #%i"),next_nav_command.id);
    switch(next_nav_command.id) {

    case MAV_CMD_NAV_TAKEOFF:
        plane_do_takeoff();
        break;

    case MAV_CMD_NAV_WAYPOINT:                  // Navigate to Waypoint
        plane_do_nav_wp();
        break;

    case MAV_CMD_NAV_LAND:              // LAND to Waypoint
        plane_do_land();
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:              // Loiter indefinitely
        plane_do_loiter_unlimited();
        break;

    case MAV_CMD_NAV_LOITER_TURNS:              // Loiter N Times
        do_loiter_turns();
        break;

    case MAV_CMD_NAV_LOITER_TIME:
        plane_do_loiter_time();
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        plane_do_RTL();
        break;

    default:
        break;
    }
}

static void handle_process_condition_command()
{
    plane_gcs_send_text_fmt(PSTR("Executing command ID #%i"),next_nonnav_command.id);
    switch(next_nonnav_command.id) {

    case MAV_CMD_CONDITION_DELAY:
        plane_do_wait_delay();
        break;

    case MAV_CMD_CONDITION_DISTANCE:
        plane_do_within_distance();
        break;

    case MAV_CMD_CONDITION_CHANGE_ALT:
        plane_do_change_alt();
        break;

    default:
        break;
    }
}

static void handle_process_do_command()
{
    plane_gcs_send_text_fmt(PSTR("Executing command ID #%i"),next_nonnav_command.id);
    switch(next_nonnav_command.id) {

    case MAV_CMD_DO_JUMP:
        plane_do_jump();
        break;

    case MAV_CMD_DO_CHANGE_SPEED:
        plane_do_change_speed();
        break;

    case MAV_CMD_DO_SET_HOME:
        plane_do_set_home();
        break;

    case MAV_CMD_DO_SET_SERVO:
        ServoRelayEvents.do_set_servo(next_nonnav_command.p1, next_nonnav_command.alt);
        break;

    case MAV_CMD_DO_SET_RELAY:
        ServoRelayEvents.do_set_relay(next_nonnav_command.p1, next_nonnav_command.alt);
        break;

    case MAV_CMD_DO_REPEAT_SERVO:
        ServoRelayEvents.do_repeat_servo(next_nonnav_command.p1, next_nonnav_command.alt,
                                         next_nonnav_command.lat, next_nonnav_command.lng);
        break;

    case MAV_CMD_DO_REPEAT_RELAY:
        ServoRelayEvents.do_repeat_relay(next_nonnav_command.p1, next_nonnav_command.alt,
                                         next_nonnav_command.lat);
        break;

#if CAMERA == ENABLED
    case MAV_CMD_DO_CONTROL_VIDEO:                      // Control on-board camera capturing. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|
        break;

    case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|
        break;

    case MAV_CMD_DO_DIGICAM_CONTROL:                    // Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|
        plane_do_take_picture();
        break;

    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
        camera.set_trigger_distance(next_nonnav_command.alt);
        break;
#endif

#if MOUNT == ENABLED
    // Sets the region of interest (ROI) for a sensor set or the
    // vehicle itself. This can then be used by the vehicles control
    // system to control the vehicle attitude and the attitude of various
    // devices such as cameras.
    //    |Region of interest mode. (see MAV_ROI enum)| Waypoint index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple cameras etc.)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|
    case MAV_CMD_NAV_ROI:
 #if 0
        // send the command to the camera mount
        camera_mount.set_roi_cmd(&command_nav_queue);
 #else
        gcs_send_text_P(SEVERITY_LOW, PSTR("DO_SET_ROI not supported"));
 #endif
        break;

    case MAV_CMD_DO_MOUNT_CONFIGURE:                    // Mission command to configure a camera mount |Mount operation mode (see MAV_CONFIGURE_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|
        camera_mount.configure_cmd();
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:                      // Mission command to control a camera mount |pitch(deg*100) or lat, depending on mount mode.| roll(deg*100) or lon depending on mount mode| yaw(deg*100) or alt (in cm) depending on mount mode| Empty| Empty| Empty| Empty|
        camera_mount.control_cmd();
        break;
#endif
    }
}

static void handle_no_commands()
{
    plane_gcs_send_text_fmt(PSTR("Returning to Home"));
    next_nav_command = rally_find_best_location(current_loc, home);
    next_nav_command.id = MAV_CMD_NAV_LOITER_UNLIM;
    nav_command_ID = MAV_CMD_NAV_LOITER_UNLIM;
    non_nav_command_ID = WAIT_COMMAND;
    handle_process_nav_cmd();
}

/*******************************************************************************
Verify command Handlers

Each type of mission element has a "verify" operation. The verify
operation returns true when the mission element has completed and we
should move onto the next mission element.
*******************************************************************************/

// verify_nav_command - switch statement to ensure the active navigation command is progressing
// returns true once the active navigation command completes successfully
static bool verify_nav_command()
{
    switch(command_nav_queue.id) {

    case MAV_CMD_NAV_TAKEOFF:
        return verify_takeoff();
        break;

    case MAV_CMD_NAV_WAYPOINT:
        return verify_nav_wp();
        break;

    case MAV_CMD_NAV_LAND:
        return verify_land();
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:
        return verify_loiter_unlimited();
        break;

    case MAV_CMD_NAV_LOITER_TURNS:
        return verify_circle();
        break;

    case MAV_CMD_NAV_LOITER_TIME:
        return verify_loiter_time();
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return verify_RTL();
        break;

    default:
        return false;
        break;
    }
}

// verify_cond_command - switch statement to ensure the active conditional command is progressing
// returns true once the active conditional command completes successfully
static bool verify_cond_command()
{
    switch(command_cond_queue.id) {

    case MAV_CMD_CONDITION_DELAY:
        return verify_wait_delay();
        break;

    case MAV_CMD_CONDITION_DISTANCE:
        return verify_within_distance();
        break;

    case MAV_CMD_CONDITION_CHANGE_ALT:
        return verify_change_alt();
        break;

    case MAV_CMD_CONDITION_YAW:
        return verify_yaw();
        break;

    default:
        return false;
        break;
    }
}

static bool plane_verify_nav_command()        // Returns true if command complete
{
    switch(nav_command_ID) {

    case MAV_CMD_NAV_TAKEOFF:
        return plane_verify_takeoff();

    case MAV_CMD_NAV_LAND:
        return plane_verify_land();

    case MAV_CMD_NAV_WAYPOINT:
        return plane_verify_nav_wp();

    case MAV_CMD_NAV_LOITER_UNLIM:
        return plane_verify_loiter_unlim();

    case MAV_CMD_NAV_LOITER_TURNS:
        return verify_loiter_turns();

    case MAV_CMD_NAV_LOITER_TIME:
        return plane_verify_loiter_time();

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return plane_verify_RTL();

    default:
        gcs_send_text_P(SEVERITY_HIGH,PSTR("verify_nav: Invalid or no current Nav cmd"));
    }
    return false;
}

static bool verify_condition_command()          // Returns true if command complete
{
    switch(non_nav_command_ID) {
    case NO_COMMAND:
        break;

    case MAV_CMD_CONDITION_DELAY:
        return plane_verify_wait_delay();
        break;

    case MAV_CMD_CONDITION_DISTANCE:
        return plane_verify_within_distance();
        break;

    case MAV_CMD_CONDITION_CHANGE_ALT:
        return plane_verify_change_alt();
        break;

    case WAIT_COMMAND:
        return 0;
        break;


    default:
        gcs_send_text_P(SEVERITY_HIGH,PSTR("verify_conditon: Invalid or no current Condition cmd"));
        break;
    }
    return false;
}

/********************************************************************************/
//
/********************************************************************************/

// do_RTL - start Return-to-Launch
static void do_RTL(void)
{
    // set rtl state
    rtl_state = RTL_STATE_START;

    // verify_RTL will do the initialisation for us
    verify_RTL();
}

/********************************************************************************/
//	Nav (Must) commands
/********************************************************************************/

// do_takeoff - initiate takeoff navigation command
static void do_takeoff()
{
    // set roll-pitch mode
    set_roll_pitch_mode(AUTO_RP);

    // set yaw mode
    set_yaw_mode(YAW_HOLD);

    // set throttle mode to AUTO although we should already be in this mode
    set_throttle_mode(AUTO_THR);

    // set our nav mode to loiter
    set_nav_mode(NAV_WP);

    // Set wp navigation target to safe altitude above current position
    Vector3f pos = inertial_nav.get_position();
    pos.z = max(pos.z, command_nav_queue.alt);
    pos.z = max(pos.z, 100.0f);
    wp_nav.set_destination(pos);

    // prevent flips
    // To-Do: check if this is still necessary
    reset_I_all();    
}

// do_nav_wp - initiate move to next waypoint
static void do_nav_wp()
{
    // set roll-pitch mode
    set_roll_pitch_mode(AUTO_RP);

    // set throttle mode
    set_throttle_mode(AUTO_THR);

    // set nav mode
    set_nav_mode(NAV_WP);

    // Set wp navigation target
    wp_nav.set_destination(pv_location_to_vector(command_nav_queue));

    // initialise original_wp_bearing which is used to check if we have missed the waypoint
    wp_bearing = wp_nav.get_bearing_to_destination();
    original_wp_bearing = wp_bearing;

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time     = 0;
    // this is the delay, stored in seconds and expanded to millis
    loiter_time_max = command_nav_queue.p1;
    // if no delay set the waypoint as "fast"
    if (loiter_time_max == 0 ) {
        wp_nav.set_fast_waypoint(true);
    }

    // set yaw_mode depending upon contents of WP_YAW_BEHAVIOR parameter
    set_yaw_mode(get_wp_yaw_mode(false));
}

// do_land - initiate landing procedure
// caller should set roll_pitch_mode to ROLL_PITCH_AUTO (for no pilot input) or ROLL_PITCH_LOITER (for pilot input)
static void do_land(const struct Location *cmd)
{
    // To-Do: check if we have already landed

    // if location provided we fly to that location at current altitude
    if (cmd != NULL && (cmd->lat != 0 || cmd->lng != 0)) {
        // set state to fly to location
        land_state = LAND_STATE_FLY_TO_LOCATION;

        // set roll-pitch mode
        set_roll_pitch_mode(AUTO_RP);

        // set yaw_mode depending upon contents of WP_YAW_BEHAVIOR parameter
        set_yaw_mode(get_wp_yaw_mode(false));

        // set throttle mode
        set_throttle_mode(AUTO_THR);

        // set nav mode
        set_nav_mode(NAV_WP);

        // calculate and set desired location above landing target
        Vector3f pos = pv_location_to_vector(*cmd);
        pos.z = min(current_loc.alt, RTL_ALT_MAX);
        wp_nav.set_destination(pos);

        // initialise original_wp_bearing which is used to check if we have missed the waypoint
        wp_bearing = wp_nav.get_bearing_to_destination();
        original_wp_bearing = wp_bearing;
    }else{
        // set landing state
        land_state = LAND_STATE_DESCENDING;

        // if we have gps lock, attempt to hold horizontal position
        if (GPS_ok()) {
            // switch to loiter which restores horizontal control to pilot
            // To-Do: check that we are not in failsafe to ensure we don't process bad roll-pitch commands
            set_roll_pitch_mode(ROLL_PITCH_LOITER);
            // switch into loiter nav mode
            set_nav_mode(NAV_LOITER);
        }else{
            // no gps lock so give horizontal control to pilot
            // To-Do: check that we are not in failsafe to ensure we don't process bad roll-pitch commands
            set_roll_pitch_mode(ROLL_PITCH_STABLE);
            // switch into loiter nav mode
            set_nav_mode(NAV_NONE);
        }

        // hold yaw while landing
        set_yaw_mode(YAW_HOLD);

        // set throttle mode to land
        set_throttle_mode(THROTTLE_LAND);

    }
}

// do_loiter_unlimited - start loitering with no end conditions
// note: caller should set yaw_mode
static void do_loiter_unlimited()
{
    Vector3f target_pos;

    // set roll-pitch mode (no pilot input)
    set_roll_pitch_mode(AUTO_RP);

    // set throttle mode to AUTO which, if not already active, will default to hold at our current altitude
    set_throttle_mode(AUTO_THR);

    // hold yaw
    set_yaw_mode(YAW_HOLD);

    // get current position
    Vector3f curr_pos = inertial_nav.get_position();

    // use current location if not provided
    if(command_nav_queue.lat == 0 && command_nav_queue.lng == 0) {
        wp_nav.get_stopping_point(curr_pos,inertial_nav.get_velocity(),target_pos);
    }else{
        // default to use position provided
        target_pos = pv_location_to_vector(command_nav_queue);
    }

    // use current altitude if not provided
    if( command_nav_queue.alt == 0 ) {
        target_pos.z = curr_pos.z;
    }

    // start way point navigator and provide it the desired location
    set_nav_mode(NAV_WP);
    wp_nav.set_destination(target_pos);
}

// do_circle - initiate moving in a circle
static void do_circle()
{
    // set roll-pitch mode (no pilot input)
    set_roll_pitch_mode(AUTO_RP);

    // set throttle mode to AUTO which, if not already active, will default to hold at our current altitude
    set_throttle_mode(AUTO_THR);

    // set nav mode to CIRCLE
    set_nav_mode(NAV_CIRCLE);

    // set target altitude if provided
    if( command_nav_queue.alt != 0 ) {
        wp_nav.set_desired_alt(command_nav_queue.alt);
    }

    // override default horizontal location target
    if( command_nav_queue.lat != 0 || command_nav_queue.lng != 0) {
        circle_set_center(pv_location_to_vector(command_nav_queue), ahrs.yaw);
    }

    // set yaw to point to center of circle
    set_yaw_mode(CIRCLE_YAW);

    // set angle travelled so far to zero
    circle_angle_total = 0;

    // record number of desired rotations from mission command
    circle_desired_rotations = command_nav_queue.p1;
}

// do_loiter_time - initiate loitering at a point for a given time period
// note: caller should set yaw_mode
static void do_loiter_time()
{
    Vector3f target_pos;

    // set roll-pitch mode (no pilot input)
    set_roll_pitch_mode(AUTO_RP);

    // set throttle mode to AUTO which, if not already active, will default to hold at our current altitude
    set_throttle_mode(AUTO_THR);

    // hold yaw
    set_yaw_mode(YAW_HOLD);

    // get current position
    Vector3f curr_pos = inertial_nav.get_position();

    // use current location if not provided
    if(command_nav_queue.lat == 0 && command_nav_queue.lng == 0) {
        wp_nav.get_stopping_point(curr_pos,inertial_nav.get_velocity(),target_pos);
    }else{
        // default to use position provided
        target_pos = pv_location_to_vector(command_nav_queue);
    }

    // use current altitude if not provided
    if( command_nav_queue.alt == 0 ) {
        target_pos.z = curr_pos.z;
    }

    // start way point navigator and provide it the desired location
    set_nav_mode(NAV_WP);
    wp_nav.set_destination(target_pos);

    // setup loiter timer
    loiter_time     = 0;
    loiter_time_max = command_nav_queue.p1;     // units are (seconds)
}

static void plane_do_RTL(void)
{
    plane_control_mode    = PLANE_RTL;
    prev_WP = current_loc;
    next_WP = rally_find_best_location(current_loc, home);

    if (g.loiter_radius < 0) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }

    setup_glide_slope();
}

static void plane_do_takeoff()
{
    set_next_WP(&next_nav_command);
    // pitch in deg, airspeed  m/s, throttle %, track WP 1 or 0
    takeoff_pitch_cd                = (int)next_nav_command.p1 * 100;
    takeoff_altitude_cm     = next_nav_command.alt;
    next_WP.lat             = home.lat + 1000;          // so we don't have bad calcs
    next_WP.lng             = home.lng + 1000;          // so we don't have bad calcs
    ap.takeoff_complete        = false;                            // set flag to use gps ground course during TO.  IMU will be doing yaw drift correction
    // Flag also used to override "on the ground" throttle disable
}

static void plane_do_nav_wp()
{
    set_next_WP(&next_nav_command);
}

static void plane_do_land()
{
    set_next_WP(&next_nav_command);
}

static void loiter_set_direction_wp(const struct Location *nav_command)
{
    if (nav_command->options & MASK_OPTIONS_LOITER_DIRECTION) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }
}

static void plane_do_loiter_unlimited()
{
    set_next_WP(&next_nav_command);
    loiter_set_direction_wp(&next_nav_command);
}

static void do_loiter_turns()
{
    set_next_WP(&next_nav_command);
    loiter.total_cd = next_nav_command.p1 * 36000UL;
    loiter_set_direction_wp(&next_nav_command);
}

static void plane_do_loiter_time()
{
    set_next_WP(&next_nav_command);
    // we set start_time_ms when we reach the waypoint
    loiter.start_time_ms = 0;
    loiter.time_max_ms = next_nav_command.p1 * (uint32_t)1000;     // units are seconds
    loiter_set_direction_wp(&next_nav_command);
}

/********************************************************************************/
//	Verify Nav (Must) commands
/********************************************************************************/

// verify_takeoff - check if we have completed the takeoff
static bool verify_takeoff()
{
    // have we reached our target altitude?
    set_takeoff_complete(wp_nav.reached_destination());
    return wp_nav.reached_destination();
}

// verify_land - returns true if landing has been completed
static bool verify_land()
{
    bool retval = false;

    switch( land_state ) {
        case LAND_STATE_FLY_TO_LOCATION:
            // check if we've reached the location
            if (wp_nav.reached_destination()) {
                // get destination so we can use it for loiter target
                Vector3f dest = wp_nav.get_destination();

                // switch into loiter nav mode
                set_nav_mode(NAV_LOITER);

                // override loiter target
                wp_nav.set_loiter_target(dest);

                // switch to loiter which restores horizontal control to pilot
                // To-Do: check that we are not in failsafe to ensure we don't process bad roll-pitch commands
                set_roll_pitch_mode(ROLL_PITCH_LOITER);

                // give pilot control of yaw
                set_yaw_mode(YAW_HOLD);

                // set throttle mode to land
                set_throttle_mode(THROTTLE_LAND);

                // advance to next state
                land_state = LAND_STATE_DESCENDING;
            }
            break;

        case LAND_STATE_DESCENDING:
            // rely on THROTTLE_LAND mode to correctly update landing status
            retval = ap.land_complete;
            break;

        default:
            // this should never happen
            // TO-DO: log an error
            retval = true;
            break;
    }

    // true is returned if we've successfully landed
    return retval;
}

// verify_nav_wp - check if we have reached the next way point
static bool verify_nav_wp()
{
    // check if we have reached the waypoint
    if( !wp_nav.reached_destination() ) {
        return false;
    }

    // start timer if necessary
    if(loiter_time == 0) {
        loiter_time = millis();
    }

    // check if timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        gcs_send_text_fmt(PSTR("Reached Command #%i"),command_nav_index);
        return true;
    }else{
        return false;
    }
}

static bool verify_loiter_unlimited()
{
    return false;
}

// verify_loiter_time - check if we have loitered long enough
static bool verify_loiter_time()
{
    // return immediately if we haven't reached our destination
    if (!wp_nav.reached_destination()) {
        return false;
    }

    // start our loiter timer
    if( loiter_time == 0 ) {
        loiter_time = millis();
    }

    // check if loiter timer has run out
    return (((millis() - loiter_time) / 1000) >= loiter_time_max);
}

// verify_circle - check if we have circled the point enough
static bool verify_circle()
{
    // have we rotated around the center enough times?
    return fabsf(circle_angle_total/(2*M_PI)) >= circle_desired_rotations;
}

// verify_RTL - handles any state changes required to implement RTL
// do_RTL should have been called once first to initialise all variables
// returns true with RTL has completed successfully
static bool verify_RTL()
{
    bool retval = false;

    switch( rtl_state ) {
        case RTL_STATE_START:
            // set roll, pitch and yaw modes
            set_roll_pitch_mode(RTL_RP);
            set_throttle_mode(RTL_THR);

            // set navigation mode
            set_nav_mode(NAV_WP);

            // if we are below rtl alt do initial climb
            if( current_loc.alt < get_RTL_alt() ) {
                // first stage of RTL is the initial climb so just hold current yaw
                set_yaw_mode(YAW_HOLD);

                // use projection of safe stopping point based on current location and velocity
                Vector3f origin, dest;
                wp_nav.get_stopping_point(inertial_nav.get_position(),inertial_nav.get_velocity(),origin);
                dest.x = origin.x;
                dest.y = origin.y;
                dest.z = get_RTL_alt();
                wp_nav.set_origin_and_destination(origin,dest);

                // advance to next rtl state
                rtl_state = RTL_STATE_INITIAL_CLIMB;
            }else{
                // point nose towards home (maybe)
                set_yaw_mode(get_wp_yaw_mode(true));

                // Set wp navigation target to above home
                wp_nav.set_destination(Vector3f(0,0,get_RTL_alt()));

                // initialise original_wp_bearing which is used to point the nose home
                wp_bearing = wp_nav.get_bearing_to_destination();
                original_wp_bearing = wp_bearing;
                
                // advance to next rtl state
                rtl_state = RTL_STATE_RETURNING_HOME;
            }
            break;
        case RTL_STATE_INITIAL_CLIMB:
            // check if we've reached the safe altitude
            if (wp_nav.reached_destination()) {
                // set nav mode
                set_nav_mode(NAV_WP);

                // Set wp navigation target to above home
                wp_nav.set_destination(Vector3f(0,0,get_RTL_alt()));

                // initialise original_wp_bearing which is used to point the nose home
                wp_bearing = wp_nav.get_bearing_to_destination();
                original_wp_bearing = wp_bearing;

                // point nose towards home (maybe)
                set_yaw_mode(get_wp_yaw_mode(true));

                // advance to next rtl state
                rtl_state = RTL_STATE_RETURNING_HOME;
            }
            break;

        case RTL_STATE_RETURNING_HOME:
            // check if we've reached home
            if (wp_nav.reached_destination()) {
                // Note: we remain in NAV_WP nav mode which should hold us above home

                // start timer
                rtl_loiter_start_time = millis();

                // give pilot back control of yaw
                if(get_wp_yaw_mode(true) != YAW_HOLD) {
                    set_yaw_mode(YAW_RESETTOARMEDYAW); // yaw back to initial yaw on take off
                } else {
                    set_yaw_mode(YAW_HOLD);
                }

                // advance to next rtl state
                rtl_state = RTL_STATE_LOITERING_AT_HOME;
            }
            break;

        case RTL_STATE_LOITERING_AT_HOME:
            // check if we've loitered long enough
            if( millis() - rtl_loiter_start_time > (uint32_t)g.rtl_loiter_time.get() ) {
                // initiate landing or descent
                if(g.rtl_alt_final == 0 || failsafe.radio) {
                    // switch to loiter which restores horizontal control to pilot
                    // To-Do: check that we are not in failsafe to ensure we don't process bad roll-pitch commands
                    set_roll_pitch_mode(ROLL_PITCH_LOITER);
                    // switch into loiter nav mode
                    set_nav_mode(NAV_LOITER);
                    // override landing location (loiter defaults to a projection from current location)
                    wp_nav.set_loiter_target(Vector3f(0,0,0));

                    // hold yaw while landing
                    set_yaw_mode(YAW_HOLD);

                    // set throttle mode to land
                    set_throttle_mode(THROTTLE_LAND);

                    // update RTL state
                    rtl_state = RTL_STATE_LAND;
                }else{
                    // descend using waypoint controller
                    if(current_loc.alt > g.rtl_alt_final) {
                        // set navigation mode
                        set_nav_mode(NAV_WP);
                        // Set wp navigation alt target to rtl_alt_final
                        wp_nav.set_destination(Vector3f(0,0,g.rtl_alt_final));
                    }
                    // update RTL state
                    rtl_state = RTL_STATE_FINAL_DESCENT;
                }
            }
            break;

        case RTL_STATE_FINAL_DESCENT:
            // check we have reached final altitude
            if(current_loc.alt <= g.rtl_alt_final || wp_nav.reached_destination()) {
                // indicate that we've completed RTL
                retval = true;
            }
            break;

        case RTL_STATE_LAND:
            // rely on land_complete flag to indicate if we have landed
            retval = ap.land_complete;
            break;

        default:
            // this should never happen
            // TO-DO: log an error
            retval = true;
            break;
    }

    // true is returned if we've successfully completed RTL
    return retval;
}

static bool plane_verify_takeoff()
{
    if (ahrs.yaw_initialised()) {
        if (steer_state.hold_course_cd == -1) {
            // save our current course to take off
            steer_state.hold_course_cd = ahrs.yaw_sensor;
            plane_gcs_send_text_fmt(PSTR("Holding course %ld"), steer_state.hold_course_cd);
        }
    }

    if (steer_state.hold_course_cd != -1) {
        // call navigation controller for heading hold
        nav_controller->update_heading_hold(steer_state.hold_course_cd);
    } else {
        nav_controller->update_level_flight();        
    }

    // see if we have reached takeoff altitude
    if (adjusted_altitude_cm() > takeoff_altitude_cm) {
        steer_state.hold_course_cd = -1;
        ap.takeoff_complete = true;
        next_WP = prev_WP = current_loc;
        return true;
    } else {
        return false;
    }
}

// we are executing a landing
static bool plane_verify_land()
{
    // we don't 'verify' landing in the sense that it never completes,
    // so we don't verify command completion. Instead we use this to
    // adjust final landing parameters

    // Set ap.land_complete if we are within 2 seconds distance or within
    // 3 meters altitude of the landing point
    if ((plane_wp_distance <= (g.land_flare_sec*g_gps->ground_speed_cm*0.01f))
        || (adjusted_altitude_cm() <= next_WP.alt + g.land_flare_alt*100)) {

        ap.land_complete = true;

        if (steer_state.hold_course_cd == -1) {
            // we have just reached the threshold of to flare for landing.
            // We now don't want to do any radical
            // turns, as rolling could put the wings into the runway.
            // To prevent further turns we set steer_state.hold_course_cd to the
            // current heading. Previously we set this to
            // crosstrack_bearing, but the xtrack bearing can easily
            // be quite large at this point, and that could induce a
            // sudden large roll correction which is very nasty at
            // this point in the landing.
            steer_state.hold_course_cd = ahrs.yaw_sensor;
            plane_gcs_send_text_fmt(PSTR("Land Complete - Hold course %ld"), steer_state.hold_course_cd);
        }

        if (g_gps->ground_speed_cm*0.01f < 3.0) {
            // reload any airspeed or groundspeed parameters that may have
            // been set for landing. We don't do this till ground
            // speed drops below 3.0 m/s as otherwise we will change
            // target speeds too early.
            g.airspeed_cruise_cm.load();
            g.min_gndspeed_cm.load();
            aparm.plthr_cruise.load();
        }
    }

    if (steer_state.hold_course_cd != -1) {
        // recalc bearing error with hold_course;
        nav_controller->update_heading_hold(steer_state.hold_course_cd);
    } else {
        nav_controller->update_waypoint(prev_WP, next_WP);
    }
    return false;
}

static bool plane_verify_nav_wp()
{
    steer_state.hold_course_cd = -1;

    nav_controller->update_waypoint(prev_WP, next_WP);

    // see if the user has specified a maximum distance to waypoint
    if (g.waypoint_max_radius > 0 && plane_wp_distance > (uint16_t)g.waypoint_max_radius) {
        if (location_passed_point(current_loc, prev_WP, next_WP)) {
            // this is needed to ensure completion of the waypoint
            prev_WP = current_loc;
        }
        return false;
    }
    
    if (plane_wp_distance <= nav_controller->turn_distance(g.waypoint_radius)) {
        plane_gcs_send_text_fmt(PSTR("Reached Waypoint #%i dist %um"),
                          (unsigned)nav_command_index,
                          (unsigned)get_distance(current_loc, next_WP));
        return true;
	}

    // have we flown past the waypoint?
    if (location_passed_point(current_loc, prev_WP, next_WP)) {
        plane_gcs_send_text_fmt(PSTR("Passed Waypoint #%i dist %um"),
                          (unsigned)nav_command_index,
                          (unsigned)get_distance(current_loc, next_WP));
        return true;
    }

    return false;
}

static bool plane_verify_loiter_unlim()
{
    update_loiter();
    return false;
}

static bool plane_verify_loiter_time()
{
    update_loiter();
    if (loiter.start_time_ms == 0) {
        if (nav_controller->reached_loiter_target()) {
            // we've reached the target, start the timer
            loiter.start_time_ms = millis();
        }
    } else if ((millis() - loiter.start_time_ms) > loiter.time_max_ms) {
        gcs_send_text_P(SEVERITY_LOW,PSTR("verify_nav: PLANE_LOITER time complete"));
        return true;
    }
    return false;
}

static bool verify_loiter_turns()
{
    update_loiter();
    if (loiter.sum_cd > loiter.total_cd) {
        loiter.total_cd = 0;
        gcs_send_text_P(SEVERITY_LOW,PSTR("verify_nav: PLANE_LOITER orbits complete"));
        // clear the command queue;
        return true;
    }
    return false;
}

static bool plane_verify_RTL()
{
    update_loiter();
	if (plane_wp_distance <= (uint32_t)max(g.waypoint_radius,0) || 
        nav_controller->reached_loiter_target()) {
			gcs_send_text_P(SEVERITY_LOW,PSTR("Reached home"));
			return true;
    } else {
        return false;
	}
}

/********************************************************************************/
//	Condition (May) commands
/********************************************************************************/

static void do_wait_delay()
{
    //cliSerial->print("dwd ");
    condition_start = millis();
    condition_value = command_cond_queue.lat * 1000;     // convert to milliseconds
    //cliSerial->println(condition_value,DEC);
}

static void do_change_alt()
{
    // adjust target appropriately for each nav mode
    switch (nav_mode) {
        case NAV_CIRCLE:
        case NAV_LOITER:
            // update loiter target altitude
            wp_nav.set_desired_alt(command_cond_queue.alt);
            break;

        case NAV_WP:
            // To-Do: update waypoint nav's destination altitude
            break;
    }

    // To-Do: store desired altitude in a variable so that it can be verified later
}

static void do_within_distance()
{
    condition_value  = command_cond_queue.lat * 100;
}

static void do_yaw()
{
    // get final angle, 1 = Relative, 0 = Absolute
    if( command_cond_queue.lng == 0 ) {
        // absolute angle
        yaw_look_at_heading = wrap_360_cd(command_cond_queue.alt * 100);
    }else{
        // relative angle
        yaw_look_at_heading = wrap_360_cd(control_yaw + command_cond_queue.alt * 100);
    }

    // get turn speed
    if( command_cond_queue.lat == 0 ) {
        // default to regular auto slew rate
        yaw_look_at_heading_slew = AUTO_YAW_SLEW_RATE;
    }else{
        int32_t turn_rate = (wrap_180_cd(yaw_look_at_heading - control_yaw) / 100) / command_cond_queue.lat;
        yaw_look_at_heading_slew = constrain_int32(turn_rate, 1, 360);    // deg / sec
    }

    // set yaw mode
    set_yaw_mode(YAW_LOOK_AT_HEADING);

    // TO-DO: restore support for clockwise / counter clockwise rotation held in command_cond_queue.p1
    // command_cond_queue.p1; // 0 = undefined, 1 = clockwise, -1 = counterclockwise
}

static void plane_do_wait_delay()
{
    condition_start = millis();
    condition_value  = next_nonnav_command.lat * 1000;          // convert to milliseconds
}

static void plane_do_change_alt()
{
    condition_rate          = labs((int)next_nonnav_command.lat);
    condition_value         = next_nonnav_command.alt;
    if (condition_value < adjusted_altitude_cm()) {
        condition_rate = -condition_rate;
    }
    target_altitude_cm      = adjusted_altitude_cm() + (condition_rate / 10);                  // Divide by ten for 10Hz update
    next_WP.alt             = condition_value;                                                                  // For future nav calculations
    offset_altitude_cm      = 0;                                                                                        // For future nav calculations
}

static void plane_do_within_distance()
{
    condition_value  = next_nonnav_command.lat;
}


/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

static bool verify_wait_delay()
{
    //cliSerial->print("vwd");
    if (millis() - condition_start > (uint32_t)max(condition_value,0)) {
        //cliSerial->println("y");
        condition_value = 0;
        return true;
    }
    //cliSerial->println("n");
    return false;
}

static bool verify_change_alt()
{
    // To-Do: use recorded target altitude to verify we have reached the target
    return true;
}

static bool verify_within_distance()
{
    if (wp_distance < max(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

// verify_yaw - return true if we have reached the desired heading
static bool verify_yaw()
{
    if( labs(wrap_180_cd(ahrs.yaw_sensor-yaw_look_at_heading)) <= 200 ) {
        return true;
    }else{
        return false;
    }
}

static bool plane_verify_wait_delay()
{
    if ((unsigned)(millis() - condition_start) > (unsigned)condition_value) {
        condition_value         = 0;
        return true;
    }
    return false;
}

static bool plane_verify_change_alt()
{
    if( (condition_rate>=0 && adjusted_altitude_cm() >= condition_value) || 
        (condition_rate<=0 && adjusted_altitude_cm() <= condition_value)) {
        condition_value = 0;
        return true;
    }
    target_altitude_cm += condition_rate / 10;
    return false;
}

static bool plane_verify_within_distance()
{
    if (plane_wp_distance < max(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

/********************************************************************************/
//	Do (Now) commands
/********************************************************************************/

// do_guided - start guided mode
// this is not actually a mission command but rather a 
static void do_guided(const struct Location *cmd)
{
    bool first_time = false;
    // switch to guided mode if we're not already in guided mode
    if (control_mode != GUIDED) {
        if (set_mode(GUIDED)) {
            first_time = true;
        }else{
            // if we failed to enter guided mode return immediately
            return;
        }
    }

    // set wp_nav's destination
    Vector3f pos = pv_location_to_vector(*cmd);
    wp_nav.set_destination(pos);

    // initialise wp_bearing for reporting purposes
    wp_bearing = wp_nav.get_bearing_to_destination();

    // point nose at next waypoint if it is more than 10m away
    if (yaw_mode == YAW_LOOK_AT_NEXT_WP) {
        // get distance to new location
        wp_distance = wp_nav.get_distance_to_destination();
        // set original_wp_bearing to point at next waypoint
        if (wp_distance >= 1000 || first_time) {
            original_wp_bearing = wp_bearing;
        }
    }
}

static void do_change_speed()
{
    wp_nav.set_horizontal_velocity(command_cond_queue.p1 * 100);
}

static void do_jump()
{
    // Used to track the state of the jump command in Mission scripting
    // -10 is a value that means the register is unused
    // when in use, it contains the current remaining jumps
    static int8_t jump = -10;                                                                   // used to track loops in jump command

    if(jump == -10) {
        // we use a locally stored index for jump
        jump = command_cond_queue.lat;
    }

    if(jump > 0) {
        jump--;
        change_command(command_cond_queue.p1);

    } else if (jump == 0) {
        // we're done, move along
        jump = -11;

    } else if (jump == -1) {
        // repeat forever
        change_command(command_cond_queue.p1);
    }
}

static void do_set_home()
{
    if(command_cond_queue.p1 == 1) {
        init_home();
    } else {
        home.id         = MAV_CMD_NAV_WAYPOINT;
        home.lng        = command_cond_queue.lng;                                       // Lon * 10**7
        home.lat        = command_cond_queue.lat;                                       // Lat * 10**7
        home.alt        = 0;
        //home_is_set 	= true;
        set_home_is_set(true);
    }
}

// do_roi - starts actions required by MAV_CMD_NAV_ROI
//          this involves either moving the camera to point at the ROI (region of interest)
//          and possibly rotating the copter to point at the ROI if our mount type does not support a yaw feature
//          Note: the ROI should already be in the command_nav_queue global variable
//	TO-DO: add support for other features of MAV_CMD_DO_SET_ROI including pointing at a given waypoint
static void do_roi()
{
#if MOUNT == ENABLED
    // check if mount type requires us to rotate the quad
    if( camera_mount.get_mount_type() != AP_Mount::k_pan_tilt && camera_mount.get_mount_type() != AP_Mount::k_pan_tilt_roll ) {
        yaw_look_at_WP = pv_location_to_vector(command_cond_queue);
        set_yaw_mode(YAW_LOOK_AT_LOCATION);
    }
    // send the command to the camera mount
    camera_mount.set_roi_cmd(&command_cond_queue);
    
    // TO-DO: expand handling of the do_nav_roi to support all modes of the MAVLink.  Currently we only handle mode 4 (see below)
    //		0: do nothing
    //		1: point at next waypoint
    //		2: point at a waypoint taken from WP# parameter (2nd parameter?)
    //		3: point at a location given by alt, lon, lat parameters
    //		4: point at a target given a target id (can't be implemented)
#else
    // if we have no camera mount aim the quad at the location
    yaw_look_at_WP = pv_location_to_vector(command_cond_queue);
    set_yaw_mode(YAW_LOOK_AT_LOCATION);
#endif
}

// do_take_picture - take a picture with the camera library
static void do_take_picture()
{
#if CAMERA == ENABLED
    camera.trigger_pic();
    if (g.log_bitmask & MASK_LOG_CAMERA) {
        Log_Write_Camera();
    }
#endif
}

static void do_loiter_at_location()
{
    if (g.loiter_radius < 0) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }
    next_WP = current_loc;
}

static void plane_do_jump()
{
    if (next_nonnav_command.lat == 0) {
        // the jump counter has reached zero - ignore
        plane_gcs_send_text_fmt(PSTR("Jumps left: 0 - skipping"));
        return;
    }
    if (next_nonnav_command.p1 >= g.command_total) {
        plane_gcs_send_text_fmt(PSTR("Skipping invalid jump to %i"), next_nonnav_command.p1);
        return;        
    }

    struct Location temp;
    temp = plane_get_cmd_with_index(g.command_index);

    plane_gcs_send_text_fmt(PSTR("Jump to WP %u. Jumps left: %d"),
                      (unsigned)next_nonnav_command.p1,
                      (int)next_nonnav_command.lat);
    if (next_nonnav_command.lat > 0) {
        // Decrement repeat counter
        temp.lat                        = next_nonnav_command.lat - 1;                                          
        plane_set_cmd_with_index(temp, g.command_index);
    }

    nav_command_ID          = NO_COMMAND;
    next_nav_command.id     = NO_COMMAND;
    non_nav_command_ID      = NO_COMMAND;

    plane_gcs_send_text_fmt(PSTR("setting command index: %i"), next_nonnav_command.p1);
    g.command_index.set_and_save(next_nonnav_command.p1);
    nav_command_index       = next_nonnav_command.p1;
    // Need to back "next_WP" up as it was set to the next waypoint following the jump
    next_WP = prev_WP;

    temp = plane_get_cmd_with_index(g.command_index);

    next_nav_command = temp;
    nav_command_ID = next_nav_command.id;
    non_nav_command_index = g.command_index;
    non_nav_command_ID = WAIT_COMMAND;

#if LOGGING_ENABLED == ENABLED
    if (should_log(MASK_LOG_CMD)) {
        Log_Write_Cmd(g.command_index, &next_nav_command);
    }
#endif

    handle_process_nav_cmd();
}

static void plane_do_change_speed()
{
    switch (next_nonnav_command.p1)
    {
    case 0:             // Airspeed
        if (next_nonnav_command.alt > 0) {
            g.airspeed_cruise_cm.set(next_nonnav_command.alt * 100);
            plane_gcs_send_text_fmt(PSTR("Set airspeed %u m/s"), (unsigned)next_nonnav_command.alt);
        }
        break;
    case 1:             // Ground speed
        plane_gcs_send_text_fmt(PSTR("Set groundspeed %u"), (unsigned)next_nonnav_command.alt);
        g.min_gndspeed_cm.set(next_nonnav_command.alt * 100);
        break;
    }

    if (next_nonnav_command.lat > 0) {
        plane_gcs_send_text_fmt(PSTR("Set throttle %u"), (unsigned)next_nonnav_command.lat);
        aparm.plthr_cruise.set(next_nonnav_command.lat);
    }
}

static void plane_do_set_home()
{
    if (next_nonnav_command.p1 == 1 && g_gps->status() == GPS::GPS_OK_FIX_3D) {
        plane_init_home();
    } else {
        home.id         = MAV_CMD_NAV_WAYPOINT;
        home.lng        = next_nonnav_command.lng;                                      // Lon * 10**7
        home.lat        = next_nonnav_command.lat;                                      // Lat * 10**7
        home.alt        = max(next_nonnav_command.alt, 0);
        ap.home_is_set = true;
    }
}

// plane_do_take_picture - take a picture with the camera library
static void plane_do_take_picture()
{
#if CAMERA == ENABLED
    camera.trigger_pic();
#if LOGGING_ENABLED == ENABLED
    if (should_log(MASK_LOG_CAMERA)) {
        Log_Write_Camera();
    }
#endif
#endif
}
#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/commands_process.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// For changing active command mid-mission
//----------------------------------------
static void change_command(uint8_t cmd_index)
{
    // check we are in AUTO mode
    if (control_mode != AUTO) {
        return;
    }

    // limit range
    cmd_index = min(g.command_total - 1, cmd_index);

    // load command
    struct Location temp = get_cmd_with_index(cmd_index);

    // verify it's a nav command
    if(temp.id > MAV_CMD_NAV_LAST) {

    }else{
        // clear out command queue
        init_commands();

        // copy command to the queue
        command_nav_queue               = temp;
        command_nav_index               = cmd_index;
        execute_nav_command();
    }
}

void plane_change_command(uint8_t cmd_index)
{
    struct Location temp;

    if (cmd_index == 0) {
        plane_init_commands();
        plane_gcs_send_text_fmt(PSTR("Received Request - reset mission"));
        return;
    }

    temp = plane_get_cmd_with_index(cmd_index);

    if (temp.id > MAV_CMD_NAV_LAST ) {
        plane_gcs_send_text_fmt(PSTR("Cannot change to non-Nav cmd %u"), (unsigned)cmd_index);
    } else {
        plane_gcs_send_text_fmt(PSTR("Received Request - jump to command #%i"),cmd_index);

        nav_command_ID          = NO_COMMAND;
        next_nav_command.id = NO_COMMAND;
        non_nav_command_ID      = NO_COMMAND;

        /*
          if we are in PLANE_AUTO then we need to set the nav_command_index
          to one less than the requested index as
          process_next_command() will increment the index before using
          it. If not in PLANE_AUTO then we just set the index as give.
          Thanks to Michael Day for finding this!
         */
        if (plane_control_mode == PLANE_AUTO) {
            nav_command_index       = cmd_index - 1;
        } else {
            nav_command_index       = cmd_index;
        }
        g.command_index.set_and_save(cmd_index);
        plane_update_commands();
    }
}

// update_commands - initiates new navigation commands if we have completed the previous command
// called by 10 Hz loop
static void update_commands()
{
    if(g.command_total <= 1)
        return;

    if(command_nav_queue.id == NO_COMMAND) {
        // Our queue is empty
        // fill command queue with a new command if available, or exit mission
        // -------------------------------------------------------------------

        // find next nav command
        int16_t tmp_index;

        if(command_nav_index < g.command_total) {

            // what is the next index for a nav command?
            tmp_index = find_next_nav_index(command_nav_index + 1);

            if(tmp_index == -1) {
                exit_mission();
                return;
            }else{
                command_nav_index = tmp_index;
                command_nav_queue = get_cmd_with_index(command_nav_index);
                execute_nav_command();
            }
        }else{
            // we are out of commands
            exit_mission();
            return;
        }
    }

    if(command_cond_queue.id == NO_COMMAND) {
        // Our queue is empty
        // fill command queue with a new command if available, or do nothing
        // -------------------------------------------------------------------

        // no nav commands completed yet
        if(prev_nav_index == NO_COMMAND)
            return;

        if(command_cond_index >= command_nav_index) {
            // don't process the fututre
            return;

        }else if(command_cond_index == NO_COMMAND) {
            // start from scratch
            // look at command after the most recent completed nav
            command_cond_index = prev_nav_index + 1;

        }else{
            // we've completed 1 cond, look at next command for another
            command_cond_index++;
        }

        if(command_cond_index < (g.command_total -2)) {
            // we're OK to load a new command (last command must be a nav command)
            command_cond_queue = get_cmd_with_index(command_cond_index);

            if(command_cond_queue.id > MAV_CMD_CONDITION_LAST) {
                // this is a do now command
                process_now_command();

                // clear command queue
                command_cond_queue.id = NO_COMMAND;

            }else if(command_cond_queue.id > MAV_CMD_NAV_LAST) {
                // this is a conditional command
                process_cond_command();

            }else{
                // this is a nav command, don't process
                // clear the command conditional queue and index
                prev_nav_index                  = NO_COMMAND;
                command_cond_index              = NO_COMMAND;
                command_cond_queue.id   = NO_COMMAND;
            }

        }
    }
}


static void plane_update_commands(void)
{
    if(plane_control_mode == PLANE_AUTO) {
        if(ap.home_is_set == true && g.command_total > 1) {
            process_next_command();
        }
    }
}

// execute_nav_command - performs minor initialisation and logging before next navigation command in the queue is executed
static void execute_nav_command(void)
{
    // This is what we report to MAVLINK
    g.command_index = command_nav_index;

    // Save CMD to Log
    if(g.log_bitmask & MASK_LOG_CMD)
        Log_Write_Cmd(g.command_index, &command_nav_queue);

    // clear navigation prameters
    reset_nav_params();

    // Act on the new command
    process_nav_command();

    // clear May indexes to force loading of more commands
    // existing May commands are tossed.
    command_cond_index      = NO_COMMAND;
}

// verify_commands - high level function to check if navigation and conditional commands have completed
static void verify_commands(void)
{
    // check if navigation command completed
    if(verify_nav_command()) {
        // clear navigation command queue so next command can be loaded
        command_nav_queue.id    = NO_COMMAND;

        // store our most recent executed nav command
        prev_nav_index          = command_nav_index;

        // Wipe existing conditionals
        command_cond_index      = NO_COMMAND;
        command_cond_queue.id   = NO_COMMAND;
    }

    // check if conditional command completed
    if(verify_cond_command()) {
        // clear conditional command queue so next command can be loaded
        command_cond_queue.id = NO_COMMAND;
    }
}

static void plane_verify_commands(void)
{
    if(plane_verify_nav_command()) {
        nav_command_ID = NO_COMMAND;
    }

    if(verify_condition_command()) {
        non_nav_command_ID = NO_COMMAND;
    }
}

static void process_next_command()
{
    // This function makes sure that we always have a current navigation command
    // and loads conditional or immediate commands if applicable

    struct Location temp;
    uint8_t old_index = nav_command_index;

    // these are Navigation/Must commands
    // ---------------------------------
    if (nav_command_ID == NO_COMMAND) {    // no current navigation command loaded
        temp.id = MAV_CMD_NAV_LAST;
        while(temp.id >= MAV_CMD_NAV_LAST && nav_command_index <= g.command_total) {
            nav_command_index++;
            temp = plane_get_cmd_with_index(nav_command_index);
        }

        plane_gcs_send_text_fmt(PSTR("Nav command index updated to #%i"),nav_command_index);

        if(nav_command_index > g.command_total) {
            // we are out of commands!
            gcs_send_text_P(SEVERITY_LOW,PSTR("out of commands!"));
            handle_no_commands();
        } else {
            next_nav_command = temp;
            nav_command_ID = next_nav_command.id;
            non_nav_command_index = NO_COMMAND;                                 // This will cause the next intervening non-nav command (if any) to be loaded
            non_nav_command_ID = NO_COMMAND;

#if LOGGING_ENABLED == ENABLED
            if (should_log(MASK_LOG_CMD)) {
                Log_Write_Cmd(g.command_index, &next_nav_command);
            }
#endif
            handle_process_nav_cmd();
        }
    }

    // these are Condition/May and Do/Now commands
    // -------------------------------------------
    if (non_nav_command_index == NO_COMMAND) {                  // If the index is NO_COMMAND then we have just loaded a nav command
        non_nav_command_index = old_index + 1;
        //plane_gcs_send_text_fmt(PSTR("Non-Nav command index #%i"),non_nav_command_index);
    } else if (non_nav_command_ID == NO_COMMAND) {      // If the ID is NO_COMMAND then we have just completed a non-nav command
        non_nav_command_index++;
    }

    //plane_gcs_send_text_fmt(PSTR("Nav command index #%i"),nav_command_index);
    //plane_gcs_send_text_fmt(PSTR("Non-Nav command index #%i"),non_nav_command_index);
    //plane_gcs_send_text_fmt(PSTR("Non-Nav command ID #%i"),non_nav_command_ID);
    if (nav_command_index <= (int)g.command_total && non_nav_command_ID == NO_COMMAND) {
        temp = plane_get_cmd_with_index(non_nav_command_index);
        if (temp.id <= MAV_CMD_NAV_LAST) {                       
            // The next command is a nav command.  No non-nav commands to do
            g.command_index.set_and_save(nav_command_index);
            non_nav_command_index = nav_command_index;
            non_nav_command_ID = WAIT_COMMAND;
            plane_gcs_send_text_fmt(PSTR("Non-Nav command ID updated to #%i idx=%u"),
                              (unsigned)non_nav_command_ID, 
                              (unsigned)non_nav_command_index);

        } else {                                                                        
            // The next command is a non-nav command.  Prepare to execute it.
            g.command_index.set_and_save(non_nav_command_index);
            next_nonnav_command = temp;
            non_nav_command_ID = next_nonnav_command.id;
            plane_gcs_send_text_fmt(PSTR("(2) Non-Nav command ID updated to #%i idx=%u"),
                              (unsigned)non_nav_command_ID, (unsigned)non_nav_command_index);

#if LOGGING_ENABLED == ENABLED
            if (should_log(MASK_LOG_CMD)) {
                Log_Write_Cmd(g.command_index, &next_nonnav_command);
            }
#endif

            process_non_nav_command();
        }
    }
}

static void process_non_nav_command()
{
    //gcs_send_text_P(SEVERITY_LOW,PSTR("new non-nav command loaded"));

    if(non_nav_command_ID < MAV_CMD_CONDITION_LAST) {
        handle_process_condition_command();
    } else {
        handle_process_do_command();
        // flag command ID so a new one is loaded
        // -----------------------------------------
        if (non_nav_command_ID != WAIT_COMMAND) {
            non_nav_command_ID = NO_COMMAND;
        }
    }
}

// Finds the next navgation command in EEPROM
static int16_t find_next_nav_index(int16_t search_index)
{
    Location tmp;
    while(search_index < g.command_total) {
        tmp = get_cmd_with_index(search_index);
        if(tmp.id <= MAV_CMD_NAV_LAST) {
            return search_index;
        }else{
            search_index++;
        }
    }
    return -1;
}

static void exit_mission()
{
    // we are out of commands
    g.command_index = 255;

    // if we are not on the ground switch to loiter or land
    if(!ap.land_complete) {
        // try to enter loiter but if that fails land
        if (!set_mode(LOITER)) {
            set_mode(LAND);
        }
    }
}

#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/compat.pde"


static void delay(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

static void mavlink_delay(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

static uint32_t millis()
{
    return hal.scheduler->millis();
}

static uint32_t micros()
{
    return hal.scheduler->micros();
}

static void pinMode(uint8_t pin, uint8_t output)
{
    hal.gpio->pinMode(pin, output);
}

static void digitalWrite(uint8_t pin, uint8_t out)
{
    hal.gpio->write(pin,out);
}

static uint8_t digitalRead(uint8_t pin)
{
    return hal.gpio->read(pin);
}

#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/control_modes.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define CONTROL_SWITCH_COUNTER  20  // 20 iterations at 100hz (i.e. 2/10th of a second) at a new switch position will cause flight mode change
static void read_control_switch()
{
    static uint8_t switch_counter = 0;

    uint8_t switchPosition = readSwitch();

    // has switch moved?
    // ignore flight mode changes if in failsafe
    if (oldSwitchPosition != switchPosition && !failsafe.radio && failsafe.radio_counter == 0) {
        switch_counter++;
        if(switch_counter >= CONTROL_SWITCH_COUNTER) {
            oldSwitchPosition       = switchPosition;
            switch_counter          = 0;

            // set flight mode and simple mode setting
            if (set_mode(flight_modes[switchPosition])) {

                if(g.ch7_option != AUX_SWITCH_SIMPLE_MODE && g.ch8_option != AUX_SWITCH_SIMPLE_MODE && g.ch7_option != AUX_SWITCH_SUPERSIMPLE_MODE && g.ch8_option != AUX_SWITCH_SUPERSIMPLE_MODE) {
                    // set Simple mode using stored paramters from Mission planner
                    // rather than by the control switch
                    if (BIT_IS_SET(g.super_simple, switchPosition)) {
                        set_simple_mode(2);
                    }else{
                        set_simple_mode(BIT_IS_SET(g.simple_modes, switchPosition));
                    }
                }
            }

        }
    }else{
        // reset switch_counter if there's been no change
        // we don't want 10 intermittant blips causing a flight mode change
        switch_counter = 0;
    }
}

static void plane_read_control_switch()
{
    static bool switch_debouncer;
    uint8_t switchPosition = readSwitch();

    // If switchPosition = 255 this indicates that the mode control channel input was out of range
    // If we get this value we do not want to change modes.
    if(switchPosition == 255) return;

    if (plane_failsafe.ch3_failsafe || plane_failsafe.ch3_counter > 0) {
        // when we are in ch3_failsafe mode then RC input is not
        // working, and we need to ignore the mode switch channel
        return;
    }

    // we look for changes in the switch position. If the
    // RST_SWITCH_CH parameter is set, then it is a switch that can be
    // used to force re-reading of the control switch. This is useful
    // when returning to the previous mode after a failsafe or fence
    // breach. This channel is best used on a momentary switch (such
    // as a spring loaded trainer switch).
    if (oldSwitchPosition != switchPosition ||
        (g.reset_switch_chan != 0 &&
         hal.rcin->read(g.reset_switch_chan-1) > RESET_SWITCH_CHAN_PWM)) {

        if (switch_debouncer == false) {
            // this ensures that mode switches only happen if the
            // switch changes for 2 reads. This prevents momentary
            // spikes in the mode control channel from causing a mode
            // switch
            switch_debouncer = true;
            return;
        }

        plane_set_mode((enum FlightMode)(plane_flight_modes[switchPosition].get()));

        oldSwitchPosition = switchPosition;
        prev_WP = current_loc;
    }

    if (g.reset_mission_chan != 0 &&
        hal.rcin->read(g.reset_mission_chan-1) > RESET_SWITCH_CHAN_PWM) {
        // reset to first waypoint in mission
        prev_WP = current_loc;
        plane_change_command(0);
    }

    switch_debouncer = false;

    if (g.inverted_flight_ch != 0) {
        // if the user has configured an inverted flight channel, then
        // fly upside down when that channel goes above INVERTED_FLIGHT_PWM
        inverted_flight = (plane_control_mode != PLANE_MANUAL && hal.rcin->read(g.inverted_flight_ch-1) > INVERTED_FLIGHT_PWM);
    }
}

static uint8_t readSwitch(void){
    int16_t pulsewidth = g.rc_5.radio_in;   // default for Arducopter
    if (pulsewidth <= 910 || pulsewidth >= 2090) return 255;            // This is an error condition
    if (pulsewidth > 1230 && pulsewidth <= 1360) return 1;
    if (pulsewidth > 1360 && pulsewidth <= 1490) return 2;
    if (pulsewidth > 1490 && pulsewidth <= 1620) return 3;
    if (pulsewidth > 1620 && pulsewidth <= 1749) return 4;              // Software Manual
    if (pulsewidth >= 1750) return 5;                                                           // Hardware Manual
    return 0;
}

static void reset_control_switch()
{
    oldSwitchPosition = 254;
    read_control_switch();
    plane_read_control_switch();
}

// read_3pos_switch
static uint8_t read_3pos_switch(int16_t radio_in){
    if (radio_in < AUX_SWITCH_PWM_TRIGGER_LOW) return AUX_SWITCH_LOW;      // switch is in low position
    if (radio_in > AUX_SWITCH_PWM_TRIGGER_HIGH) return AUX_SWITCH_HIGH;    // switch is in high position
    return AUX_SWITCH_MIDDLE;                                       // switch is in middle position
}

// read_aux_switches - checks aux switch positions and invokes configured actions
static void read_aux_switches()
{
    uint8_t switch_position;

    // exit immediately during radio failsafe
    if (failsafe.radio || failsafe.radio_counter != 0) {
        return;
    }

    // check if ch7 switch has changed position
    switch_position = read_3pos_switch(g.rc_7.radio_in);
    if (ap.CH7_flag != switch_position) {
        // set the CH7 flag
        ap.CH7_flag = switch_position;

        // invoke the appropriate function
        do_aux_switch_function(g.ch7_option, ap.CH7_flag);
    }

    // check if Ch8 switch has changed position
    switch_position = read_3pos_switch(g.rc_8.radio_in);
    if (ap.CH8_flag != switch_position) {
        // set the CH8 flag
        ap.CH8_flag = switch_position;

        // invoke the appropriate function
        do_aux_switch_function(g.ch8_option, ap.CH8_flag);
    }
}

// init_aux_switches - invoke configured actions at start-up for aux function where it is safe to do so
static void init_aux_switches()
{
    // set the CH7 flag
    ap.CH7_flag = read_3pos_switch(g.rc_7.radio_in);
    ap.CH8_flag = read_3pos_switch(g.rc_8.radio_in);

    // init channel 7 options
    switch(g.ch7_option) {
        case AUX_SWITCH_SIMPLE_MODE:
        case AUX_SWITCH_SONAR:
        case AUX_SWITCH_FENCE:
        case AUX_SWITCH_RESETTOARMEDYAW:
        case AUX_SWITCH_SUPERSIMPLE_MODE:
        case AUX_SWITCH_ACRO_TRAINER:
        case AUX_SWITCH_EPM:
        case AUX_SWITCH_SPRAYER:
            do_aux_switch_function(g.ch7_option, ap.CH7_flag);
            break;
    }
    // init channel 8 option
    switch(g.ch8_option) {
        case AUX_SWITCH_SIMPLE_MODE:
        case AUX_SWITCH_SONAR:
        case AUX_SWITCH_FENCE:
        case AUX_SWITCH_RESETTOARMEDYAW:
        case AUX_SWITCH_SUPERSIMPLE_MODE:
        case AUX_SWITCH_ACRO_TRAINER:
        case AUX_SWITCH_EPM:
        case AUX_SWITCH_SPRAYER:
            do_aux_switch_function(g.ch8_option, ap.CH8_flag);
            break;
    }
}

// do_aux_switch_function - implement the function invoked by the ch7 or ch8 switch
static void do_aux_switch_function(int8_t ch_function, uint8_t ch_flag)
{
    int8_t tmp_function = ch_function;

    // multi mode check
    if(ch_function == AUX_SWITCH_MULTI_MODE) {
        if (g.rc_6.radio_in < CH6_PWM_TRIGGER_LOW) {
            tmp_function = AUX_SWITCH_FLIP;
        }else if (g.rc_6.radio_in > CH6_PWM_TRIGGER_HIGH) {
            tmp_function = AUX_SWITCH_SAVE_WP;
        }else{
            tmp_function = AUX_SWITCH_RTL;
        }
    }

    switch(tmp_function) {
        case AUX_SWITCH_FLIP:
#if FLIP == ENABLED
            // flip if switch is on, positive throttle and we're actually flying
            if((ch_flag == AUX_SWITCH_HIGH) && (g.rc_3.control_in >= 0) && ap.takeoff_complete) {
			init_flip();
            }
#endif
            break;

        case AUX_SWITCH_SIMPLE_MODE:
            // low = simple mode off, middle or high position turns simple mode on
            set_simple_mode(ch_flag == AUX_SWITCH_HIGH || ch_flag == AUX_SWITCH_MIDDLE);
            break;

        case AUX_SWITCH_SUPERSIMPLE_MODE:
            // low = simple mode off, middle = simple mode, high = super simple mode
            set_simple_mode(ch_flag);
            break;

        case AUX_SWITCH_RTL:
            if (ch_flag == AUX_SWITCH_HIGH) {
                // engage RTL (if not possible we remain in current flight mode)
                set_mode(RTL);
            }else{
                // return to flight mode switch's flight mode if we are currently in RTL
                if (control_mode == RTL) {
                    reset_control_switch();
                }
            }
            break;

        case AUX_SWITCH_SAVE_TRIM:
            if ((ch_flag == AUX_SWITCH_HIGH) && (control_mode <= ACRO) && (g.rc_3.control_in == 0)) {
                save_trim();
            }
            break;

        case AUX_SWITCH_SAVE_WP:
            // save waypoint when switch is brought high
            if (ch_flag == AUX_SWITCH_HIGH) {

                // if in auto mode, reset the mission
                if(control_mode == AUTO) {
                    aux_switch_wp_index = 0;
                    g.command_total.set_and_save(1);
                    set_mode(RTL);  // if by chance we are unable to switch to RTL we just stay in AUTO and hope the GPS failsafe will take-over
                    Log_Write_Event(DATA_SAVEWP_CLEAR_MISSION_RTL);
                    return;
                }

				// we're on the ground
				if((g.rc_3.control_in == 0) && (aux_switch_wp_index == 0)){
					// nothing to do
					return;
				}

                // initialise new waypoint to current location
                Location new_wp;

                if(aux_switch_wp_index == 0) {
                    // this is our first WP, let's save WP 1 as a takeoff
                    // increment index to WP index of 1 (home is stored at 0)
                    aux_switch_wp_index = 1;

                    // set our location ID to 16, MAV_CMD_NAV_WAYPOINT
                    new_wp.id = MAV_CMD_NAV_TAKEOFF;
                    new_wp.options = 0;
                    new_wp.p1 = 0;
                    new_wp.lat = 0;
                    new_wp.lng = 0;
                    new_wp.alt = max(current_loc.alt,100);

                    // save command:
                    // we use the current altitude to be the target for takeoff.
                    // only altitude will matter to the AP mission script for takeoff.
                    // If we are above the altitude, we will skip the command.
                    set_cmd_with_index(new_wp, aux_switch_wp_index);
                }

                // initialise new waypoint to current location
                new_wp = current_loc;

                // increment index
                aux_switch_wp_index++;

                // set the next_WP (home is stored at 0)
                // max out at 100 since I think we need to stay under the EEPROM limit
                aux_switch_wp_index = constrain_int16(aux_switch_wp_index, 1, 100);

                if(g.rc_3.control_in > 0) {
                    // set our location ID to 16, MAV_CMD_NAV_WAYPOINT
                    new_wp.id = MAV_CMD_NAV_WAYPOINT;
                }else{
					// set our location ID to 21, MAV_CMD_NAV_LAND
					new_wp.id = MAV_CMD_NAV_LAND;
                }

                // save command
                set_cmd_with_index(new_wp, aux_switch_wp_index);

                // log event
                Log_Write_Event(DATA_SAVEWP_ADD_WP);
            }
            break;

#if CAMERA == ENABLED
        case AUX_SWITCH_CAMERA_TRIGGER:
            if (ch_flag == AUX_SWITCH_HIGH) {
                do_take_picture();
            }
            break;
#endif

        case AUX_SWITCH_SONAR:
            // enable or disable the sonar
            if (ch_flag == AUX_SWITCH_HIGH) {
                g.sonar_enabled = true;
            }else{
                g.sonar_enabled = false;
            }
            break;

#if AC_FENCE == ENABLED
        case AUX_SWITCH_FENCE:
            // enable or disable the fence
            if (ch_flag == AUX_SWITCH_HIGH) {
                fence.enable(true);
                Log_Write_Event(DATA_FENCE_ENABLE);
            }else{
                fence.enable(false);
                Log_Write_Event(DATA_FENCE_DISABLE);
            }
            break;
#endif
        case AUX_SWITCH_RESETTOARMEDYAW:
            if (ch_flag == AUX_SWITCH_HIGH) {
                set_yaw_mode(YAW_RESETTOARMEDYAW);
            }else{
                set_yaw_mode(YAW_HOLD);
            }
            break;

        case AUX_SWITCH_ACRO_TRAINER:
            switch(ch_flag) {
                case AUX_SWITCH_LOW:
                    g.acro_trainer = ACRO_TRAINER_DISABLED;
                    Log_Write_Event(DATA_ACRO_TRAINER_DISABLED);
                    break;
                case AUX_SWITCH_MIDDLE:
                    g.acro_trainer = ACRO_TRAINER_LEVELING;
                    Log_Write_Event(DATA_ACRO_TRAINER_LEVELING);
                    break;
                case AUX_SWITCH_HIGH:
                    g.acro_trainer = ACRO_TRAINER_LIMITED;
                    Log_Write_Event(DATA_ACRO_TRAINER_LIMITED);
                    break;
            }
            break;
#if EPM_ENABLED == ENABLED
        case AUX_SWITCH_EPM:
            switch(ch_flag) {
                case AUX_SWITCH_LOW:
                    epm.off();
                    Log_Write_Event(DATA_EPM_OFF);
                    break;
                case AUX_SWITCH_MIDDLE:
                    epm.neutral();
                    Log_Write_Event(DATA_EPM_NEUTRAL);
                    break;
                case AUX_SWITCH_HIGH:
                    epm.on();
                    Log_Write_Event(DATA_EPM_ON);
                    break;
            }
            break;
#endif
#if SPRAYER == ENABLED
        case AUX_SWITCH_SPRAYER:
            sprayer.enable(ch_flag == AUX_SWITCH_HIGH);
            // if we are disarmed the pilot must want to test the pump
            sprayer.test_pump((ch_flag == AUX_SWITCH_HIGH) && !motors.armed());
            break;
#endif

        case AUX_SWITCH_AUTO:
            if (ch_flag == AUX_SWITCH_HIGH) {
                set_mode(AUTO);
            }else{
                // return to flight mode switch's flight mode if we are currently in AUTO
                if (control_mode == AUTO) {
                    reset_control_switch();
                }
            }
            break;

#if AUTOTUNE == ENABLED
        case AUX_SWITCH_AUTOTUNE:
            // turn on auto tuner
            switch(ch_flag) {
                case AUX_SWITCH_LOW:
                case AUX_SWITCH_MIDDLE:
                    // turn off tuning and return to standard pids
                    if (roll_pitch_mode == ROLL_PITCH_AUTOTUNE) {
                        set_roll_pitch_mode(ROLL_PITCH_STABLE);
                    }
                    break;
                case AUX_SWITCH_HIGH:
                    // start an auto tuning session
                    // set roll-pitch mode to our special auto tuning stabilize roll-pitch mode
                    set_roll_pitch_mode(ROLL_PITCH_AUTOTUNE);
                    break;
            }
            break;
#endif

        case AUX_SWITCH_LAND:
            if (ch_flag == AUX_SWITCH_HIGH) {
                set_mode(LAND);
            }else{
                // return to flight mode switch's flight mode if we are currently in LAND
                if (control_mode == LAND) {
                    reset_control_switch();
                }
            }
            break;
    }
}

// save_trim - adds roll and pitch trims from the radio to ahrs
static void save_trim()
{
    // save roll and pitch trim
    float roll_trim = ToRad((float)g.rc_1.control_in/100.0f);
    float pitch_trim = ToRad((float)g.rc_2.control_in/100.0f);
    ahrs.add_trim(roll_trim, pitch_trim);
    Log_Write_Event(DATA_SAVE_TRIM);
    gcs_send_text_P(SEVERITY_HIGH, PSTR("Trim saved"));
}

// auto_trim - slightly adjusts the ahrs.roll_trim and ahrs.pitch_trim towards the current stick positions
// meant to be called continuously while the pilot attempts to keep the copter level
static void auto_trim()
{
    if(auto_trim_counter > 0) {
        auto_trim_counter--;

        // flash the leds
        AP_Notify::flags.save_trim = true;

        // calculate roll trim adjustment
        float roll_trim_adjustment = ToRad((float)g.rc_1.control_in / 4000.0f);

        // calculate pitch trim adjustment
        float pitch_trim_adjustment = ToRad((float)g.rc_2.control_in / 4000.0f);

        // make sure accelerometer values impact attitude quickly
        ahrs.set_fast_gains(true);

        // add trim to ahrs object
        // save to eeprom on last iteration
        ahrs.add_trim(roll_trim_adjustment, pitch_trim_adjustment, (auto_trim_counter == 0));

        // on last iteration restore leds and accel gains to normal
        if(auto_trim_counter == 0) {
            ahrs.set_fast_gains(false);
            AP_Notify::flags.save_trim = false;
        }
    }
}

#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/crash_check.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Code to detect a crash main ArduCopter code
#ifndef CRASH_CHECK_ITERATIONS_MAX
 # define CRASH_CHECK_ITERATIONS_MAX        20      // 2 second (ie. 10 iterations at 10hz) inverted indicates a crash
#endif
#ifndef CRASH_CHECK_ANGLE_DEVIATION_CD
 # define CRASH_CHECK_ANGLE_DEVIATION_CD    2000    // 20 degrees beyond angle max is signal we are inverted
#endif
#ifndef CRASH_CHECK_ALT_CHANGE_LIMIT_CM
 # define CRASH_CHECK_ALT_CHANGE_LIMIT_CM   50      // baro altitude must not change by more than 50cm
#endif

// crash_check - disarms motors if a crash has been detected
// crashes are detected by the vehicle being more than 20 degrees beyond it's angle limits continuously for more than 1 second
// should be called at 10hz
void crash_check()
{
    static uint8_t inverted_count;  // number of iterations we have been inverted
    static int32_t baro_alt_prev;

    // return immediately if motors are not armed or pilot's throttle is above zero
    if (!motors.armed() || (g.rc_3.control_in != 0 && !failsafe.radio)) {
        inverted_count = 0;
        return;
    }

    // return immediately if we are not in an angle stabilize flight mode or we are flipping
    if (control_mode == ACRO || ap.do_flip) {
        inverted_count = 0;
        return;
    }

    // check angles
    int32_t lean_max = g.angle_max + CRASH_CHECK_ANGLE_DEVIATION_CD;
    if (labs(ahrs.roll_sensor) > lean_max || labs(ahrs.pitch_sensor) > lean_max) {
        inverted_count++;

        // if we have just become inverted record the baro altitude
        if (inverted_count == 1) {
            baro_alt_prev = baro_alt;

        // exit if baro altitude change indicates we are moving (probably falling)
        }else if (labs(baro_alt - baro_alt_prev) > CRASH_CHECK_ALT_CHANGE_LIMIT_CM) {
            inverted_count = 0;
            return;

        // check if inverted for 2 seconds
        }else if (inverted_count >= CRASH_CHECK_ITERATIONS_MAX) {
            // log an error in the dataflash
            Log_Write_Error(ERROR_SUBSYSTEM_CRASH_CHECK, ERROR_CODE_CRASH_CHECK_CRASH);
            // send message to gcs
            gcs_send_text_P(SEVERITY_HIGH,PSTR("Crash: Disarming"));
            // disarm motors
            init_disarm_motors();
        }
    }else{
        // we are not inverted so reset counter
        inverted_count = 0;
    }
}
#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/drift.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

////////////////////////////////////////////////////////////////////////////////
// Drift Mode
////////////////////////////////////////////////////////////////////////////////
#if DRIFT == ENABLED

#ifndef DRIFT_SPEEDGAIN
 # define DRIFT_SPEEDGAIN 14.0
#endif

// The function call for managing the flight mode drift
static void
get_roll_pitch_drift()
{
}

// get_yaw_drift - roll-pitch and yaw controller for drift mode
static void
get_yaw_drift()
{
    static float breaker = 0.0;
    // convert pilot input to lean angles
    // moved to Yaw since it is called before get_roll_pitch_drift();
    get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, control_roll, control_pitch);

    // Grab inertial velocity
    Vector3f vel = inertial_nav.get_velocity();

    // rotate roll, pitch input from north facing to vehicle's perspective
    float roll_vel =  vel.y * cos_yaw - vel.x * sin_yaw; // body roll vel
    float pitch_vel = vel.y * sin_yaw + vel.x * cos_yaw; // body pitch vel

    float pitch_vel2 = min(fabs(pitch_vel), 800);
    // simple gain scheduling for yaw input
    get_yaw_rate_stabilized_ef((float)(control_roll/2) * (1.0 - (pitch_vel2 / 2400.0)));

    roll_vel = constrain_float(roll_vel, -322, 322);
    pitch_vel = constrain_float(pitch_vel, -322, 322);

    // always limit roll
    control_roll = roll_vel * -DRIFT_SPEEDGAIN;
    get_stabilize_roll(control_roll);

    if(control_pitch == 0){
        // .14/ (.03 * 100) = 4.6 seconds till full breaking
        breaker+= .03;
        breaker = min(breaker, DRIFT_SPEEDGAIN);
        // If we let go of sticks, bring us to a stop
        control_pitch = pitch_vel * breaker;
    }else{
        breaker = 0.0;
    }

    // stabilize pitch
    get_stabilize_pitch(control_pitch);
}
#endif
#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/events.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *       This event will be called when the failsafe changes
 *       boolean failsafe reflects the current state
 */
static void failsafe_radio_on_event()
{
    // if motors are not armed there is nothing to do
    if( !motors.armed() ) {
        return;
    }

    // This is how to handle a failsafe.
    switch(control_mode) {
        case STABILIZE:
        case ACRO:
        case SPORT:
            // if throttle is zero disarm motors
            if (g.rc_3.control_in == 0) {
                init_disarm_motors();
            }else if(g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) {
                // if failsafe_throttle is 3 (i.e. FS_THR_ENABLED_ALWAYS_LAND) land immediately
                set_mode(LAND);
            }else if(home_distance > wp_nav.get_waypoint_radius()) {
                if (!set_mode(RTL)) {
                    set_mode(LAND);
                }
            }else{
                // We have no GPS or are very close to home so we will land
                set_mode(LAND);
            }
            break;
        case AUTO:
            // failsafe_throttle is 1 do RTL, 2 means continue with the mission
            if (g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_RTL) {
                if(home_distance > wp_nav.get_waypoint_radius()) {
                    if (!set_mode(RTL)) {
                        set_mode(LAND);
                    }
                }else{
                    // We are very close to home so we will land
                    set_mode(LAND);
                }
            }else if(g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) {
                // if failsafe_throttle is 3 (i.e. FS_THR_ENABLED_ALWAYS_LAND) land immediately
            	set_mode(LAND);
            }
            // if failsafe_throttle is 2 (i.e. FS_THR_ENABLED_CONTINUE_MISSION) no need to do anything
            break;
        case LOITER:
        case ALT_HOLD:
            // if landed with throttle at zero disarm, otherwise do the regular thing
            if (g.rc_3.control_in == 0 && ap.land_complete) {
                init_disarm_motors();
            }else if(g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) {
                // if failsafe_throttle is 3 (i.e. FS_THR_ENABLED_ALWAYS_LAND) land immediately
                set_mode(LAND);
            }else if(home_distance > wp_nav.get_waypoint_radius()) {
                if (!set_mode(RTL)) {
                    set_mode(LAND);
                }
            }else{
                // We have no GPS or are very close to home so we will land
                set_mode(LAND);
            }
            break;
        case LAND:
            // continue to land if battery failsafe is also active otherwise fall through to default handling
            if (g.failsafe_battery_enabled == FS_BATT_LAND && failsafe.battery) {
                break;
            }
        default:
            if(g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) {
                // if failsafe_throttle is 3 (i.e. FS_THR_ENABLED_ALWAYS_LAND) land immediately
                set_mode(LAND);
            }else if(home_distance > wp_nav.get_waypoint_radius()) {
                if (!set_mode(RTL)){
                    set_mode(LAND);
                }
            }else{
                // We have no GPS or are very close to home so we will land
                set_mode(LAND);
            }
            break;
    }

    // log the error to the dataflash
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_OCCURRED);

}

// failsafe_off_event - respond to radio contact being regained
// we must be in AUTO, LAND or RTL modes
// or Stabilize or ACRO mode but with motors disarmed
static void failsafe_radio_off_event()
{
    // no need to do anything except log the error as resolved
    // user can now override roll, pitch, yaw and throttle and even use flight mode switch to restore previous flight mode
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_RESOLVED);
}

static void failsafe_battery_event(void)
{
    // return immediately if low battery event has already been triggered
    if (failsafe.battery) {
        return;
    }

    // failsafe check
    if (g.failsafe_battery_enabled != FS_BATT_DISABLED && motors.armed()) {
        switch(control_mode) {
            case STABILIZE:
            case ACRO:
            case SPORT:
                // if throttle is zero disarm motors
                if (g.rc_3.control_in == 0) {
                    init_disarm_motors();
                }else{
                    // set mode to RTL or LAND
                    if (g.failsafe_battery_enabled == FS_BATT_RTL && home_distance > wp_nav.get_waypoint_radius()) {
                        if (!set_mode(RTL)) {
                            set_mode(LAND);
                        }
                    }else{
                        set_mode(LAND);
                    }
                }
                break;
            case AUTO:
                // set mode to RTL or LAND
                if (home_distance > wp_nav.get_waypoint_radius()) {
                    if (!set_mode(RTL)) {
                        set_mode(LAND);
                    }
                }else{
                    set_mode(LAND);
                }
                break;
            case LOITER:
            case ALT_HOLD:
                // if landed with throttle at zero disarm, otherwise fall through to default handling
                if (g.rc_3.control_in == 0 && ap.land_complete) {
                    init_disarm_motors();
                    break;
                }
            default:
                // set mode to RTL or LAND
                if (g.failsafe_battery_enabled == FS_BATT_RTL && home_distance > wp_nav.get_waypoint_radius()) {
                    if (!set_mode(RTL)) {
                        set_mode(LAND);
                    }
                }else{
                    set_mode(LAND);
                }
                break;
        }
    }

    // set the low battery flag
    set_failsafe_battery(true);

    // warn the ground station and log to dataflash
    gcs_send_text_P(SEVERITY_LOW,PSTR("Low Battery!"));
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_BATT, ERROR_CODE_FAILSAFE_OCCURRED);

}

// failsafe_gps_check - check for gps failsafe
static void failsafe_gps_check()
{
    uint32_t last_gps_update_ms;

    // return immediately if gps failsafe is disabled or we have never had GPS lock
    if (g.failsafe_gps_enabled == FS_GPS_DISABLED || !ap.home_is_set) {
        // if we have just disabled the gps failsafe, ensure the gps failsafe event is cleared
        if (failsafe.gps) {
            failsafe_gps_off_event();
            set_failsafe_gps(false);
        }
        return;
    }

    // calc time since last gps update
    last_gps_update_ms = millis() - gps_glitch.last_good_update();

    // check if all is well
    if( last_gps_update_ms < FAILSAFE_GPS_TIMEOUT_MS) {
        // check for recovery from gps failsafe
        if( failsafe.gps ) {
            failsafe_gps_off_event();
            set_failsafe_gps(false);
        }
        return;
    }

    // do nothing if gps failsafe already triggered or motors disarmed
    if( failsafe.gps || !motors.armed()) {
        return;
    }

    // GPS failsafe event has occured
    // update state, warn the ground station and log to dataflash
    set_failsafe_gps(true);
    gcs_send_text_P(SEVERITY_LOW,PSTR("Lost GPS!"));
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GPS, ERROR_CODE_FAILSAFE_OCCURRED);

    // take action based on flight mode and FS_GPS_ENABLED parameter
    if (mode_requires_GPS(control_mode) || g.failsafe_gps_enabled == FS_GPS_LAND_EVEN_STABILIZE) {
        if (g.failsafe_gps_enabled == FS_GPS_ALTHOLD && !failsafe.radio) {
            set_mode(ALT_HOLD);
        }else{
            set_mode(LAND);
        }
    }
}

// failsafe_gps_off_event - actions to take when GPS contact is restored
static void failsafe_gps_off_event(void)
{
    // log recovery of GPS in logs?
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GPS, ERROR_CODE_FAILSAFE_RESOLVED);
}

// failsafe_gcs_check - check for ground station failsafe
static void failsafe_gcs_check()
{
    uint32_t last_gcs_update_ms;

    // return immediately if gcs failsafe is disabled, gcs has never been connected or we are not overriding rc controls from the gcs
    if( g.failsafe_gcs == FS_GCS_DISABLED || failsafe.last_heartbeat_ms == 0 || !failsafe.rc_override_active) {
        return;
    }

    // calc time since last gcs update
    last_gcs_update_ms = millis() - failsafe.last_heartbeat_ms;

    // check if all is well
    if( last_gcs_update_ms < FS_GCS_TIMEOUT_MS) {
        // check for recovery from gcs failsafe
        if (failsafe.gcs) {
            failsafe_gcs_off_event();
            set_failsafe_gcs(false);
        }
        return;
    }

    // do nothing if gcs failsafe already triggered or motors disarmed
    if( failsafe.gcs || !motors.armed()) {
        return;
    }

    // GCS failsafe event has occured
    // update state, log to dataflash
    set_failsafe_gcs(true);
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GCS, ERROR_CODE_FAILSAFE_OCCURRED);

    // This is how to handle a failsafe.
    // use the throttle failsafe setting to decide what to do
    switch(control_mode) {
        case STABILIZE:
        case ACRO:
        case SPORT:
            // if throttle is zero disarm motors
            if (g.rc_3.control_in == 0) {
                init_disarm_motors();
            }else if(home_distance > wp_nav.get_waypoint_radius()) {
                if (!set_mode(RTL)) {
                    set_mode(LAND);
                }
            }else{
                // We have no GPS or are very close to home so we will land
                set_mode(LAND);
            }
            break;
        case AUTO:
            // if g.failsafe_gcs is 1 do RTL, 2 means continue with the mission
            if (g.failsafe_gcs == FS_GCS_ENABLED_ALWAYS_RTL) {
                if (home_distance > wp_nav.get_waypoint_radius()) {
                    if (!set_mode(RTL)) {
                        set_mode(LAND);
                    }
                }else{
                    // We are very close to home so we will land
                    set_mode(LAND);
                }
            }
            // if failsafe_throttle is 2 (i.e. FS_THR_ENABLED_CONTINUE_MISSION) no need to do anything
            break;
        default:
            if(home_distance > wp_nav.get_waypoint_radius()) {
                if (!set_mode(RTL)) {
                    set_mode(LAND);
                }
            }else{
                // We have no GPS or are very close to home so we will land
                set_mode(LAND);
            }
            break;
    }
}

// failsafe_gcs_off_event - actions to take when GCS contact is restored
static void failsafe_gcs_off_event(void)
{
    // log recovery of GCS in logs?
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GCS, ERROR_CODE_FAILSAFE_RESOLVED);
}

static void failsafe_short_on_event(enum failsafe_state fstype)
{
    // This is how to handle a short loss of control signal plane_failsafe.
    plane_failsafe.state = fstype;
    plane_failsafe.ch3_timer_ms = millis();
    gcs_send_text_P(SEVERITY_LOW, PSTR("Failsafe - Short event on, "));
    switch(plane_control_mode)
    {
    case PLANE_MANUAL:
    case PLANE_STABILIZE:
    case PLANE_ACRO:
    case PLANE_FLY_BY_WIRE_A:
    case PLANE_FLY_BY_WIRE_B:
    case PLANE_CRUISE:
    case PLANE_TRAINING:
        plane_failsafe.saved_mode = plane_control_mode;
        plane_failsafe.saved_mode_set = 1;
        if(g.short_fs_action == 2) {
            plane_set_mode(PLANE_FLY_BY_WIRE_A);
        } else {
            plane_set_mode(PLANE_CIRCLE);
        }
        break;

    case PLANE_AUTO:
    case PLANE_GUIDED:
    case PLANE_LOITER:
        if(g.short_fs_action != 0) {
            plane_failsafe.saved_mode = plane_control_mode;
            plane_failsafe.saved_mode_set = 1;
            if(g.short_fs_action == 2) {
                plane_set_mode(PLANE_FLY_BY_WIRE_A);
            } else {
                plane_set_mode(PLANE_CIRCLE);
            }
        }
        break;

    case PLANE_CIRCLE:
    case PLANE_RTL:
    default:
        break;
    }
    plane_gcs_send_text_fmt(PSTR("flight mode = %u"), (unsigned)plane_control_mode);
}

static void failsafe_long_on_event(enum failsafe_state fstype)
{
    // This is how to handle a long loss of control signal plane_failsafe.
    gcs_send_text_P(SEVERITY_LOW, PSTR("Failsafe - Long event on, "));
    //  If the GCS is locked up we allow control to revert to RC
    hal.rcin->clear_overrides();
    plane_failsafe.state = fstype;
    switch(plane_control_mode)
    {
    case PLANE_MANUAL:
    case PLANE_STABILIZE:
    case PLANE_ACRO:
    case PLANE_FLY_BY_WIRE_A:
    case PLANE_FLY_BY_WIRE_B:
    case PLANE_CRUISE:
    case PLANE_TRAINING:
    case PLANE_CIRCLE:
        if(g.long_fs_action == 2) {
            plane_set_mode(PLANE_FLY_BY_WIRE_A);
        } else {
            plane_set_mode(PLANE_RTL);
        }
        break;

    case PLANE_AUTO:
    case PLANE_GUIDED:
    case PLANE_LOITER:
        if(g.long_fs_action == 2) {
            plane_set_mode(PLANE_FLY_BY_WIRE_A);
        } else if (g.long_fs_action == 1) {
            plane_set_mode(PLANE_RTL);
        }
        break;

    case PLANE_RTL:
    default:
        break;
    }
    plane_gcs_send_text_fmt(PSTR("flight mode = %u"), (unsigned)plane_control_mode);
}

static void failsafe_short_off_event()
{
    // We're back in radio contact
    gcs_send_text_P(SEVERITY_LOW, PSTR("Failsafe - Short event off"));
    plane_failsafe.state = FAILSAFE_NONE;

    // re-read the switch so we can return to our preferred mode
    // --------------------------------------------------------
    if (plane_control_mode == PLANE_CIRCLE && plane_failsafe.saved_mode_set) {
        plane_failsafe.saved_mode_set = 0;
        plane_set_mode(plane_failsafe.saved_mode);
    }
}

void low_battery_event(void)
{
    if (plane_failsafe.low_battery) {
        return;
    }
    plane_gcs_send_text_fmt(PSTR("Low Battery %.2fV Used %.0f mAh"),
                      battery.voltage(), battery.current_total_mah());
    plane_set_mode(PLANE_RTL);
    aparm.plthr_cruise.load();
    plane_failsafe.low_battery = true;
    AP_Notify::flags.failsafe_battery = true;
}

static void update_events()
{
    ServoRelayEvents.update_events();
}

#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/failsafe.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
//  failsafe support
//  Andrew Tridgell, December 2011
//
//  our failsafe strategy is to detect main loop lockup and disarm the motors
//

static bool failsafe_enabled = true;
static uint16_t last_mainLoop_count;
static uint32_t last_timestamp;
static bool in_failsafe;

//
// failsafe_enable - enable failsafe
//
void failsafe_enable()
{
    failsafe_enabled = true;
    last_timestamp = micros();
}

//
// failsafe_disable - used when we know we are going to delay the mainloop significantly
//
void failsafe_disable()
{
    failsafe_enabled = false;
}

//
//  failsafe_check - this function is called from the core timer interrupt at 1kHz.
//
void failsafe_check()
{
    uint32_t tnow = hal.scheduler->micros();

    if (mainLoop_count != last_mainLoop_count) {
        // the main loop is running, all is OK
        last_mainLoop_count = mainLoop_count;
        last_timestamp = tnow;
        in_failsafe = false;
        return;
    }

	if (!isplane){
		if (failsafe_enabled && tnow - last_timestamp > 2000000) {
			// motors are running but we have gone 2 second since the
			// main loop ran. That means we're in trouble and should
			// disarm the motors.
			in_failsafe = true;
		}
	
    if (failsafe_enabled && in_failsafe && tnow - last_timestamp > 1000000) {
        // disarm motors every second
        last_timestamp = tnow;
        if(motors.armed()) {
            motors.armed(false);
            motors.output();
        }
    }
	}
	
	if (isplane){
    if (tnow - last_timestamp > 200000) {
        // we have gone at least 0.2 seconds since the main loop
        // ran. That means we're in trouble, or perhaps are in
        // an initialisation routine or log erase. Start passing RC
        // inputs through to outputs
        in_failsafe = true;
    }

    if (in_failsafe && tnow - last_timestamp > 20000) {
        // pass RC inputs to outputs every 20ms
        last_timestamp = tnow;
        hal.rcin->clear_overrides();
        channel_roll->radio_out     = channel_roll->read();
        channel_pitch->radio_out    = channel_pitch->read();
        channel_throttle->radio_out = channel_throttle->read();
        channel_rudder->radio_out   = channel_rudder->read();

        // setup secondary output channels that don't have
        // corresponding input channels
        RC_Channel_aux::set_servo_out(RC_Channel_aux::k_aileron, channel_roll->radio_out);
        RC_Channel_aux::set_servo_out(RC_Channel_aux::k_elevator, channel_pitch->radio_out);
        RC_Channel_aux::set_servo_out(RC_Channel_aux::k_rudder, channel_rudder->radio_out);

        if (g.vtail_output != MIXING_DISABLED) {
            channel_output_mixer(g.vtail_output, channel_pitch->radio_out, channel_rudder->radio_out);
        } else if (g.elevon_output != MIXING_DISABLED) {
            channel_output_mixer(g.elevon_output, channel_pitch->radio_out, channel_roll->radio_out);
        }
        if (!demoing_servos) {
            channel_roll->output();
        }
        channel_pitch->output();
        channel_throttle->output();
        channel_rudder->output();

        // setup secondary output channels that do have
        // corresponding input channels
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_manual, true);
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_aileron_with_input, true);
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_elevator_with_input, true);
    }
	}
}
#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/fence.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Code to integrate AC_Fence library with main ArduCopter code

#if AC_FENCE == ENABLED

uint8_t lim_state = 0, lim_old_state = 0;

// fence_check - ask fence library to check for breaches and initiate the response
// called at 1hz
void fence_check()
{
    uint8_t new_breaches; // the type of fence that has been breached
    uint8_t orig_breaches = fence.get_breaches();

    // return immediately if motors are not armed
    if(!motors.armed()) {
        return;
    }

    // give fence library our current distance from home in meters
    fence.set_home_distance(home_distance*0.01f);

    // check for a breach
    new_breaches = fence.check_fence();

    // if there is a new breach take action
    if( new_breaches != AC_FENCE_TYPE_NONE ) {

        // if the user wants some kind of response and motors are armed
        if(fence.get_action() != AC_FENCE_ACTION_REPORT_ONLY ) {

            // disarm immediately if we think we are on the ground
            // don't disarm if the high-altitude fence has been broken because it's likely the user has pulled their throttle to zero to bring it down
            if(manual_flight_mode(control_mode) && g.rc_3.control_in == 0 && !failsafe.radio && ((fence.get_breaches() & AC_FENCE_TYPE_ALT_MAX)== 0)){
                init_disarm_motors();
            }else{
                // if we are within 100m of the fence, RTL
                if (fence.get_breach_distance(new_breaches) <= AC_FENCE_GIVE_UP_DISTANCE) {
                    if (!set_mode(RTL)) {
                        set_mode(LAND);
                    }
                }else{
                    // if more than 100m outside the fence just force a land
                    set_mode(LAND);
                }
            }
        }

        // log an error in the dataflash
        Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_FENCE, new_breaches);
    }

    // record clearing of breach
    if(orig_breaches != AC_FENCE_TYPE_NONE && fence.get_breaches() == AC_FENCE_TYPE_NONE) {
        Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_FENCE, ERROR_CODE_ERROR_RESOLVED);
    }
}

// fence_send_mavlink_status - send fence status to ground station
static void fence_send_mavlink_status(mavlink_channel_t chan)
{   
    if (fence.enabled()) {
        // traslate fence library breach types to mavlink breach types
        uint8_t mavlink_breach_type = FENCE_BREACH_NONE;
        uint8_t breaches = fence.get_breaches();
        if ((breaches & AC_FENCE_TYPE_ALT_MAX) != 0) {
            mavlink_breach_type = FENCE_BREACH_MAXALT;
        }
        if ((breaches & AC_FENCE_TYPE_CIRCLE) != 0) {
            mavlink_breach_type = FENCE_BREACH_BOUNDARY;
        }

        // send status
        mavlink_msg_fence_status_send(chan,
                                      (int8_t)(fence.get_breaches()!=0),
                                      fence.get_breach_count(),
                                      mavlink_breach_type,
                                      fence.get_breach_time());
    }
}

#endif
#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/flip.pde"
// 2010 Jose Julio
// 2011 Adapted and updated for AC2 by Jason Short
//
// Automatic Acrobatic Procedure (AAP) v1 : Roll flip
// State machine aproach:
//    Some states are fixed commands (for a fixed time)
//    Some states are fixed commands (until some IMU condition)
//    Some states include controls inside
#if FLIP == ENABLED
uint8_t flip_timer;
uint8_t flip_state;

#define AAP_THR_INC 170
#define AAP_THR_DEC 120
#define AAP_ROLL_OUT 2000

static int8_t flip_dir;

void init_flip()
{
    if(false == ap.do_flip) {
        ap.do_flip = true;
        flip_state = 0;
        flip_dir = (ahrs.roll_sensor >= 0) ? 1 : -1;
		Log_Write_Event(DATA_BEGIN_FLIP);
    }
}

void roll_flip()
{
    int32_t roll = ahrs.roll_sensor * flip_dir;

    // Roll State machine
    switch (flip_state) {
    case 0:
        if (roll < 4500) {
            // Roll control
			roll_rate_target_bf     = 40000 * flip_dir;
		    if (throttle_mode_manual(throttle_mode)){
    		    // increase throttle right before flip
                set_throttle_out(g.rc_3.control_in + AAP_THR_INC, false);
            }
        }else{
            flip_state++;
        }
        break;

    case 1:
        if((roll >= 4500) || (roll < -9000)) {
		    #if FRAME_CONFIG == HELI_FRAME
				roll_rate_target_bf = 40000 * flip_dir;
		    #else
			    roll_rate_target_bf = 40000 * flip_dir;
		    #endif
		    // decrease throttle while inverted
		    if (throttle_mode_manual(throttle_mode)){
                set_throttle_out(g.rc_3.control_in - AAP_THR_DEC, false);
            }
        }else{
            flip_state++;
            flip_timer = 0;
        }
        break;

    case 2:
        // 100 = 1 second with 100hz
        if (flip_timer < 100) {
            // we no longer need to adjust the roll_rate. 
            // It will be handled by normal flight control loops

            // increase throttle to gain any lost alitude
            if (throttle_mode_manual(throttle_mode)){
                set_throttle_out(g.rc_3.control_in + AAP_THR_INC, false);
            }
            flip_timer++;
        }else{
        	Log_Write_Event(DATA_END_FLIP);
            ap.do_flip = false;
            flip_state = 0;
        }
        break;
    }
}
#endif
#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/geofence.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *  geo-fencing support
 *  Andrew Tridgell, December 2011
 */

#if AC_FENCE == ENABLED

/*
 *  The state of geo-fencing. This structure is dynamically allocated
 *  the first time it is used. This means we only pay for the pointer
 *  and not the structure on systems where geo-fencing is not being
 *  used.
 *
 *  We store a copy of the boundary in memory as we need to access it
 *  very quickly at runtime
 */
static struct GeofenceState {
    uint8_t num_points;
    bool boundary_uptodate;
    bool fence_triggered;
    uint16_t breach_count;
    uint8_t breach_type;
    uint32_t breach_time;
    uint8_t old_switch_position;
    /* point 0 is the return point */
    Vector2l boundary[MAX_FENCEPOINTS];
} *geofence_state;


/*
 *  fence boundaries fetch/store
 */
static Vector2l get_fence_point_with_index(unsigned i)
{
    uint16_t mem;
    Vector2l ret;

    if (i > (unsigned)g.fence_total) {
        return Vector2l(0,0);
    }

    // read fence point
    mem = FENCE_START_BYTE + (i * FENCE_WP_SIZE);
    ret.x = hal.storage->read_dword(mem);
    mem += sizeof(uint32_t);
    ret.y = hal.storage->read_dword(mem);

    return ret;
}

// save a fence point
static void set_fence_point_with_index(Vector2l &point, unsigned i)
{
    uint16_t mem;

    if (i >= (unsigned)g.fence_total.get()) {
        // not allowed
        return;
    }

    mem = FENCE_START_BYTE + (i * FENCE_WP_SIZE);

    hal.storage->write_dword(mem, point.x);
    mem += sizeof(uint32_t);
    hal.storage->write_dword(mem, point.y);

    if (geofence_state != NULL) {
        geofence_state->boundary_uptodate = false;
    }
}

/*
 *  allocate and fill the geofence state structure
 */
static void geofence_load(void)
{
    uint8_t i;

    if (geofence_state == NULL) {
        if (hal.util->available_memory() < 512 + sizeof(struct GeofenceState)) {
            // too risky to enable as we could run out of stack
            goto failed;
        }
        geofence_state = (struct GeofenceState *)calloc(1, sizeof(struct GeofenceState));
        if (geofence_state == NULL) {
            // not much we can do here except disable it
            goto failed;
        }
        geofence_state->old_switch_position = 254;
    }

    if (g.fence_total <= 0) {
        g.fence_total.set(0);
        return;
    }

    for (i=0; i<g.fence_total; i++) {
        geofence_state->boundary[i] = get_fence_point_with_index(i);
    }
    geofence_state->num_points = i;

    if (!Polygon_complete(&geofence_state->boundary[1], geofence_state->num_points-1)) {
        // first point and last point must be the same
        goto failed;
    }
    if (Polygon_outside(geofence_state->boundary[0], &geofence_state->boundary[1], geofence_state->num_points-1)) {
        // return point needs to be inside the fence
        goto failed;
    }

    geofence_state->boundary_uptodate = true;
    geofence_state->fence_triggered = false;

    gcs_send_text_P(SEVERITY_LOW,PSTR("geo-fence loaded"));
    gcs_send_message(MSG_FENCE_STATUS);
    return;

failed:
    g.fence_action.set(FENCE_ACTION_NONE);
    gcs_send_text_P(SEVERITY_HIGH,PSTR("geo-fence setup error"));
}

/*
 *  return true if geo-fencing is enabled
 */
static bool geofence_enabled(void)
{
    if (g.fence_action == FENCE_ACTION_NONE ||
        g.fence_total < 5 ||
        (g.fence_action != FENCE_ACTION_REPORT &&
         (g.fence_channel == 0 ||
          hal.rcin->read(g.fence_channel-1) < FENCE_ENABLE_PWM))) {
        // geo-fencing is disabled
        if (geofence_state != NULL) {
            // re-arm for when the channel trigger is switched on
            geofence_state->fence_triggered = false;
        }
        return false;
    }

    return true;
}


/*
 *  return true if we have breached the geo-fence minimum altiude
 */
static bool geofence_check_minalt(void)
{
    if (g.fence_maxalt <= g.fence_minalt) {
        return false;
    }
    if (g.fence_minalt == 0) {
        return false;
    }
    return (adjusted_altitude_cm() < (g.fence_minalt*100.0) + home.alt);
}

/*
 *  return true if we have breached the geo-fence maximum altiude
 */
static bool geofence_check_maxalt(void)
{
    if (g.fence_maxalt <= g.fence_minalt) {
        return false;
    }
    if (g.fence_maxalt == 0) {
        return false;
    }
    return (adjusted_altitude_cm() > (g.fence_maxalt*100.0) + home.alt);
}


/*
 *  check if we have breached the geo-fence
 */
static void geofence_check(bool altitude_check_only)
{
    if (!geofence_enabled()) {
        // switch back to the chosen control mode if still in
        // PLANE_GUIDED to the return point
        if (geofence_state != NULL &&
            (g.fence_action == FENCE_ACTION_GUIDED || g.fence_action == FENCE_ACTION_GUIDED_THR_PASS) &&
            g.fence_channel != 0 &&
            plane_control_mode == PLANE_GUIDED &&
            g.fence_total >= 5 &&
            geofence_state->boundary_uptodate &&
            geofence_state->old_switch_position == oldSwitchPosition &&
            guided_WP.lat == geofence_state->boundary[0].x &&
            guided_WP.lng == geofence_state->boundary[0].y) {
            geofence_state->old_switch_position = 254;
            reset_control_switch();
        }
        return;
    }

    /* allocate the geo-fence state if need be */
    if (geofence_state == NULL || !geofence_state->boundary_uptodate) {
        geofence_load();
        if (!geofence_enabled()) {
            // may have been disabled by load
            return;
        }
    }

    bool outside = false;
    uint8_t breach_type = FENCE_BREACH_NONE;
    struct Location loc;

    if (geofence_check_minalt()) {
        outside = true;
        breach_type = FENCE_BREACH_MINALT;
    } else if (geofence_check_maxalt()) {
        outside = true;
        breach_type = FENCE_BREACH_MAXALT;
    } else if (!altitude_check_only && ahrs.get_projected_position(loc)) {
        Vector2l location;
        location.x = loc.lat;
        location.y = loc.lng;
        outside = Polygon_outside(location, &geofence_state->boundary[1], geofence_state->num_points-1);
        if (outside) {
            breach_type = FENCE_BREACH_BOUNDARY;
        }
    }

    if (!outside) {
        if (geofence_state->fence_triggered && !altitude_check_only) {
            // we have moved back inside the fence
            geofence_state->fence_triggered = false;
            gcs_send_text_P(SEVERITY_LOW,PSTR("geo-fence OK"));
 #if FENCE_TRIGGERED_PIN > 0
            digitalWrite(FENCE_TRIGGERED_PIN, LOW);
 #endif
            gcs_send_message(MSG_FENCE_STATUS);
        }
        // we're inside, all is good with the world
        return;
    }

    // we are outside the fence
    if (geofence_state->fence_triggered &&
        (plane_control_mode == PLANE_GUIDED || g.fence_action == FENCE_ACTION_REPORT)) {
        // we have already triggered, don't trigger again until the
        // user disables/re-enables using the fence channel switch
        return;
    }

    // we are outside, and have not previously triggered.
    geofence_state->fence_triggered = true;
    geofence_state->breach_count++;
    geofence_state->breach_time = millis();
    geofence_state->breach_type = breach_type;

 #if FENCE_TRIGGERED_PIN > 0
    digitalWrite(FENCE_TRIGGERED_PIN, HIGH);
 #endif

    gcs_send_text_P(SEVERITY_LOW,PSTR("geo-fence triggered"));
    gcs_send_message(MSG_FENCE_STATUS);

    // see what action the user wants
    switch (g.fence_action) {
    case FENCE_ACTION_REPORT:
        break;

    case FENCE_ACTION_GUIDED:
    case FENCE_ACTION_GUIDED_THR_PASS:
        if (g.fence_retalt > 0) {
            //fly to the return point using fence_retalt
            guided_WP.alt = home.alt + 100.0*g.fence_retalt;
        } else if (g.fence_minalt >= g.fence_maxalt) {
            // invalid min/max, use RTL_altitude
            guided_WP.alt = home.alt + g.RTL_altitude_cm;
        } else {
            // fly to the return point, with an altitude half way between
            // min and max
            guided_WP.alt = home.alt + 100.0*(g.fence_minalt + g.fence_maxalt)/2;
        }
        guided_WP.id = 0;
        guided_WP.p1  = 0;
        guided_WP.options = 0;
        guided_WP.lat = geofence_state->boundary[0].x;
        guided_WP.lng = geofence_state->boundary[0].y;

        geofence_state->old_switch_position = oldSwitchPosition;

        set_guided_WP();

        if (plane_control_mode == PLANE_MANUAL && g.auto_trim) {
            // make sure we don't auto trim the surfaces on this change
            plane_control_mode = PLANE_STABILIZE;
        }

        plane_set_mode(PLANE_GUIDED);
        if (g.fence_action == FENCE_ACTION_GUIDED_THR_PASS) {
            guided_throttle_passthru = true;
        }
        break;
    }

}

/*
 *  return true if geofencing allows stick mixing. When we have
 *  triggered failsafe and are in PLANE_GUIDED mode then stick mixing is
 *  disabled. Otherwise the aircraft may not be able to recover from
 *  a breach of the geo-fence
 */
static bool geofence_stickmixing(void) {
    if (geofence_enabled() &&
        geofence_state != NULL &&
        geofence_state->fence_triggered &&
        plane_control_mode == PLANE_GUIDED) {
        // don't mix in user input
        return false;
    }
    // normal mixing rules
    return true;
}

/*
 *
 */
static void geofence_send_status(mavlink_channel_t chan)
{
    if (geofence_enabled() && geofence_state != NULL) {
        mavlink_msg_fence_status_send(chan,
                                      (int8_t)geofence_state->fence_triggered,
                                      geofence_state->breach_count,
                                      geofence_state->breach_type,
                                      geofence_state->breach_time);
    }
}

// public function for use in failsafe modules
bool geofence_breached(void)
{
    return geofence_state ? geofence_state->fence_triggered : false;
}


#else // GEOFENCE_ENABLED

static void geofence_check(bool altitude_check_only) {
}
static bool geofence_stickmixing(void) {
    return true;
}
static bool geofence_enabled(void) {
    return false;
}

#endif // GEOFENCE_ENABLED
#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/heli.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Traditional helicopter variables and functions

#if FRAME_CONFIG == HELI_FRAME

#ifndef HELI_DYNAMIC_FLIGHT_SPEED_MIN
 #define HELI_DYNAMIC_FLIGHT_SPEED_MIN      500     // we are in "dynamic flight" when the speed is over 1m/s for 2 seconds
#endif

// counter to control dynamic flight profile
static int8_t heli_dynamic_flight_counter;

// Tradheli flags
static struct {
    uint8_t dynamic_flight  : 1;    // 0   // true if we are moving at a significant speed (used to turn on/off leaky I terms)
} heli_flags;

#if HELI_CC_COMP == ENABLED
static LowPassFilterFloat rate_dynamics_filter;     // Rate Dynamics filter
#endif

// heli_init - perform any special initialisation required for the tradheli
static void heli_init()
{
#if HELI_CC_COMP == ENABLED
    rate_dynamics_filter.set_cutoff_frequency(0.01f, 4.0f);
#endif
}

// get_pilot_desired_collective - converts pilot input (from 0 ~ 1000) to a value that can be fed into the g.rc_3.servo_out function
static int16_t get_pilot_desired_collective(int16_t control_in)
{
    // return immediately if reduce collective range for manual flight has not been configured
    if (g.heli_stab_col_min == 0 && g.heli_stab_col_max == 1000) {
        return control_in;
    }

    // scale pilot input to reduced collective range
    float scalar = ((float)(g.heli_stab_col_max - g.heli_stab_col_min))/1000.0f;
    int16_t collective_out = g.heli_stab_col_min + control_in * scalar;
    collective_out = constrain_int16(collective_out, 0, 1000);
    return collective_out;
}

// heli_check_dynamic_flight - updates the dynamic_flight flag based on our horizontal velocity
// should be called at 50hz
static void check_dynamic_flight(void)
{
    if (!motors.armed() || throttle_mode == THROTTLE_LAND || !motors.motor_runup_complete()) {
        heli_dynamic_flight_counter = 0;
        heli_flags.dynamic_flight = false;
        return;
    }

    bool moving = false;

    // with GPS lock use inertial nav to determine if we are moving
    if (GPS_ok()) {
        // get horizontal velocity
        float velocity = inertial_nav.get_velocity_xy();
        moving = (velocity >= HELI_DYNAMIC_FLIGHT_SPEED_MIN);
    }else{
        // with no GPS lock base it on throttle and forward lean angle
        moving = (g.rc_3.servo_out > 800 || ahrs.pitch_sensor < -1500);
    }

    if (moving) {
        // if moving for 2 seconds, set the dynamic flight flag
        if (!heli_flags.dynamic_flight) {
            heli_dynamic_flight_counter++;
            if (heli_dynamic_flight_counter >= 100) {
                heli_flags.dynamic_flight = true;
                heli_dynamic_flight_counter = 100;
            }
        }
    }else{
        // if not moving for 2 seconds, clear the dynamic flight flag
        if (heli_flags.dynamic_flight) {
            if (heli_dynamic_flight_counter > 0) {
                heli_dynamic_flight_counter--;
            }else{
                heli_flags.dynamic_flight = false;
            }
        }
    }
}

// heli_integrated_swash_controller - convert desired roll and pitch rate to roll and pitch swash angles
// should be called at 100hz
// output placed directly into g.rc_1.servo_out and g.rc_2.servo_out
static void heli_integrated_swash_controller(int32_t target_roll_rate, int32_t target_pitch_rate)
{
    int32_t         roll_p, roll_i, roll_d, roll_ff;            // used to capture pid values for logging
    int32_t         pitch_p, pitch_i, pitch_d, pitch_ff;
	int32_t         current_roll_rate, current_pitch_rate;	    // this iteration's rate
    int32_t         roll_rate_error, pitch_rate_error;          // simply target_rate - current_rate
    int32_t         roll_output, pitch_output;                  // output from pid controller
    static bool     roll_pid_saturated, pitch_pid_saturated;    // tracker from last loop if the PID was saturated

    current_roll_rate = (omega.x * DEGX100);                    // get current roll rate
    current_pitch_rate = (omega.y * DEGX100);                   // get current pitch rate

    roll_rate_error = target_roll_rate - current_roll_rate;
    pitch_rate_error = target_pitch_rate - current_pitch_rate;

    roll_p = g.pid_rate_roll.get_p(roll_rate_error);
    pitch_p = g.pid_rate_pitch.get_p(pitch_rate_error);

    if (roll_pid_saturated){
        roll_i = g.pid_rate_roll.get_integrator();                                                      // Locked Integrator due to PID saturation on previous cycle
    } else {
        if (motors.has_flybar()) {                                                                      // Mechanical Flybars get regular integral for rate auto trim
            if (target_roll_rate > -50 && target_roll_rate < 50){								        // Frozen at high rates
                roll_i = g.pid_rate_roll.get_i(roll_rate_error, G_Dt);
            } else {
                roll_i = g.pid_rate_roll.get_integrator();
            }
        } else {
            if (heli_flags.dynamic_flight){
                roll_i = g.pid_rate_roll.get_i(roll_rate_error, G_Dt);
            } else {
                roll_i = g.pid_rate_roll.get_leaky_i(roll_rate_error, G_Dt, RATE_INTEGRATOR_LEAK_RATE);
            }
        }
    }

    if (pitch_pid_saturated){
        pitch_i = g.pid_rate_pitch.get_integrator();                                                    // Locked Integrator due to PID saturation on previous cycle
    } else {
        if (motors.has_flybar()) {                                                                      // Mechanical Flybars get regular integral for rate auto trim
            if (target_pitch_rate > -50 && target_pitch_rate < 50){								        // Frozen at high rates
                pitch_i = g.pid_rate_pitch.get_i(pitch_rate_error, G_Dt);
            } else {
                pitch_i = g.pid_rate_pitch.get_integrator();
            }
        } else {
            if (heli_flags.dynamic_flight){
                pitch_i = g.pid_rate_pitch.get_i(pitch_rate_error, G_Dt);
            } else {
                pitch_i = g.pid_rate_pitch.get_leaky_i(pitch_rate_error, G_Dt, RATE_INTEGRATOR_LEAK_RATE);
            }
        }
    }

    roll_d = g.pid_rate_roll.get_d(target_roll_rate, G_Dt);
    pitch_d = g.pid_rate_pitch.get_d(target_pitch_rate, G_Dt);

    roll_ff = g.heli_roll_ff * target_roll_rate;
    pitch_ff = g.heli_pitch_ff * target_pitch_rate;
    
    roll_output = roll_p + roll_i + roll_d + roll_ff;
    pitch_output = pitch_p + pitch_i + pitch_d + pitch_ff;

#if HELI_CC_COMP == ENABLED
// Do cross-coupling compensation for low rpm helis
// Credit: Jolyon Saunders
// Note: This is not widely tested at this time.  Will not be used by default yet.
    float cc_axis_ratio = 2.0f; // Ratio of compensation on pitch vs roll axes. Number >1 means pitch is affected more than roll
    float cc_kp = 0.0002f;      // Compensation p term. Setting this to zero gives h_phang only, while increasing it will increase the p term of correction
    float cc_kd = 0.127f;       // Compensation d term, scaled. This accounts for flexing of the blades, dampers etc. Originally was (motors.ext_gyro_gain * 0.0001)
    float cc_angle, cc_total_output;
    uint32_t cc_roll_d, cc_pitch_d, cc_sum_d;
    int32_t cc_scaled_roll;
    int32_t cc_roll_output;     // Used to temporarily hold output while rotation is being calculated
    int32_t cc_pitch_output;    // Used to temporarily hold output while rotation is being calculated
    static int32_t last_roll_output = 0;
    static int32_t last_pitch_output = 0;

    cc_scaled_roll  = roll_output / cc_axis_ratio; // apply axis ratio to roll
    cc_total_output = safe_sqrt(cc_scaled_roll * cc_scaled_roll + pitch_output * pitch_output) * cc_kp;
    
    // find the delta component
    cc_roll_d  = (roll_output - last_roll_output) / cc_axis_ratio;
    cc_pitch_d = pitch_output - last_pitch_output;
    cc_sum_d = safe_sqrt(cc_roll_d * cc_roll_d + cc_pitch_d * cc_pitch_d);

    // do the magic.
    cc_angle = cc_kd * cc_sum_d * cc_total_output - cc_total_output * motors.get_phase_angle();

    // smooth angle variations, apply constraints
    cc_angle = rate_dynamics_filter.apply(cc_angle);
    cc_angle = constrain_float(cc_angle, -90.0f, 0.0f);
    cc_angle = radians(cc_angle);

    // Make swash rate vector
    Vector2f swashratevector;
    swashratevector.x = cosf(cc_angle);
    swashratevector.y = sinf(cc_angle);
    swashratevector.normalize();

    // rotate the output
    cc_roll_output  = roll_output;
    cc_pitch_output = pitch_output;
    roll_output     = - (cc_pitch_output * swashratevector.y - cc_roll_output * swashratevector.x);
    pitch_output    =    cc_pitch_output * swashratevector.x + cc_roll_output * swashratevector.y;

    // make current outputs old, for next iteration
    last_roll_output  = cc_roll_output;
    last_pitch_output = cc_pitch_output;
# endif // HELI_CC_COMP   
    
#if HELI_PIRO_COMP == ENABLED
    if (control_mode <= ACRO){
    
        int32_t         piro_roll_i, piro_pitch_i;            // used to hold i term while doing prio comp
    
        piro_roll_i  = roll_i;
        piro_pitch_i = pitch_i;

        Vector2f yawratevector;
        yawratevector.x     = cos(-omega.z/100);
        yawratevector.y     = sin(-omega.z/100);
        yawratevector.normalize();
            
        roll_i      = piro_roll_i * yawratevector.x - piro_pitch_i * yawratevector.y;
        pitch_i     = piro_pitch_i * yawratevector.x + piro_roll_i * yawratevector.y;

        g.pid_rate_pitch.set_integrator(pitch_i);
        g.pid_rate_roll.set_integrator(roll_i); 
    }
#endif //HELI_PIRO_COMP   

    if (labs(roll_output) > 4500){
        roll_output = constrain_int32(roll_output, -4500, 4500);         // constrain output
        roll_pid_saturated = true;                                       // freeze integrator next cycle
    } else {
        roll_pid_saturated = false;                                      // unfreeze integrator
    }
    
    if (labs(pitch_output) > 4500){
        pitch_output = constrain_int32(pitch_output, -4500, 4500);        // constrain output
        pitch_pid_saturated = true;                                       // freeze integrator next cycle
    } else {
        pitch_pid_saturated = false;                                      // unfreeze integrator
    }

    g.rc_1.servo_out = roll_output;
    g.rc_2.servo_out = pitch_output;
}

static int16_t
get_heli_rate_yaw(int32_t target_rate)
{
    int32_t         p,i,d,ff;               // used to capture pid values for logging
    int32_t         current_rate;           // this iteration's rate
    int32_t         rate_error;
    int32_t         output;
    static bool     pid_saturated;          // tracker from last loop if the PID was saturated

    current_rate = (omega.z * DEGX100);                         // get current rate

    // rate control
    rate_error = target_rate - current_rate;

    // separately calculate p, i, d values for logging
    p = g.pid_rate_yaw.get_p(rate_error);

    if (pid_saturated){
        i = g.pid_rate_yaw.get_integrator();                    // Locked Integrator due to PID saturation on previous cycle
    } else {
        if (motors.motor_runup_complete()){
            i = g.pid_rate_yaw.get_i(rate_error, G_Dt);
        } else {
            i = g.pid_rate_yaw.get_leaky_i(rate_error, G_Dt, RATE_INTEGRATOR_LEAK_RATE);	// If motor is not running use leaky I-term to avoid excessive build-up
        }
    }

    d = g.pid_rate_yaw.get_d(rate_error, G_Dt);

    ff = g.heli_yaw_ff*target_rate;

    output = p + i + d + ff;

    if (labs(output) > 4500){
        output = constrain_int32(output, -4500, 4500);          // constrain output
        pid_saturated = true;                                   // freeze integrator next cycle
    } else {
        pid_saturated = false;                                  // unfreeze integrator
    }

	return output;                                              // output control
}

// heli_update_landing_swash - sets swash plate flag so higher minimum is used when landed or landing
// should be called soon after update_land_detector in main code
static void heli_update_landing_swash()
{
    switch(throttle_mode) {
        case THROTTLE_MANUAL:
        case THROTTLE_MANUAL_TILT_COMPENSATED:
        case THROTTLE_MANUAL_HELI:
            // manual modes always uses full swash range
            motors.set_collective_for_landing(false);
            break;

        case THROTTLE_LAND:
            // landing always uses limit swash range
            motors.set_collective_for_landing(true);
            break;

        case THROTTLE_HOLD:
        case THROTTLE_AUTO:
        default:
            // auto and hold use limited swash when landed
            motors.set_collective_for_landing(!heli_flags.dynamic_flight || ap.land_complete || !ap.auto_armed);
            break;
    }
}

// heli_update_rotor_speed_targets - reads pilot input and passes new rotor speed targets to heli motors object
static void heli_update_rotor_speed_targets()
{
    // get rotor control method
    uint8_t rsc_control_mode = motors.get_rsc_mode();

    switch (rsc_control_mode) {
        case AP_MOTORS_HELI_RSC_MODE_NONE:
            // even though pilot passes rotors speed directly to rotor ESC via receiver, motor lib needs to know if
            // rotor is spinning in case we are using direct drive tailrotor which must be spun up at same time
        case AP_MOTORS_HELI_RSC_MODE_CH8_PASSTHROUGH:
            // pass through pilot desired rotor speed
            motors.set_desired_rotor_speed(g.rc_8.control_in);
            break;
        case AP_MOTORS_HELI_RSC_MODE_SETPOINT:
            // pass setpoint through as desired rotor speed
            if (g.rc_8.control_in > 0) {
                motors.set_desired_rotor_speed(motors.get_rsc_setpoint());
            }else{
                motors.set_desired_rotor_speed(0);
            }
            break;
    }
}

#endif  // FRAME_CONFIG == HELI_FRAME
#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/inertia.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// read_inertia - read inertia in from accelerometers
static void read_inertia()
{
    // inertial altitude estimates
    inertial_nav.update(G_Dt);
}

// read_inertial_altitude - pull altitude and climb rate from inertial nav library
static void read_inertial_altitude()
{
    // with inertial nav we can update the altitude and climb rate at 50hz
    current_loc.alt = inertial_nav.get_altitude();
    climb_rate = inertial_nav.get_velocity_z();
}
#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/leds.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// updates the status of notify
// should be called at 50hz
static void update_notify()
{
    notify.update();
}

#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/motors.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define AUTO_TRIM_DELAY         100 // called at 10hz so 10 seconds
#define AUTO_DISARMING_DELAY    15  // called at 1hz so 15 seconds

// arm_motors_check - checks for pilot input to arm or disarm the copter
// called at 10hz
static void arm_motors_check()
{
    static int16_t arming_counter;
    bool allow_arming = false;

    // ensure throttle is down
    if (g.rc_3.control_in > 0) {
        arming_counter = 0;
        return;
    }

    // allow arming/disarming in fully manual flight modes ACRO, STABILIZE, SPORT and DRIFT
    if (manual_flight_mode(control_mode)) {
        allow_arming = true;
    }

    // allow arming/disarming in Loiter and AltHold if landed
    if (ap.land_complete && (control_mode == LOITER || control_mode == ALT_HOLD)) {
        allow_arming = true;
    }

    // kick out other flight modes
    if (!allow_arming) {
        arming_counter = 0;
        return;
    }

    #if FRAME_CONFIG == HELI_FRAME
    // heli specific arming check
    if (!motors.allow_arming()){
        arming_counter = 0;
        return;
    }
    #endif  // HELI_FRAME

    int16_t tmp = g.rc_4.control_in;

    // full right
    if (tmp > 4000) {

        // increase the arming counter to a maximum of 1 beyond the auto trim counter
        if( arming_counter <= AUTO_TRIM_DELAY ) {
            arming_counter++;
        }

        // arm the motors and configure for flight
        if (arming_counter == ARM_DELAY && !motors.armed()) {
            // run pre-arm-checks and display failures
            pre_arm_checks(true);
            if(ap.pre_arm_check && arm_checks(true)) {
                init_arm_motors();
            }else{
                // reset arming counter if pre-arm checks fail
                arming_counter = 0;
            }
        }

        // arm the motors and configure for flight
        if (arming_counter == AUTO_TRIM_DELAY && motors.armed() && control_mode == STABILIZE) {
            auto_trim_counter = 250;
        }

    // full left
    }else if (tmp < -4000) {

        // increase the counter to a maximum of 1 beyond the disarm delay
        if( arming_counter <= DISARM_DELAY ) {
            arming_counter++;
        }

        // disarm the motors
        if (arming_counter == DISARM_DELAY && motors.armed()) {
            init_disarm_motors();
        }

    // Yaw is centered so reset arming counter
    }else{
        arming_counter = 0;
    }
}

// auto_disarm_check - disarms the copter if it has been sitting on the ground in manual mode with throttle low for at least 15 seconds
// called at 1hz
static void auto_disarm_check()
{
    static uint8_t auto_disarming_counter;

    // exit immediately if we are already disarmed or throttle is not zero
    if (!motors.armed() || g.rc_3.control_in > 0) {
        auto_disarming_counter = 0;
        return;
    }

    // allow auto disarm in manual flight modes or Loiter/AltHold if we're landed
    if(manual_flight_mode(control_mode) || (ap.land_complete && (control_mode == LOITER || control_mode == ALT_HOLD))) {
        auto_disarming_counter++;

        if(auto_disarming_counter >= AUTO_DISARMING_DELAY) {
            init_disarm_motors();
            auto_disarming_counter = 0;
        }
    }else{
        auto_disarming_counter = 0;
    }
}

// init_arm_motors - performs arming process including initialisation of barometer and gyros
static void init_arm_motors()
{
	// arming marker
    // Flag used to track if we have armed the motors the first time.
    // This is used to decide if we should run the ground_start routine
    // which calibrates the IMU
    static bool did_ground_start = false;

    // disable cpu failsafe because initialising everything takes a while
    failsafe_disable();

    // disable inertial nav errors temporarily
    inertial_nav.ignore_next_error();

#if LOGGING_ENABLED == ENABLED
    // start dataflash
    start_logging();
#endif

#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    gcs_send_text_P(SEVERITY_HIGH, PSTR("ARMING MOTORS"));
#endif

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we arm
    // the motors
    hal.uartA->set_blocking_writes(false);
    hal.uartC->set_blocking_writes(false);
    if (hal.uartD != NULL) {
        hal.uartD->set_blocking_writes(false);
    }

    // Remember Orientation
    // --------------------
    init_simple_bearing();

    initial_armed_bearing = ahrs.yaw_sensor;

    // Reset home position
    // -------------------
    if (ap.home_is_set) {
        init_home();
        calc_distance_and_bearing();
    }

    // all I terms are invalid
    // -----------------------
    reset_I_all();

    if(did_ground_start == false) {
        did_ground_start = true;
        startup_ground();
    }

#if HIL_MODE != HIL_MODE_ATTITUDE
    // fast baro calibration to reset ground pressure
    init_barometer(false);
#endif

    // go back to normal AHRS gains
    ahrs.set_fast_gains(false);

    // enable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(true);

    // set hover throttle
    motors.set_mid_throttle(g.throttle_mid);

    // Cancel arming if throttle is raised too high so that copter does not suddenly take off
    read_radio();
    if (g.rc_3.control_in > g.throttle_cruise && g.throttle_cruise > 100) {
        motors.output_min();
        failsafe_enable();
        return;
    }

#if SPRAYER == ENABLED
    // turn off sprayer's test if on
    sprayer.test_pump(false);
#endif

    // enable output to motors
    output_min();

    // finally actually arm the motors
    motors.armed(true);

    // log arming to dataflash
    Log_Write_Event(DATA_ARMED);

    // reenable failsafe
    failsafe_enable();
}

// perform pre-arm checks and set ap.pre_arm_check flag
static void pre_arm_checks(bool display_failure)
{
    // exit immediately if we've already successfully performed the pre-arm check
    if (ap.pre_arm_check) {
        return;
    }

    // succeed if pre arm checks are disabled
    if(g.arming_check == ARMING_CHECK_NONE) {
        set_pre_arm_check(true);
        set_pre_arm_rc_check(true);
        return;
    }

    // pre-arm rc checks a prerequisite
    pre_arm_rc_checks();
    if(!ap.pre_arm_rc_check) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: RC not calibrated"));
        }
        return;
    }

    // check Baro
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_BARO)) {
        // barometer health check
        if(!barometer.healthy) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Baro not healthy"));
            }
            return;
        }
        // check Baro & inav alt are within 1m
        if(fabs(inertial_nav.get_altitude() - baro_alt) > 100) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Alt disparity"));
            }
            return;
        }
    }

    // check Compass
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_COMPASS)) {
        // check the compass is healthy
        if(!compass.healthy()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass not healthy"));
            }
            return;
        }

        // check compass learning is on or offsets have been set
        Vector3f offsets = compass.get_offsets();
        if(!compass._learn && offsets.length() == 0) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass not calibrated"));
            }
            return;
        }

        // check for unreasonable compass offsets
        if(offsets.length() > 500) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass offsets too high"));
            }
            return;
        }

        // check for unreasonable mag field length
        float mag_field = compass.get_field().length();
        if (mag_field > COMPASS_MAGFIELD_EXPECTED*1.65 || mag_field < COMPASS_MAGFIELD_EXPECTED*0.35) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check mag field"));
            }
            return;
        }
    }

    // check GPS
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_GPS)) {
        // check gps is ok if required - note this same check is repeated again in arm_checks
        if ((mode_requires_GPS(control_mode) || g.failsafe_gps_enabled == FS_GPS_LAND_EVEN_STABILIZE) && !pre_arm_gps_checks(display_failure)) {
            return;
        }

#if AC_FENCE == ENABLED
        // check fence is initialised
        if(!fence.pre_arm_check() || (((fence.get_enabled_fences() & AC_FENCE_TYPE_CIRCLE) != 0) && !pre_arm_gps_checks(display_failure))) {
            return;
        }
#endif
    }

    // check INS
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_INS)) {
        // check accelerometers have been calibrated
        if(!ins.calibrated()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: INS not calibrated"));
            }
            return;
        }

        // check accels and gyros are healthy
        if(!ins.healthy()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: INS not healthy"));
            }
            return;
        }
    }

#ifndef CONFIG_ARCH_BOARD_PX4FMU_V1
    // check board voltage
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_VOLTAGE)) {
        if(board_voltage() < BOARD_VOLTAGE_MIN || board_voltage() > BOARD_VOLTAGE_MAX) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check Board Voltage"));
            }
            return;
        }
    }
#endif

    // check various parameter values
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_PARAMETERS)) {

        // ensure ch7 and ch8 have different functions
        if ((g.ch7_option != 0 || g.ch8_option != 0) && g.ch7_option == g.ch8_option) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Ch7&Ch8 Opt cannot be same"));
            }
            return;
        }

        // failsafe parameter checks
        if (g.failsafe_throttle) {
            // check throttle min is above throttle failsafe trigger and that the trigger is above ppm encoder's loss-of-signal value of 900
            if (g.rc_3.radio_min <= g.failsafe_throttle_value+10 || g.failsafe_throttle_value < 910) {
                if (display_failure) {
                    gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check FS_THR_VALUE"));
                }
                return;
            }
        }

        // lean angle parameter check
        if (g.angle_max < 1000 || g.angle_max > 8000) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check ANGLE_MAX"));
            }
            return;
        }

        // acro balance parameter check
        if ((g.acro_balance_roll > g.pi_stabilize_roll.kP()) || (g.acro_balance_pitch > g.pi_stabilize_pitch.kP())) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: ACRO_BAL_ROLL/PITCH"));
            }
            return;
        }
    }

    // if we've gotten this far then pre arm checks have completed
    set_pre_arm_check(true);
}

// perform pre_arm_rc_checks checks and set ap.pre_arm_rc_check flag
static void pre_arm_rc_checks()
{
    // exit immediately if we've already successfully performed the pre-arm rc check
    if( ap.pre_arm_rc_check ) {
        return;
    }

    // set rc-checks to success if RC checks are disabled
    if ((g.arming_check != ARMING_CHECK_ALL) && !(g.arming_check & ARMING_CHECK_RC)) {
        set_pre_arm_rc_check(true);
        return;
    }

    // check if radio has been calibrated
    if(!g.rc_3.radio_min.load() && !g.rc_3.radio_max.load()) {
        return;
    }

    // check channels 1 & 2 have min <= 1300 and max >= 1700
    if (g.rc_1.radio_min > 1300 || g.rc_1.radio_max < 1700 || g.rc_2.radio_min > 1300 || g.rc_2.radio_max < 1700) {
        return;
    }

    // check channels 3 & 4 have min <= 1300 and max >= 1700
    if (g.rc_3.radio_min > 1300 || g.rc_3.radio_max < 1700 || g.rc_4.radio_min > 1300 || g.rc_4.radio_max < 1700) {
        return;
    }

    // if we've gotten this far rc is ok
    set_pre_arm_rc_check(true);
}

// performs pre_arm gps related checks and returns true if passed
static bool pre_arm_gps_checks(bool display_failure)
{
    float speed_cms = inertial_nav.get_velocity().length();     // speed according to inertial nav in cm/s

    // ensure GPS is ok and our speed is below 50cm/s
    if (!GPS_ok() || gps_glitch.glitching() || speed_cms == 0 || speed_cms > PREARM_MAX_VELOCITY_CMS) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Bad GPS Pos"));
        }
        return false;
    }

    // warn about hdop separately - to prevent user confusion with no gps lock
    if (g_gps->hdop > g.gps_hdop_good) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: High GPS HDOP"));
        }
        return false;
    }

    // if we got here all must be ok
    return true;
}

// arm_checks - perform final checks before arming
// always called just before arming.  Return true if ok to arm
static bool arm_checks(bool display_failure)
{
    // succeed if arming checks are disabled
    if (g.arming_check == ARMING_CHECK_NONE) {
        return true;
    }

    // check Baro & inav alt are within 1m
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_BARO)) {
        if(fabs(inertial_nav.get_altitude() - baro_alt) > 100) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Alt disparity"));
            }
            return false;
        }
    }

    // check gps is ok if required - note this same check is also done in pre-arm checks
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_GPS)) {
        if ((mode_requires_GPS(control_mode) || g.failsafe_gps_enabled == FS_GPS_LAND_EVEN_STABILIZE) && !pre_arm_gps_checks(display_failure)) {
            return false;
        }
    }

    // check parameters
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_PARAMETERS)) {
        // check throttle is above failsafe throttle
        if (g.failsafe_throttle != FS_THR_DISABLED && g.rc_3.radio_in < g.failsafe_throttle_value) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Thr below FS"));
            }
            return false;
        }
    }

    // check lean angle
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_INS)) {
        if (labs(ahrs.roll_sensor) > g.angle_max || labs(ahrs.pitch_sensor) > g.angle_max) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Leaning"));
            }
            return false;
        }
    }

    // check if safety switch has been pushed
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Safety Switch"));
        }
        return false;
    }

    // if we've gotten this far all is ok
    return true;
}

// init_disarm_motors - disarm motors
static void init_disarm_motors()
{
    // return immediately if we are already disarmed
    if (!motors.armed()) {
        return;
    }

#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    gcs_send_text_P(SEVERITY_HIGH, PSTR("DISARMING MOTORS"));
#endif

    motors.armed(false);

    // disable inertial nav errors temporarily
    inertial_nav.ignore_next_error();

    compass.save_offsets();

    g.throttle_cruise.save();

#if AUTOTUNE == ENABLED
    // save auto tuned parameters
    auto_tune_save_tuning_gains_and_reset();
#endif

    // we are not in the air
    set_takeoff_complete(false);
    
    // setup fast AHRS gains to get right attitude
    ahrs.set_fast_gains(true);

    // log disarm to the dataflash
    Log_Write_Event(DATA_DISARMED);

    // suspend logging
    DataFlash.EnableWrites(false);

    // disable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(false);
}

/*****************************************
* Set the flight control servos based on the current calculated values
*****************************************/
static void
set_servos_4()
{
#if FRAME_CONFIG == TRI_FRAME
    // To-Do: implement improved stability patch for tri so that we do not need to limit throttle input to motors
    g.rc_3.servo_out = min(g.rc_3.servo_out, 800);
#endif
    motors.output();
}

// servo_write - writes to a servo after checking the channel is not used for a motor
static void servo_write(uint8_t ch, uint16_t pwm)
{
    bool servo_ok = false;

    #if (FRAME_CONFIG == QUAD_FRAME)
        // Quads can use RC5 and higher as servos
        if (ch >= CH_5) servo_ok = true;
    #elif (FRAME_CONFIG == TRI_FRAME || FRAME_CONFIG == SINGLE_FRAME)
        // Tri's and Singles can use RC5, RC6, RC8 and higher
        if (ch == CH_5 || ch == CH_6 || ch >= CH_8) servo_ok = true;
    #elif (FRAME_CONFIG == HEXA_FRAME || FRAME_CONFIG == Y6_FRAME)
        // Hexa and Y6 can use RC7 and higher
        if (ch >= CH_7) servo_ok = true;
    #elif (FRAME_CONFIG == OCTA_FRAME || FRAME_CONFIG == OCTA_QUAD_FRAME)
        // Octa and X8 can use RC9 and higher
        if (ch >= CH_9) servo_ok = true;
    #elif (FRAME_CONFIG == HELI_FRAME)
        // Heli's can use RC5, RC6, RC7, not RC8, and higher
        if (ch == CH_5 || ch == CH_6 || ch == CH_7 || ch >= CH_9) servo_ok = true;
    #else
        // throw compile error if frame type is unrecognise
        #error Unrecognised frame type
    #endif

    if (servo_ok) {
        hal.rcout->enable_ch(ch);
        hal.rcout->write(ch, pwm);
    }

}
#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/navigation.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// run_nav_updates - top level call for the autopilot
// ensures calculations such as "distance to waypoint" are calculated before autopilot makes decisions
// To-Do - rename and move this function to make it's purpose more clear
static void run_nav_updates(void)
{
    // fetch position from inertial navigation
    calc_position();

    // calculate distance and bearing for reporting and autopilot decisions
    calc_distance_and_bearing();

    // run autopilot to make high level decisions about control modes
    run_autopilot();
}

// calc_position - get lat and lon positions from inertial nav library
static void calc_position(){
    if( inertial_nav.position_ok() ) {
        // pull position from interial nav library
        current_loc.lng = inertial_nav.get_longitude();
        current_loc.lat = inertial_nav.get_latitude();
    }
}

// calc_distance_and_bearing - calculate distance and direction to waypoints for reporting and autopilot decisions
static void calc_distance_and_bearing()
{
    Vector3f curr = inertial_nav.get_position();

    // get target from loiter or wpinav controller
    if( nav_mode == NAV_LOITER || nav_mode == NAV_CIRCLE ) {
        wp_distance = wp_nav.get_distance_to_target();
        wp_bearing = wp_nav.get_bearing_to_target();
    }else if( nav_mode == NAV_WP ) {
        wp_distance = wp_nav.get_distance_to_destination();
        wp_bearing = wp_nav.get_bearing_to_destination();
    }else{
        wp_distance = 0;
        wp_bearing = 0;
    }

    // calculate home distance and bearing
    if(GPS_ok()) {
        home_distance = pythagorous2(curr.x, curr.y);
        home_bearing = pv_get_bearing_cd(curr,Vector3f(0,0,0));

        // update super simple bearing (if required) because it relies on home_bearing
        update_super_simple_bearing(false);
    }
}

// run_autopilot - highest level call to process mission commands
static void run_autopilot()
{
    switch( control_mode ) {
        case AUTO:
            // load the next command if the command queues are empty
            update_commands();

            // process the active navigation and conditional commands
            verify_commands();
            break;
        case GUIDED:
            // no need to do anything - wp_nav should take care of getting us to the desired location
            break;
        case RTL:
            verify_RTL();
            break;
    }
}

// set_nav_mode - update nav mode and initialise any variables as required
static bool set_nav_mode(uint8_t new_nav_mode)
{
    bool nav_initialised = false;       // boolean to ensure proper initialisation of nav modes
    Vector3f stopping_point;            // stopping point for circle mode

    // return immediately if no change
    if( new_nav_mode == nav_mode ) {
        return true;
    }

    switch( new_nav_mode ) {

        case NAV_NONE:
            nav_initialised = true;
            // initialise global navigation variables including wp_distance
            reset_nav_params();
            break;

        case NAV_CIRCLE:
            // set center of circle to current position
            wp_nav.get_stopping_point(inertial_nav.get_position(),inertial_nav.get_velocity(),stopping_point);
            circle_set_center(stopping_point,ahrs.yaw);
            nav_initialised = true;
            break;

        case NAV_LOITER:
            // set target to current position
            wp_nav.init_loiter_target(inertial_nav.get_position(), inertial_nav.get_velocity());
            nav_initialised = true;
            break;

        case NAV_WP:
            nav_initialised = true;
            break;
    }

    // if initialisation has been successful update the yaw mode
    if( nav_initialised ) {
        nav_mode = new_nav_mode;
    }

    // return success or failure
    return nav_initialised;
}

// update_nav_mode - run navigation controller based on nav_mode
// called at 100hz
static void update_nav_mode()
{
    static uint8_t log_counter;     // used to slow NTUN logging

    // exit immediately if not auto_armed or inertial nav position bad
    if (!ap.auto_armed || !inertial_nav.position_ok()) {
        return;
    }

    switch( nav_mode ) {

        case NAV_NONE:
            // do nothing
            break;

        case NAV_CIRCLE:
            // call circle controller which in turn calls loiter controller
            update_circle();
            break;

        case NAV_LOITER:
            // reset target if we are still on the ground
            if (ap.land_complete) {
                wp_nav.init_loiter_target(inertial_nav.get_position(),inertial_nav.get_velocity());
            }else{
                // call loiter controller
                wp_nav.update_loiter();
            }
            break;

        case NAV_WP:
            // call waypoint controller
            wp_nav.update_wpnav();
            break;
    }

    // log to dataflash at 10hz
    log_counter++;
    if (log_counter >= 10 && (g.log_bitmask & MASK_LOG_NTUN) && nav_mode != NAV_NONE) {
        log_counter = 0;
        Log_Write_Nav_Tuning();
    }
}

// Keeps old data out of our calculation / logs
static void reset_nav_params(void)
{
    // Will be set by new command
    wp_bearing                      = 0;

    // Will be set by new command
    wp_distance                     = 0;

    // Will be set by nav or loiter controllers
    lon_error                       = 0;
    lat_error                       = 0;
}

// get_yaw_slew - reduces rate of change of yaw to a maximum
// assumes it is called at 100hz so centi-degrees and update rate cancel each other out
static int32_t get_yaw_slew(int32_t current_yaw, int32_t desired_yaw, int16_t deg_per_sec)
{
    return wrap_360_cd(current_yaw + constrain_int16(wrap_180_cd(desired_yaw - current_yaw), -deg_per_sec, deg_per_sec));
}


//////////////////////////////////////////////////////////
// circle navigation controller
//////////////////////////////////////////////////////////

// circle_set_center -- set circle controller's center position and starting angle
static void
circle_set_center(const Vector3f current_position, float heading_in_radians)
{
    float max_velocity;
    float cir_radius = g.circle_radius * 100;

    // set circle center to circle_radius ahead of current position
    circle_center.x = current_position.x + cir_radius * cos_yaw;
    circle_center.y = current_position.y + cir_radius * sin_yaw;

    // if we are doing a panorama set the circle_angle to the current heading
    if( g.circle_radius <= 0 ) {
        circle_angle = heading_in_radians;
        circle_angular_velocity_max = ToRad(g.circle_rate);
        circle_angular_acceleration = circle_angular_velocity_max;  // reach maximum yaw velocity in 1 second
    }else{
        // set starting angle to current heading - 180 degrees
        circle_angle = wrap_PI(heading_in_radians-PI);

        // calculate max velocity based on waypoint speed ensuring we do not use more than half our max acceleration for accelerating towards the center of the circle
        max_velocity = min(wp_nav.get_horizontal_velocity(), safe_sqrt(0.5f*wp_nav.get_waypoint_acceleration()*g.circle_radius*100.0f));

        // angular_velocity in radians per second
        circle_angular_velocity_max = max_velocity/((float)g.circle_radius * 100.0f);
        circle_angular_velocity_max = constrain_float(ToRad(g.circle_rate),-circle_angular_velocity_max,circle_angular_velocity_max);

        // angular_velocity in radians per second
        circle_angular_acceleration = wp_nav.get_waypoint_acceleration()/((float)g.circle_radius * 100);
        if (g.circle_rate < 0.0f) {
            circle_angular_acceleration = -circle_angular_acceleration;
        }
    }

    // initialise other variables
    circle_angle_total = 0;
    circle_angular_velocity = 0;

    // initialise loiter target.  Note: feed forward velocity set to zero
    wp_nav.init_loiter_target(current_position, Vector3f(0,0,0));
}

// update_circle - circle position controller's main call which in turn calls loiter controller with updated target position
static void
update_circle()
{
    static float last_update;    // time of last circle call

    // calculate dt
    uint32_t now = hal.scheduler->millis();
    float dt = (now - last_update) / 1000.0f;

    // ensure enough time has passed since the last iteration
    if (dt >= 0.095f) {
        float cir_radius = g.circle_radius * 100;
        Vector3f circle_target;

        // range check dt
        if (dt >= 1.0f) {
            dt = 0;
        }

        // update time of circle call
        last_update = now;

        // ramp up angular velocity to maximum
        if (g.circle_rate >= 0) {
            if (circle_angular_velocity < circle_angular_velocity_max) {
                circle_angular_velocity += circle_angular_acceleration * dt;
                circle_angular_velocity = constrain_float(circle_angular_velocity, 0, circle_angular_velocity_max);
            }
        }else{
            if (circle_angular_velocity > circle_angular_velocity_max) {
                circle_angular_velocity += circle_angular_acceleration * dt;
                circle_angular_velocity = constrain_float(circle_angular_velocity, circle_angular_velocity_max, 0);
            }
        }

        // update the target angle
        circle_angle += circle_angular_velocity * dt;
        circle_angle = wrap_PI(circle_angle);

        // update the total angle travelled
        circle_angle_total += circle_angular_velocity * dt;

        // if the circle_radius is zero we are doing panorama so no need to update loiter target
        if( g.circle_radius != 0.0 ) {
            // calculate target position
            circle_target.x = circle_center.x + cir_radius * cosf(-circle_angle);
            circle_target.y = circle_center.y - cir_radius * sinf(-circle_angle);
            circle_target.z = wp_nav.get_desired_alt();

            // re-use loiter position controller
            wp_nav.set_loiter_target(circle_target);
        }
    }

    // call loiter controller
    wp_nav.update_loiter();
}
#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/perf_info.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
//  high level performance monitoring
//
//  we measure the main loop time
//

#define PERF_INFO_OVERTIME_THRESHOLD_MICROS 10500

uint16_t perf_info_loop_count;
uint16_t perf_info_long_running;

// perf_info_reset - reset all records of loop time to zero
void perf_info_reset()
{
    perf_info_loop_count = 0;
    perf_info_max_time = 0;
    perf_info_long_running = 0;
}

// perf_info_check_loop_time - check latest loop time vs min, max and overtime threshold
void perf_info_check_loop_time(uint32_t time_in_micros)
{
    perf_info_loop_count++;
    if( time_in_micros > perf_info_max_time) {
        perf_info_max_time = time_in_micros;
    }
    if( time_in_micros > PERF_INFO_OVERTIME_THRESHOLD_MICROS ) {
        perf_info_long_running++;
    }
}

// perf_info_get_long_running_percentage - get number of long running loops as a percentage of the total number of loops
uint16_t perf_info_get_num_loops()
{
    return perf_info_loop_count;
}

// perf_info_get_max_time - return maximum loop time (in microseconds)
uint32_t perf_info_get_max_time()
{
    return perf_info_max_time;
}

// perf_info_get_num_long_running - get number of long running loops
uint16_t perf_info_get_num_long_running()
{
    return perf_info_long_running;
}
#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/plane_navigation.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


// set the nav_controller pointer to the right controller
static void set_nav_controller(void)
{
    switch ((AP_Navigation::ControllerType)g.nav_controller.get()) {
    case AP_Navigation::CONTROLLER_L1:
        nav_controller = &L1_controller;
        break;
    }
}

/*
  reset the total loiter angle
 */
static void loiter_angle_reset(void)
{
    loiter.sum_cd = 0;
    loiter.total_cd = 0;
}

/*
  update the total angle we have covered in a loiter. Used to support
  commands to do N circles of loiter
 */
static void loiter_angle_update(void)
{
    int32_t target_bearing_cd = nav_controller->target_bearing_cd();
    int32_t loiter_delta_cd;
    if (loiter.sum_cd == 0) {
        // use 1 cd for initial delta
        loiter_delta_cd = 1;
    } else {
        loiter_delta_cd = target_bearing_cd - loiter.old_target_bearing_cd;
    }
    loiter.old_target_bearing_cd = target_bearing_cd;
    loiter_delta_cd = wrap_180_cd(loiter_delta_cd);

    loiter.sum_cd += loiter_delta_cd;
}

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
static void navigate()
{
    // allow change of nav controller mid-flight
    set_nav_controller();

    // do not navigate with corrupt data
    // ---------------------------------
    if (!have_position) {
        return;
    }

    if (next_WP.lat == 0) {
        return;
    }

    // waypoint distance from plane
    // ----------------------------
    plane_wp_distance = get_distance(current_loc, next_WP);

    if (plane_wp_distance < 0) {
        gcs_send_text_P(SEVERITY_HIGH,PSTR("WP error - distance < 0"));
        return;
    }

    // update total loiter angle
    loiter_angle_update();

    // control mode specific updates to navigation demands
    // ---------------------------------------------------
    update_navigation();
}


static void calc_airspeed_errors()
{
    float aspeed_cm = airspeed.get_airspeed_cm();

    // Normal airspeed target
    target_airspeed_cm = g.airspeed_cruise_cm;

    // FBW_B airspeed target
    if (plane_control_mode == PLANE_FLY_BY_WIRE_B || 
        plane_control_mode == PLANE_CRUISE) {
        target_airspeed_cm = ((int32_t)(aparm.airspeed_max -
                                        aparm.airspeed_min) *
                              channel_throttle->control_in) +
                             ((int32_t)aparm.airspeed_min * 100);
    }

    // Set target to current airspeed + ground speed undershoot,
    // but only when this is faster than the target airspeed commanded
    // above.
    if (plane_control_mode >= PLANE_FLY_BY_WIRE_B && (g.min_gndspeed_cm > 0)) {
        int32_t min_gnd_target_airspeed = aspeed_cm + groundspeed_undershoot;
        if (min_gnd_target_airspeed > target_airspeed_cm)
            target_airspeed_cm = min_gnd_target_airspeed;
    }

    // Bump up the target airspeed based on throttle nudging
    if (plane_control_mode >= PLANE_AUTO && airspeed_nudge_cm > 0) {
        target_airspeed_cm += airspeed_nudge_cm;
    }

    // Apply airspeed limit
    if (target_airspeed_cm > (aparm.airspeed_max * 100))
        target_airspeed_cm = (aparm.airspeed_max * 100);

    airspeed_error_cm = target_airspeed_cm - aspeed_cm;
}

static void calc_gndspeed_undershoot()
{
 	// Use the component of ground speed in the forward direction
	// This prevents flyaway if wind takes plane backwards
    if (g_gps && g_gps->status() >= GPS::GPS_OK_FIX_2D) {
	    Vector2f gndVel = ahrs.groundspeed_vector();
		const Matrix3f &rotMat = ahrs.get_dcm_matrix();
		Vector2f yawVect = Vector2f(rotMat.a.x,rotMat.b.x);
		yawVect.normalize();
		float gndSpdFwd = yawVect * gndVel;
        groundspeed_undershoot = (g.min_gndspeed_cm > 0) ? (g.min_gndspeed_cm - gndSpdFwd*100) : 0;
    }
}

static void calc_altitude_error()
{
    if (plane_control_mode == PLANE_FLY_BY_WIRE_B ||
        plane_control_mode == PLANE_CRUISE) {
        return;
    }
    if (nav_controller->reached_loiter_target()) {
        // once we reach a loiter target then lock to the final
        // altitude target
        target_altitude_cm = next_WP.alt;
    } else if (offset_altitude_cm != 0) {
        // control climb/descent rate
        target_altitude_cm = next_WP.alt - (offset_altitude_cm*((float)(plane_wp_distance-30) / (float)(wp_totalDistance-30)));

        // stay within a certain range
        if (prev_WP.alt > next_WP.alt) {
            target_altitude_cm = constrain_int32(target_altitude_cm, next_WP.alt, prev_WP.alt);
        }else{
            target_altitude_cm = constrain_int32(target_altitude_cm, prev_WP.alt, next_WP.alt);
        }
    } else if (non_nav_command_ID != MAV_CMD_CONDITION_CHANGE_ALT) {
        target_altitude_cm = next_WP.alt;
    }

    altitude_error_cm       = target_altitude_cm - adjusted_altitude_cm();
}

static void update_loiter()
{
    nav_controller->update_loiter(next_WP, abs(g.loiter_radius), loiter.direction);
}

/*
  handle PLANE_CRUISE mode, locking heading to GPS course when we have
  sufficient ground speed, and no aileron or rudder input
 */
static void update_cruise()
{
    if (!cruise_state.locked_heading &&
        channel_roll->control_in == 0 &&
        channel_rudder->control_in == 0 &&
        g_gps && g_gps->status() >= GPS::GPS_OK_FIX_2D &&
        g_gps->ground_speed_cm >= 300 &&
        cruise_state.lock_timer_ms == 0) {
        // user wants to lock the heading - start the timer
        cruise_state.lock_timer_ms = hal.scheduler->millis();
    }
    if (cruise_state.lock_timer_ms != 0 &&
        (hal.scheduler->millis() - cruise_state.lock_timer_ms) > 500) {
        // lock the heading after 0.5 seconds of zero heading input
        // from user
        cruise_state.locked_heading = true;
        cruise_state.lock_timer_ms = 0;
        cruise_state.locked_heading_cd = g_gps->ground_course_cd;
        prev_WP = current_loc;
    }
    if (cruise_state.locked_heading) {
        next_WP = prev_WP;
        // always look 1km ahead
        location_update(next_WP, 
                        cruise_state.locked_heading_cd*0.01f, 
                        get_distance(prev_WP, current_loc) + 1000);
        nav_controller->update_waypoint(prev_WP, next_WP);
    }
}


/*
  handle speed and height control in FBWB or PLANE_CRUISE mode. 
  In this mode the elevator is used to change target altitude. The
  throttle is used to change target airspeed or throttle
 */
static void update_fbwb_speed_height(void)
{
    static float last_elevator_input;
    float elevator_input;
    elevator_input = channel_pitch->control_in / 4500.0f;
    
    if (g.flybywire_elev_reverse) {
        elevator_input = -elevator_input;
    }
    
    target_altitude_cm += g.flybywire_climb_rate * elevator_input * delta_us_fast_loop * 0.0001f;
    
    if (elevator_input == 0.0f && last_elevator_input != 0.0f) {
        // the user has just released the elevator, lock in
        // the current altitude
        target_altitude_cm = current_loc.alt;
    }

    // check for FBWB altitude limit
    if (g.FBWB_min_altitude_cm != 0 && target_altitude_cm < home.alt + g.FBWB_min_altitude_cm) {
        target_altitude_cm = home.alt + g.FBWB_min_altitude_cm;
    }
    altitude_error_cm = target_altitude_cm - adjusted_altitude_cm();
    
    last_elevator_input = elevator_input;
    
    calc_throttle();
    calc_nav_pitch();
}

static void setup_glide_slope(void)
{
    // establish the distance we are travelling to the next waypoint,
    // for calculating out rate of change of altitude
    wp_totalDistance        = get_distance(current_loc, next_WP);
    plane_wp_distance             = wp_totalDistance;

    /*
      work out if we will gradually change altitude, or try to get to
      the new altitude as quickly as possible.
     */
    switch (plane_control_mode) {
    case PLANE_RTL:
    case PLANE_GUIDED:
        /* glide down slowly if above target altitude, but ascend more
           rapidly if below it. See
           https://github.com/diydrones/ardupilot/issues/39
        */
        if (current_loc.alt > next_WP.alt) {
            offset_altitude_cm = next_WP.alt - current_loc.alt;            
        } else {
            offset_altitude_cm = 0;
        }
        break;

    case PLANE_AUTO:
        if (prev_WP.id != MAV_CMD_NAV_TAKEOFF && 
            prev_WP.alt != home.alt && 
            (next_WP.id == MAV_CMD_NAV_WAYPOINT || next_WP.id == MAV_CMD_NAV_LAND)) {
            offset_altitude_cm = next_WP.alt - prev_WP.alt;
        } else {
            offset_altitude_cm = 0;        
        }
        break;
    default:
        offset_altitude_cm = 0;        
        break;
    }
}

/*
  return relative altitude in meters (relative to home)
 */
static float relative_altitude(void)
{
    return (current_loc.alt - home.alt) * 0.01f;
}

/*
  return relative altitude in centimeters, absolute value
 */
static int32_t relative_altitude_abs_cm(void)
{
    return labs(current_loc.alt - home.alt);
}

#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/position_vector.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// position_vector.pde related utility functions

// position vectors are Vector2f
//    .x = latitude from home in cm
//    .y = longitude from home in cm
//    .z = altitude above home in cm

// pv_latlon_to_vector - convert lat/lon coordinates to a position vector
Vector3f pv_latlon_to_vector(int32_t lat, int32_t lon, int32_t alt)
{
    Vector3f tmp((lat-home.lat) * LATLON_TO_CM, (lon-home.lng) * LATLON_TO_CM * scaleLongDown, alt);
    return tmp;
}

// pv_latlon_to_vector - convert lat/lon coordinates to a position vector
Vector3f pv_location_to_vector(Location loc)
{
    Vector3f tmp((loc.lat-home.lat) * LATLON_TO_CM, (loc.lng-home.lng) * LATLON_TO_CM * scaleLongDown, loc.alt);
    return tmp;
}

// pv_get_lon - extract latitude from position vector
int32_t pv_get_lat(const Vector3f pos_vec)
{
    return home.lat + (int32_t)(pos_vec.x / LATLON_TO_CM);
}

// pv_get_lon - extract longitude from position vector
int32_t pv_get_lon(const Vector3f &pos_vec)
{
    return home.lng + (int32_t)(pos_vec.y / LATLON_TO_CM * scaleLongUp);
}

// pv_get_horizontal_distance_cm - return distance between two positions in cm
float pv_get_horizontal_distance_cm(const Vector3f &origin, const Vector3f &destination)
{
    return pythagorous2(destination.x-origin.x,destination.y-origin.y);
}

// pv_get_bearing_cd - return bearing in centi-degrees between two positions
float pv_get_bearing_cd(const Vector3f &origin, const Vector3f &destination)
{
    float bearing = 9000 + atan2f(-(destination.x-origin.x), destination.y-origin.y) * DEGX100;
    if (bearing < 0) {
        bearing += 36000;
    }
    return bearing;
}
#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/radio.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

static void set_control_channels(void)
{
    channel_roll     = RC_Channel::rc_channel(rcmap.roll()-1);
	channel_pitch    = RC_Channel::rc_channel(rcmap.pitch()-1);
    channel_throttle = RC_Channel::rc_channel(rcmap.throttle()-1);
    channel_rudder   = RC_Channel::rc_channel(rcmap.yaw()-1);

    // set rc channel ranges
    channel_roll->set_angle(SERVO_MAX);
    channel_pitch->set_angle(SERVO_MAX);
    channel_rudder->set_angle(SERVO_MAX);
    channel_throttle->set_range(0, 100);

    if (!arming.is_armed() && arming.arming_required() == AP_Arming::YES_MIN_PWM) {
        hal.rcout->set_safety_pwm(1UL<<(rcmap.throttle()-1), channel_throttle->radio_min);
    }
}

static void default_dead_zones()
{
    g.rc_1.set_default_dead_zone(30);
    g.rc_2.set_default_dead_zone(30);
#if FRAME_CONFIG == HELI_FRAME
    g.rc_3.set_default_dead_zone(10);
    g.rc_4.set_default_dead_zone(15);
    g.rc_8.set_default_dead_zone(10);
#else
    g.rc_3.set_default_dead_zone(30);
    g.rc_4.set_default_dead_zone(40);
#endif
    g.rc_6.set_default_dead_zone(0);
}

static void init_rc_in()
{
    // set rc channel ranges
    g.rc_1.set_angle(ROLL_PITCH_INPUT_MAX);
    g.rc_2.set_angle(ROLL_PITCH_INPUT_MAX);
    g.rc_3.set_range(g.throttle_min, g.throttle_max);
    g.rc_4.set_angle(4500);

    g.rc_1.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    g.rc_2.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    g.rc_4.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
#if FRAME_CONFIG == SINGLE_FRAME
    // we set four servos to angle
    g.single_servo_1.set_type(RC_CHANNEL_TYPE_ANGLE);
    g.single_servo_2.set_type(RC_CHANNEL_TYPE_ANGLE);
    g.single_servo_3.set_type(RC_CHANNEL_TYPE_ANGLE);
    g.single_servo_4.set_type(RC_CHANNEL_TYPE_ANGLE);
    g.single_servo_1.set_angle(DEFAULT_ANGLE_MAX);
    g.single_servo_2.set_angle(DEFAULT_ANGLE_MAX);
    g.single_servo_3.set_angle(DEFAULT_ANGLE_MAX);
    g.single_servo_4.set_angle(DEFAULT_ANGLE_MAX);
#endif

    //set auxiliary servo ranges
    g.rc_5.set_range(0,1000);
    g.rc_6.set_range(0,1000);
    g.rc_7.set_range(0,1000);
    g.rc_8.set_range(0,1000);

    // update assigned functions for auxiliary servos
    aux_servos_update_fn();

    // set default dead zones
    default_dead_zones();
}

 // init_rc_out -- initialise motors and check if pilot wants to perform ESC calibration
static void init_rc_out()
{
    channel_roll->enable_out();
    channel_pitch->enable_out();
    if (arming.arming_required() != AP_Arming::YES_ZERO_PWM) {
        channel_throttle->enable_out();
    }
    channel_rudder->enable_out();

    // Initialization of servo outputs
    for (uint8_t i=0; i<8; i++) {
        RC_Channel::rc_channel(i)->output_trim();
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    servo_write(CH_9,   g.rc_9.radio_trim);
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2 || CONFIG_HAL_BOARD == HAL_BOARD_PX4
    servo_write(CH_10,  g.rc_10.radio_trim);
    servo_write(CH_11,  g.rc_11.radio_trim);
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    servo_write(CH_12,  g.rc_12.radio_trim);
#endif

    motors.set_update_rate(g.rc_speed);
    motors.set_frame_orientation(g.frame_orientation);
    motors.Init();                                              // motor initialisation
    motors.set_min_throttle(g.throttle_min);

    for(uint8_t i = 0; i < 5; i++) {
        delay(20);
        read_radio();
    }

    // we want the input to be scaled correctly
    g.rc_3.set_range_out(0,1000);

    // full throttle means to enter ESC calibration
    if(g.rc_3.control_in >= (g.throttle_max - 50)) {
        if(g.esc_calibrate == 0) {
            // we will enter esc_calibrate mode on next reboot
            g.esc_calibrate.set_and_save(1);
            // display message on console
            cliSerial->printf_P(PSTR("Entering ESC Calibration: please restart APM.\n"));
            // turn on esc calibration notification
            AP_Notify::flags.esc_calibration = true;
            // block until we restart
            while(1) { delay(5); }
        }else{
            cliSerial->printf_P(PSTR("ESC Calibration active: passing throttle through to ESCs.\n"));
            // clear esc flag
            g.esc_calibrate.set_and_save(0);
            // pass through user throttle to escs
            init_esc();
        }
    }else{
        // did we abort the calibration?
        if(g.esc_calibrate == 1)
            g.esc_calibrate.set_and_save(0);
    }

    // enable output to motors
    pre_arm_rc_checks();
    if (ap.pre_arm_rc_check) {
        output_min();
    }
	
    // setup PX4 to output the min throttle when safety off if arming
    // is setup for min on disarm
    if (arming.arming_required() == AP_Arming::YES_MIN_PWM) {
        hal.rcout->set_safety_pwm(1UL<<(rcmap.throttle()-1), channel_throttle->radio_min);
    }
}

// check for pilot input on rudder stick for arming
static void rudder_arm_check() 
{
    //TODO: ensure rudder arming disallowed during radio calibration

    //TODO: waggle ailerons and rudder and beep after rudder arming
    
    static uint32_t rudder_arm_timer;

    if (arming.is_armed()) {
        //already armed, no need to run remainder of this function
        rudder_arm_timer = 0;
        return;
    } 

    if (! arming.rudder_arming_enabled()) {
        //parameter disallows rudder arming
        return;
    }

    //if throttle is not down, then pilot cannot rudder arm
    if (g.rc_3.control_in > 0) {
        rudder_arm_timer = 0;
        return;
    }

    //if not in a 'manual' mode then disallow rudder arming
    if (auto_throttle_mode ) {
        rudder_arm_timer = 0;
        return;      
    }

    // full right rudder starts arming counter
    if (g.rc_4.control_in > 4000) {
        uint32_t now = millis();

        if (rudder_arm_timer == 0 || 
            now - rudder_arm_timer < 3000) {

            if (rudder_arm_timer == 0) rudder_arm_timer = now;
        } else {
            //time to arm!
            if (arming.arm(AP_Arming::RUDDER)) {
                channel_throttle->enable_out();                        
                //only log if arming was successful
#if LOGGING_ENABLED == ENABLED
                Log_Arm_Disarm();
#endif

            }                
        }
    } else { 
        // not at full right rudder
        rudder_arm_timer = 0;
    }
}

// output_min - enable and output lowest possible value to motors
void output_min()
{
    // enable motors
    motors.enable();
    motors.output_min();
}

#define FAILSAFE_RADIO_TIMEOUT_MS 2000       // 2 seconds
static void read_radio()
{
    static uint32_t last_update = 0;
    if (hal.rcin->valid_channels() > 0) {
        last_update = millis();
        ap.new_radio_frame = true;
        uint16_t periods[8];
        hal.rcin->read(periods,8);
        g.rc_1.set_pwm(periods[rcmap.roll()-1]);
        g.rc_2.set_pwm(periods[rcmap.pitch()-1]);

        set_throttle_and_failsafe(periods[rcmap.throttle()-1]);

        g.rc_4.set_pwm(periods[rcmap.yaw()-1]);
        g.rc_5.set_pwm(periods[4]);
        g.rc_6.set_pwm(periods[5]);
        g.rc_7.set_pwm(periods[6]);
        g.rc_8.set_pwm(periods[7]);

        // flag we must have an rc receiver attached
        if (!failsafe.rc_override_active) {
            ap.rc_receiver_present = true;
        }
    }else{
        uint32_t elapsed = millis() - last_update;
        // turn on throttle failsafe if no update from ppm encoder for 2 seconds
        if ((elapsed >= FAILSAFE_RADIO_TIMEOUT_MS)
                && g.failsafe_throttle && motors.armed() && !failsafe.radio) {
            Log_Write_Error(ERROR_SUBSYSTEM_RADIO, ERROR_CODE_RADIO_LATE_FRAME);
            set_failsafe_radio(true);
        }
    }
}

static void plane_read_radio()
{
    if (!hal.rcin->valid_channels()) {
        control_failsafe(channel_throttle->radio_in);
        return;
    }

    plane_failsafe.last_valid_rc_ms = hal.scheduler->millis();

    elevon.ch1_temp = channel_roll->read();
    elevon.ch2_temp = channel_pitch->read();
    uint16_t pwm_roll, pwm_pitch;

    if (g.mix_mode == 0) {
        pwm_roll = elevon.ch1_temp;
        pwm_pitch = elevon.ch2_temp;
    }else{
        pwm_roll = BOOL_TO_SIGN(g.reverse_elevons) * (BOOL_TO_SIGN(g.reverse_ch2_elevon) * int16_t(elevon.ch2_temp - elevon.trim2) - BOOL_TO_SIGN(g.reverse_ch1_elevon) * int16_t(elevon.ch1_temp - elevon.trim1)) / 2 + 1500;
        pwm_pitch = (BOOL_TO_SIGN(g.reverse_ch2_elevon) * int16_t(elevon.ch2_temp - elevon.trim2) + BOOL_TO_SIGN(g.reverse_ch1_elevon) * int16_t(elevon.ch1_temp - elevon.trim1)) / 2 + 1500;
    }
    
    if (plane_control_mode == PLANE_TRAINING) {
        // in training mode we don't want to use a deadzone, as we
        // want manual pass through when not exceeding attitude limits
        channel_roll->set_pwm_no_deadzone(pwm_roll);
        channel_pitch->set_pwm_no_deadzone(pwm_pitch);
        channel_throttle->set_pwm_no_deadzone(channel_throttle->read());
        channel_rudder->set_pwm_no_deadzone(channel_rudder->read());
    } else {
        channel_roll->set_pwm(pwm_roll);
        channel_pitch->set_pwm(pwm_pitch);
        channel_throttle->set_pwm(channel_throttle->read());
        channel_rudder->set_pwm(channel_rudder->read());
    }

    g.rc_5.set_pwm(hal.rcin->read(CH_5));
    g.rc_6.set_pwm(hal.rcin->read(CH_6));
    g.rc_7.set_pwm(hal.rcin->read(CH_7));
    g.rc_8.set_pwm(hal.rcin->read(CH_8));

    control_failsafe(channel_throttle->radio_in);

    channel_throttle->servo_out = channel_throttle->control_in;

    if (g.throttle_nudge && channel_throttle->servo_out > 50) {
        float nudge = (channel_throttle->servo_out - 50) * 0.02f;
        if (airspeed.use()) {
            airspeed_nudge_cm = (aparm.airspeed_max * 100 - g.airspeed_cruise_cm) * nudge;
        } else {
            throttle_nudge = (aparm.plthr_max - aparm.plthr_cruise) * nudge;
        }
    } else {
        airspeed_nudge_cm = 0;
        throttle_nudge = 0;
    }

    rudder_arm_check();
}

static void control_failsafe(uint16_t pwm)
{
    if(g.throttle_fs_enabled == 0)
        return;

    // Check for failsafe condition based on loss of GCS control
    if (plane_failsafe.rc_override_active) {
        if (millis() - plane_failsafe.last_heartbeat_ms > g.short_fs_timeout*1000) {
            plane_failsafe.ch3_failsafe = true;
            AP_Notify::flags.failsafe_radio = true;
        } else {
            plane_failsafe.ch3_failsafe = false;
            AP_Notify::flags.failsafe_radio = false;
        }

        //Check for failsafe and debounce funky reads
    } else if (g.throttle_fs_enabled) {
        if (throttle_failsafe_level()) {
            // we detect a failsafe from radio
            // throttle has dropped below the mark
            plane_failsafe.ch3_counter++;
            if (plane_failsafe.ch3_counter == 10) {
                plane_gcs_send_text_fmt(PSTR("MSG FS ON %u"), (unsigned)pwm);
                plane_failsafe.ch3_failsafe = true;
                AP_Notify::flags.failsafe_radio = true;
            }
            if (plane_failsafe.ch3_counter > 10) {
                plane_failsafe.ch3_counter = 10;
            }

        }else if(plane_failsafe.ch3_counter > 0) {
            // we are no longer in failsafe condition
            // but we need to recover quickly
            plane_failsafe.ch3_counter--;
            if (plane_failsafe.ch3_counter > 3) {
                plane_failsafe.ch3_counter = 3;
            }
            if (plane_failsafe.ch3_counter == 1) {
                plane_gcs_send_text_fmt(PSTR("MSG FS OFF %u"), (unsigned)pwm);
            } else if(plane_failsafe.ch3_counter == 0) {
                plane_failsafe.ch3_failsafe = false;
                AP_Notify::flags.failsafe_radio = false;
            }
        }
    }
}

#define FS_COUNTER 3        // radio failsafe kicks in after 3 consecutive throttle values below failsafe_throttle_value
static void set_throttle_and_failsafe(uint16_t throttle_pwm)
{
    // if failsafe not enabled pass through throttle and exit
    if(g.failsafe_throttle == FS_THR_DISABLED) {
        g.rc_3.set_pwm(throttle_pwm);
        return;
    }

    //check for low throttle value
    if (throttle_pwm < (uint16_t)g.failsafe_throttle_value) {

        // if we are already in failsafe or motors not armed pass through throttle and exit
        if (failsafe.radio || !motors.armed()) {
            g.rc_3.set_pwm(throttle_pwm);
            return;
        }

        // check for 3 low throttle values
        // Note: we do not pass through the low throttle until 3 low throttle values are recieved
        failsafe.radio_counter++;
        if( failsafe.radio_counter >= FS_COUNTER ) {
            failsafe.radio_counter = FS_COUNTER;  // check to ensure we don't overflow the counter
            set_failsafe_radio(true);
            g.rc_3.set_pwm(throttle_pwm);   // pass through failsafe throttle
        }
    }else{
        // we have a good throttle so reduce failsafe counter
        failsafe.radio_counter--;
        if( failsafe.radio_counter <= 0 ) {
            failsafe.radio_counter = 0;   // check to ensure we don't underflow the counter

            // disengage failsafe after three (nearly) consecutive valid throttle values
            if (failsafe.radio) {
                set_failsafe_radio(false);
            }
        }
        // pass through throttle
        g.rc_3.set_pwm(throttle_pwm);
    }
}

// aux_servos_update - update auxiliary servos assigned functions in case the user has changed them
void aux_servos_update_fn()
{
// Quads can use RC5 and higher as auxiliary channels
#if (FRAME_CONFIG == QUAD_FRAME)
 #if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8, &g.rc_9, &g.rc_10, &g.rc_11, &g.rc_12);
 #else // APM1, APM2, SITL
    update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8, &g.rc_10, &g.rc_11);
 #endif

// Tri's and Singles can use RC5, RC6, RC8 and higher
#elif (FRAME_CONFIG == TRI_FRAME || FRAME_CONFIG == SINGLE_FRAME)
 #if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_8, &g.rc_9, &g.rc_10, &g.rc_11, &g.rc_12);
 #else // APM1, APM2, SITL
    update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_8, &g.rc_10, &g.rc_11);
 #endif

// Hexa and Y6 can use RC7 and higher
#elif (FRAME_CONFIG == HEXA_FRAME || FRAME_CONFIG == Y6_FRAME)
 #if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    update_aux_servo_function(&g.rc_7, &g.rc_8, &g.rc_9, &g.rc_10, &g.rc_11, &g.rc_12);
 #else
    update_aux_servo_function(&g.rc_7, &g.rc_8, &g.rc_10, &g.rc_11);
 #endif

// Octa and X8 can use RC9 and higher
#elif (FRAME_CONFIG == OCTA_FRAME || FRAME_CONFIG == OCTA_QUAD_FRAME)
 #if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    update_aux_servo_function(&g.rc_9, &g.rc_10, &g.rc_11, &g.rc_12);
 #else
    update_aux_servo_function(&g.rc_10, &g.rc_11);
 #endif

// Heli's can use RC5, RC6, RC7, not RC8, and higher
#elif (FRAME_CONFIG == HELI_FRAME)
 #if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_9, &g.rc_10, &g.rc_11, &g.rc_12);
 #else // APM1, APM2, SITL
    update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_10, &g.rc_11);
 #endif

// throw compile error if frame type is unrecognise
#else
  #error Unrecognised frame type
#endif
}

static void trim_control_surfaces()
{
    plane_read_radio();
    int16_t trim_roll_range = (channel_roll->radio_max - channel_roll->radio_min)/5;
    int16_t trim_pitch_range = (channel_pitch->radio_max - channel_pitch->radio_min)/5;
    if (channel_roll->radio_in < channel_roll->radio_min+trim_roll_range ||
        channel_roll->radio_in > channel_roll->radio_max-trim_roll_range ||
        channel_pitch->radio_in < channel_pitch->radio_min+trim_pitch_range ||
        channel_pitch->radio_in > channel_pitch->radio_max-trim_pitch_range) {
        // don't trim for extreme values - if we attempt to trim so
        // there is less than 20 percent range left then assume the
        // sticks are not properly centered. This also prevents
        // problems with starting APM with the TX off
        return;
    }

    // Store control surface trim values
    // ---------------------------------
    if(g.mix_mode == 0) {
        if (channel_roll->radio_in != 0) {
            channel_roll->radio_trim = channel_roll->radio_in;
        }
        if (channel_pitch->radio_in != 0) {
            channel_pitch->radio_trim = channel_pitch->radio_in;
        }

        // the secondary aileron/elevator is trimmed only if it has a
        // corresponding transmitter input channel, which k_aileron
        // doesn't have
        RC_Channel_aux::set_radio_trim(RC_Channel_aux::k_aileron_with_input);
        RC_Channel_aux::set_radio_trim(RC_Channel_aux::k_elevator_with_input);
    } else{
        if (elevon.ch1_temp != 0) {
            elevon.trim1 = elevon.ch1_temp;
        }
        if (elevon.ch2_temp != 0) {
            elevon.trim2 = elevon.ch2_temp;
        }
        //Recompute values here using new values for elevon1_trim and elevon2_trim
        //We cannot use radio_in[CH_ROLL] and radio_in[CH_PITCH] values from plane_read_radio() because the elevon trim values have changed
        uint16_t center                         = 1500;
        channel_roll->radio_trim       = center;
        channel_pitch->radio_trim      = center;
    }
    if (channel_rudder->radio_in != 0) {
        channel_rudder->radio_trim = channel_rudder->radio_in;
    }

    // save to eeprom
    channel_roll->save_eeprom();
    channel_pitch->save_eeprom();
    channel_rudder->save_eeprom();
}

static void trim_radio()
{
    for (uint8_t i = 0; i < 30; i++) {
    if (!isplane){
		read_radio();
		}
	if (isplane){	
        plane_read_radio();
		}
    }

    g.rc_1.trim();      // roll
    g.rc_2.trim();      // pitch
    g.rc_4.trim();      // yaw

    g.rc_1.save_eeprom();
    g.rc_2.save_eeprom();
    g.rc_4.save_eeprom();

    if (isplane){
    trim_control_surfaces();
	}
}

static bool throttle_failsafe_level(void)
{
    if (!g.throttle_fs_enabled) {
        return false;
    }
    if (hal.scheduler->millis() - plane_failsafe.last_valid_rc_ms > 2000) {
        // we haven't had a valid RC frame for 2 seconds
        return true;
    }
    if (channel_throttle->get_reverse()) {
        return channel_throttle->radio_in >= g.throttle_fs_value;
    }
    return channel_throttle->radio_in <= g.throttle_fs_value;
}
#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/rally.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * rally point support
 * Michael Day, September 2013
 */

//get a rally point
static bool get_rally_point_with_index(unsigned i, RallyLocation &ret)
{
    if (i >= (unsigned)g.rally_total) {
        return false;
    }
    hal.storage->read_block(&ret, RALLY_START_BYTE + (i * sizeof(RallyLocation)), sizeof(RallyLocation));
    if (ret.lat == 0 && ret.lng == 0) {
        // sanity check ...
        return false;
    }
    return true; 
}

//save a rally point
static bool set_rally_point_with_index(unsigned i, const RallyLocation &rallyLoc)
{
    if (i >= (unsigned) g.rally_total) {
        //not allowed
        return false;
    }

    if (i >= MAX_RALLYPOINTS) {
        //also not allowed
        return false;
    }
    hal.storage->write_block(RALLY_START_BYTE + (i * sizeof(RallyLocation)), &rallyLoc, sizeof(RallyLocation));
    return true;
}

// 'best' means 'closest to Location loc' for now.
static bool find_best_rally_point(const Location &myloc, const Location &homeloc, RallyLocation &ret) 
{
    float min_dis = -1;

    for (unsigned i = 0; i < (unsigned) g.rally_total; i++) {
        RallyLocation next_rally;
        if (!get_rally_point_with_index(i, next_rally)) {
            continue;
        }
        Location rally_loc = rally_location_to_location(next_rally, homeloc);
        float dis = get_distance(myloc, rally_loc);

        if (dis < min_dis || min_dis < 0) {
            min_dis = dis;
            ret = next_rally;
        }
    }

    if (g.rally_limit_km > 0 && min_dis > g.rally_limit_km*1000.0f && 
        get_distance(myloc, homeloc) < min_dis) {
        // return false, which makes home be used instead
        return false;
    }

    return min_dis >= 0;
}

// translate a RallyLocation to a Location
static Location rally_location_to_location(const RallyLocation &r_loc, const Location &homeloc) 
{
    Location ret = {};

    ret.id = MAV_CMD_NAV_LOITER_UNLIM;
    ret.options = MASK_OPTIONS_RELATIVE_ALT;

    //Currently can't do true AGL on the APM.  Relative altitudes are
    //relative to HOME point's altitude.  Terrain on the board is inbound
    //for the PX4, though.  This line will need to be updated when that happens:
    ret.alt = (r_loc.alt*100UL) + homeloc.alt;

    ret.lat = r_loc.lat;
    ret.lng = r_loc.lng;

    return ret;
}

// return best PLANE_RTL location from current position
static Location rally_find_best_location(const Location &myloc, const Location &homeloc)
{
    RallyLocation ral_loc = {};
    Location ret = {};
    if (find_best_rally_point(myloc, home, ral_loc)) {
        //we have setup Rally points: use them instead of Home for PLANE_RTL
        ret = rally_location_to_location(ral_loc, home);
    } else {
        ret = homeloc;
        // Altitude to hold over home
        ret.alt = read_alt_to_hold();
    }
    return ret;
}

#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/sensors.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// filter altitude from the barometer with a low pass filter
static LowPassFilterInt32 altitude_filter;

// Sensors are not available in HIL_MODE_ATTITUDE
#if HIL_MODE != HIL_MODE_ATTITUDE

 #if CONFIG_SONAR == ENABLED
static void init_sonar(void)
{
  #if CONFIG_SONAR_SOURCE == SONAR_SOURCE_ADC
    sonar->calculate_scaler(g.sonar_type, 3.3f);
  #else
    sonar->calculate_scaler(g.sonar_type, 5.0f);
  #endif
}
 #endif

static void init_barometer(bool full_calibration)
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("Calibrating barometer"));
    if (full_calibration) {
        barometer.calibrate();
    }else{
        barometer.update_calibration();
    }
	    // filter at 100ms sampling, with 0.7Hz cutoff frequency
    altitude_filter.set_cutoff_frequency(0.1, 0.7);
	
    gcs_send_text_P(SEVERITY_LOW, PSTR("barometer calibration complete"));
}

// return barometric altitude in centimeters
static int32_t read_barometer(void)
{
    barometer.read();
    return barometer.get_altitude() * 100.0f;
}

static int32_t plane_read_barometer(void)
{
    barometer.read();
#if LOGGING_ENABLED
    if (should_log(MASK_LOG_IMU)) {
        Log_Write_Baro();
    }
#endif
    return altitude_filter.apply(barometer.get_altitude() * 100.0);
}

/*
  ask airspeed sensor for a new value
 */
static void read_airspeed(void)
{
    if (airspeed.enabled()) {
        airspeed.read();
#if LOGGING_ENABLED == ENABLED
        if (should_log(MASK_LOG_IMU)) {
            Log_Write_Airspeed();
        }
#endif
        calc_airspeed_errors();
    }
}

static void zero_airspeed(void)
{
    airspeed.calibrate();
    gcs_send_text_P(SEVERITY_LOW,PSTR("zero airspeed calibrated"));
}

// return sonar altitude in centimeters
static int16_t read_sonar(void)
{
#if CONFIG_SONAR == ENABLED
    // exit immediately if sonar is disabled
    if( !g.sonar_enabled ) {
        sonar_alt_health = 0;
        return 0;
    }

    int16_t temp_alt = sonar->read();

    if (temp_alt >= sonar->min_distance && temp_alt <= sonar->max_distance * SONAR_RELIABLE_DISTANCE_PCT) {
        if ( sonar_alt_health < SONAR_ALT_HEALTH_MAX ) {
            sonar_alt_health++;
        }
    }else{
        sonar_alt_health = 0;
    }

 #if SONAR_TILT_CORRECTION == 1
    // correct alt for angle of the sonar
    float temp = cos_pitch_x * cos_roll_x;
    temp = max(temp, 0.707f);
    temp_alt = (float)temp_alt * temp;
 #endif

    return temp_alt;
#else
    return 0;
#endif
}


#endif // HIL_MODE != HIL_MODE_ATTITUDE


static void init_compass()
{
    if (g.compass_enabled==true) {
		if (!compass.init() || !compass.read()) {
			// make sure we don't pass a broken compass to DCM
			cliSerial->println_P(PSTR("COMPASS INIT ERROR"));
			Log_Write_Error(ERROR_SUBSYSTEM_COMPASS,ERROR_CODE_FAILED_TO_INITIALISE);
			return;
		}
		ahrs.set_compass(&compass);
	}
}

static void init_optflow()
{
#if OPTFLOW == ENABLED
    optflow.init();
    if (!optflow.healthy()) {
        g.optflow_enabled = false;
        cliSerial->print_P(PSTR("Failed to Init OptFlow\n"));
        Log_Write_Error(ERROR_SUBSYSTEM_OPTFLOW,ERROR_CODE_FAILED_TO_INITIALISE);
    }
#endif      // OPTFLOW == ENABLED
}

// read_battery - check battery voltage and current and invoke failsafe if necessary
// called at 10hz
static void read_battery(void)
{
    battery.read();

    // update compass with current value
    if (battery.monitoring() == AP_BATT_MONITOR_VOLTAGE_AND_CURRENT) {
        compass.set_current(battery.current_amps());
    }

    // check for low voltage or current if the low voltage check hasn't already been triggered
    // we only check when we're not powered by USB to avoid false alarms during bench tests
    if (!ap.usb_connected && !failsafe.battery && battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah)) {
        failsafe_battery_event();
    }
}

// plane_read_battery - reads battery voltage and current and invokes failsafe
// should be called at 10hz
static void plane_read_battery(void)
{
    battery.read();

    if (!ap.usb_connected && battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah)) {
        low_battery_event();
    }
}

// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void read_receiver_rssi(void)
{
    // avoid divide by zero
    if (g.rssi_range <= 0) {
        receiver_rssi = 0;
    }else{
        rssi_analog_source->set_pin(g.rssi_pin);
        float ret = rssi_analog_source->voltage_average() * 255 / g.rssi_range;
        receiver_rssi = constrain_int16(ret, 0, 255);
    }
}

static int32_t adjusted_altitude_cm(void)
{
    return current_loc.alt - (g.alt_offset*100);
}
#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/setup.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// Functions called from the setup menu
static int8_t   setup_accel_scale       (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_compass           (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_compassmot        (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_erase             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_flightmodes       (uint8_t argc, const Menu::arg *argv);
static int8_t   plane_setup_flightmodes (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_optflow           (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_radio             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_range             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_factory           (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_set               (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_show              (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_sonar             (uint8_t argc, const Menu::arg *argv);
static int8_t	setup_declination		(uint8_t argc, const Menu::arg *argv);
static int8_t	setup_level				(uint8_t argc, const Menu::arg *argv);
// Command/function table for the setup menu
const struct Menu::command setup_menu_commands[] PROGMEM = {
    // command			function called
    // =======          ===============
    {"reset",                       setup_factory},
    {"radio",                       setup_radio},
    {"modes",                       setup_flightmodes},
    {"modes1",                      plane_setup_flightmodes},
    {"level",                       setup_level},
    {"accel",                       setup_accel_scale},
    {"compass",                     setup_compass},
    {"compassmot",                  setup_compassmot},
    {"erase",                       setup_erase},
    {"optflow",                     setup_optflow},
    {"range",                       setup_range},
    {"set",                         setup_set},
    {"declination",       		    setup_declination},
    {"show",                        setup_show},
    {"sonar",                       setup_sonar},
};

// Create the setup menu object.
MENU(setup_menu, "setup", setup_menu_commands);

// Called from the top-level menu to run the setup menu.
static int8_t
setup_mode(uint8_t argc, const Menu::arg *argv)
{
    // Give the user some guidance
    cliSerial->printf_P(PSTR("Setup Mode\n\n\n"));

    if(g.rc_1.radio_min >= 1300) {
        delay(1000);
        cliSerial->printf_P(PSTR("\n!Warning, radio not configured!"));
        delay(1000);
        cliSerial->printf_P(PSTR("\n Type 'radio' now.\n\n"));
    }
    // Run the setup menu.  When the menu exits, we will return to the main menu.
    setup_menu.run();
    return 0;
}

static int8_t
setup_accel_scale(uint8_t argc, const Menu::arg *argv)
{
    float trim_roll, trim_pitch;
    cliSerial->println_P(PSTR("Initialising gyros"));
    ahrs.init();

    ins.init(AP_InertialSensor::COLD_START, ins_sample_rate);
    AP_InertialSensor_UserInteractStream interact(hal.console);
    if(ins.calibrate_accel(&interact, trim_roll, trim_pitch)) {
        // reset ahrs's trim to suggested values from calibration routine
        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
    }
    report_ins();
    return(0);
}

static int8_t
setup_compass(uint8_t argc, const Menu::arg *argv)
{
    if (!strcmp_P(argv[1].str, PSTR("on"))) {
        g.compass_enabled.set_and_save(true);
        init_compass();

    } else if (!strcmp_P(argv[1].str, PSTR("off"))) {
        Vector3f mag_offsets(0,0,0);
        compass.set_offsets(mag_offsets);
        compass.save_offsets();
        g.compass_enabled.set_and_save(false);

    }else{
        cliSerial->printf_P(PSTR("\nOp:[on,off]\n"));
        report_compass();
        return 0;
    }

    g.compass_enabled.save();
    report_compass();
    return 0;
}

static int8_t
setup_erase(uint8_t argc, const Menu::arg *argv)
{
    int c;

    cliSerial->printf_P(PSTR("\nType 'Y' and hit Enter to erase all waypoint and parameter data, any other key to abort: "));

    do {
        c = cliSerial->read();
    } while (-1 == c);

    if (('y' != c) && ('Y' != c))
        return(-1);
    zero_eeprom();
    return 0;
}

static int8_t
setup_flightmodes(uint8_t argc, const Menu::arg *argv)
{
    uint8_t switchPosition = 0;
    uint8_t oldSwitchPosition = 0;
    int8_t mode = 0;

    cliSerial->printf_P(PSTR("\nMode switch to edit, aileron: select modes, rudder: Simple on/off\n"));
    print_hit_enter();


    while(1) {
        delay(20);
        read_radio();
        switchPosition = readSwitch();

        // look for control switch change
        if (oldSwitchPosition != switchPosition) {

            mode = flight_modes[switchPosition];
            mode = constrain_int16(mode, 0, NUM_MODES-1);

            // update the user
            print_switch(switchPosition, mode, BIT_IS_SET(g.simple_modes, switchPosition));

            // Remember switch position
            oldSwitchPosition = switchPosition;
        }

        // look for stick input
        if (abs(g.rc_1.control_in) > 3000) {
            mode++;
            if(mode >= NUM_MODES)
                mode = 0;

            // save new mode
            flight_modes[switchPosition] = mode;

            // print new mode
            print_switch(switchPosition, mode, BIT_IS_SET(g.simple_modes, switchPosition));
            delay(500);
        }

        // look for stick input
        if (g.rc_4.control_in > 3000) {
            g.simple_modes |= (1<<switchPosition);
            // print new mode
            print_switch(switchPosition, mode, BIT_IS_SET(g.simple_modes, switchPosition));
            delay(500);
        }

        // look for stick input
        if (g.rc_4.control_in < -3000) {
            g.simple_modes &= ~(1<<switchPosition);

            // print new mode
            print_switch(switchPosition, mode, BIT_IS_SET(g.simple_modes, switchPosition));
            delay(500);

        }

        // escape hatch
        if(cliSerial->available() > 0) {

            for (mode = 0; mode < 6; mode++)
                flight_modes[mode].save();

            g.simple_modes.save();

            print_done();
            report_flight_modes();
            return (0);
        }
    }
}

static int8_t
plane_setup_flightmodes(uint8_t argc, const Menu::arg *argv)
{
    uint8_t switchPosition = 0;
    uint8_t oldSwitchPosition = 0;
    int8_t mode = 0;

    cliSerial->printf_P(PSTR("\nMove RC toggle switch to each position to edit, move aileron stick to select modes."));
    print_hit_enter();
    trim_radio();

    while(1) {
        delay(20);
        plane_read_radio();
        switchPosition = readSwitch();


        // look for control switch change
        if (oldSwitchPosition != switchPosition) {
            // force position 5 to PLANE_MANUAL
            if (switchPosition > 4) {
                plane_flight_modes[switchPosition] = PLANE_MANUAL;
            }
            // update our current mode
            mode = plane_flight_modes[switchPosition];

            // update the user
            plane_print_switch(switchPosition, mode);

            // Remember switch position
            oldSwitchPosition = switchPosition;
        }

        // look for stick input
        int16_t radioInputSwitch = radio_input_switch();

        if (radioInputSwitch != 0) {

            mode += radioInputSwitch;

            while (
                mode != PLANE_MANUAL &&
                mode != PLANE_CIRCLE &&
                mode != PLANE_STABILIZE &&
                mode != PLANE_TRAINING &&
                mode != PLANE_ACRO &&
                mode != PLANE_FLY_BY_WIRE_A &&
                mode != PLANE_FLY_BY_WIRE_B &&
                mode != PLANE_CRUISE &&
                mode != PLANE_AUTO &&
                mode != PLANE_RTL &&
                mode != PLANE_LOITER)
            {
                if (mode < PLANE_MANUAL)
                    mode = PLANE_LOITER;
                else if (mode >PLANE_LOITER)
                    mode = PLANE_MANUAL;
                else
                    mode += radioInputSwitch;
            }

            // Override position 5
            if(switchPosition > 4)
                mode = PLANE_MANUAL;

            // save new mode
            plane_flight_modes[switchPosition] = mode;

            // print new mode
            plane_print_switch(switchPosition, mode);
        }

        // escape hatch
        if(cliSerial->available() > 0) {
            // save changes
            for (mode=0; mode<6; mode++)
                plane_flight_modes[mode].save();
            plane_report_flight_modes();
            print_done();
            return (0);
        }
    }
}

// Perform radio setup.
// Called by the setup menu 'radio' command.
static int8_t
setup_radio(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->println_P(PSTR("\n\nRadio Setup:"));
    uint8_t i;

    for(i = 0; i < 100; i++) {
        delay(20);

    if (!isplane){
		read_radio();
		}
	if (isplane){	
        plane_read_radio();
		}
    }

    if(g.rc_1.radio_in < 500) {
        while(1) {
            cliSerial->printf_P(PSTR("\nNo radio; Check connectors."));
            delay(1000);
            // stop here
        }
    }

    g.rc_1.radio_min = g.rc_1.radio_in;
    g.rc_2.radio_min = g.rc_2.radio_in;
    g.rc_3.radio_min = g.rc_3.radio_in;
    g.rc_4.radio_min = g.rc_4.radio_in;
    g.rc_5.radio_min = g.rc_5.radio_in;
    g.rc_6.radio_min = g.rc_6.radio_in;
    g.rc_7.radio_min = g.rc_7.radio_in;
    g.rc_8.radio_min = g.rc_8.radio_in;

    g.rc_1.radio_max = g.rc_1.radio_in;
    g.rc_2.radio_max = g.rc_2.radio_in;
    g.rc_3.radio_max = g.rc_3.radio_in;
    g.rc_4.radio_max = g.rc_4.radio_in;
    g.rc_5.radio_max = g.rc_5.radio_in;
    g.rc_6.radio_max = g.rc_6.radio_in;
    g.rc_7.radio_max = g.rc_7.radio_in;
    g.rc_8.radio_max = g.rc_8.radio_in;

    g.rc_1.radio_trim = g.rc_1.radio_in;
    g.rc_2.radio_trim = g.rc_2.radio_in;
    g.rc_4.radio_trim = g.rc_4.radio_in;
    // 3 is not trimed
    g.rc_5.radio_trim = 1500;
    g.rc_6.radio_trim = 1500;
    g.rc_7.radio_trim = 1500;
    g.rc_8.radio_trim = 1500;

    cliSerial->printf_P(PSTR("\nMove all controls to extremes. Enter to save: "));
    while(1) {

        delay(20);
        // Filters radio input - adjust filters in the radio.pde file
        // ----------------------------------------------------------
    if (!isplane){
		read_radio();
		}
	if (isplane){	
        plane_read_radio();
		}

        g.rc_1.update_min_max();
        g.rc_2.update_min_max();
        g.rc_3.update_min_max();
        g.rc_4.update_min_max();
        g.rc_5.update_min_max();
        g.rc_6.update_min_max();
        g.rc_7.update_min_max();
        g.rc_8.update_min_max();

        if(cliSerial->available() > 0) {
            delay(20);
            while (cliSerial->read() != -1); /* flush */

            g.rc_1.save_eeprom();
            g.rc_2.save_eeprom();
            g.rc_3.save_eeprom();
            g.rc_4.save_eeprom();
            g.rc_5.save_eeprom();
            g.rc_6.save_eeprom();
            g.rc_7.save_eeprom();
            g.rc_8.save_eeprom();

            print_done();
            break;
        }
    }
    report_radio();
    return(0);
}

static int8_t
setup_range(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf_P(PSTR("\nCH 6 Ranges are divided by 1000: [low, high]\n"));

    g.radio_tuning_low.set_and_save(argv[1].i);
    g.radio_tuning_high.set_and_save(argv[2].i);
    report_tuning();
    return 0;
}

// Initialise the EEPROM to 'factory' settings (mostly defined in APM_Config.h or via defaults).
// Called by the setup menu 'factoryreset' command.
static int8_t
setup_factory(uint8_t argc, const Menu::arg *argv)
{
    int16_t c;

    cliSerial->printf_P(PSTR("\n'Y' = factory reset, any other key to abort:\n"));

    do {
        c = cliSerial->read();
    } while (-1 == c);

    if (('y' != c) && ('Y' != c))
        return(-1);

    AP_Param::erase_all();
    cliSerial->printf_P(PSTR("\nFACTORY RESET complete - please reset board to continue"));

    delay(1000);

    for (;; ) {
    }
    // note, cannot actually return here
    return(0);
}

//Set a parameter to a specified value. It will cast the value to the current type of the
//parameter and make sure it fits in case of INT8 and INT16
static int8_t setup_set(uint8_t argc, const Menu::arg *argv)
{
    int8_t value_int8;
    int16_t value_int16;

    AP_Param *param;
    enum ap_var_type p_type;

    if(argc!=3)
    {
        cliSerial->printf_P(PSTR("Invalid command. Usage: set <name> <value>\n"));
        return 0;
    }

    param = AP_Param::find(argv[1].str, &p_type);
    if(!param)
    {
        cliSerial->printf_P(PSTR("Param not found: %s\n"), argv[1].str);
        return 0;
    }

    switch(p_type)
    {
        case AP_PARAM_INT8:
            value_int8 = (int8_t)(argv[2].i);
            if(argv[2].i!=value_int8)
            {
                cliSerial->printf_P(PSTR("Value out of range for type INT8\n"));
                return 0;
            }
            ((AP_Int8*)param)->set_and_save(value_int8);
            break;
        case AP_PARAM_INT16:
            value_int16 = (int16_t)(argv[2].i);
            if(argv[2].i!=value_int16)
            {
                cliSerial->printf_P(PSTR("Value out of range for type INT16\n"));
                return 0;
            }
            ((AP_Int16*)param)->set_and_save(value_int16);
            break;

        //int32 and float don't need bounds checking, just use the value provoded by Menu::arg
        case AP_PARAM_INT32:
            ((AP_Int32*)param)->set_and_save(argv[2].i);
            break;
        case AP_PARAM_FLOAT:
            ((AP_Float*)param)->set_and_save(argv[2].f);
            break;
        default:
            cliSerial->printf_P(PSTR("Cannot set parameter of type %d.\n"), p_type);
            break;
    }

    return 0;
}

// Print the current configuration.
// Called by the setup menu 'show' command.
static int8_t
setup_show(uint8_t argc, const Menu::arg *argv)
{
    AP_Param *param;
    ap_var_type type;

    //If a parameter name is given as an argument to show, print only that parameter
    if(argc>=2)
    {

        param=AP_Param::find(argv[1].str, &type);

        if(!param)
        {
            cliSerial->printf_P(PSTR("Parameter not found: '%s'\n"), argv[1]);
            return 0;
        }
        AP_Param::show(param, argv[1].str, type, cliSerial);
        return 0;
    }

    // clear the area
    print_blanks(8);

    report_version();
    report_radio();
    report_frame();
    report_batt_monitor();
    report_sonar();
    report_flight_modes();
    report_ins();
    report_compass();
    report_optflow();

    AP_Param::show_all(cliSerial);

    return(0);
}

static int8_t
setup_sonar(uint8_t argc, const Menu::arg *argv)
{
    if (!strcmp_P(argv[1].str, PSTR("on"))) {
        g.sonar_enabled.set_and_save(true);

    } else if (!strcmp_P(argv[1].str, PSTR("off"))) {
        g.sonar_enabled.set_and_save(false);

    } else if (argc > 1 && (argv[1].i >= 0 && argv[1].i <= 3)) {
        g.sonar_enabled.set_and_save(true);      // if you set the sonar type, surely you want it on
        g.sonar_type.set_and_save(argv[1].i);

    }else{
        cliSerial->printf_P(PSTR("\nOp:[on, off, 0-3]\n"));
        report_sonar();
        return 0;
    }

    report_sonar();
    return 0;
}

// setup_compassmot - sets compass's motor interference parameters
static int8_t
setup_compassmot(uint8_t argc, const Menu::arg *argv)
{
    int8_t   comp_type;                 // throttle or current based compensation
    Vector3f compass_base;              // compass vector when throttle is zero
    Vector3f motor_impact;              // impact of motors on compass vector
    Vector3f motor_impact_scaled;       // impact of motors on compass vector scaled with throttle
    Vector3f motor_compensation;        // final compensation to be stored to eeprom
    float    throttle_pct;              // throttle as a percentage 0.0 ~ 1.0
    float    throttle_pct_max = 0.0f;   // maximum throttle reached (as a percentage 0~1.0)
    float    current_amps_max = 0.0f;   // maximum current reached
    float    interference_pct;          // interference as a percentage of total mag field (for reporting purposes only)
    uint32_t last_run_time;
    uint8_t  print_counter = 49;
    bool     updated = false;           // have we updated the compensation vector at least once

    // default compensation type to use current if possible
    if (battery.monitoring() == AP_BATT_MONITOR_VOLTAGE_AND_CURRENT) {
        comp_type = AP_COMPASS_MOT_COMP_CURRENT;
    }else{
        comp_type = AP_COMPASS_MOT_COMP_THROTTLE;
    }

    // check if radio is calibration
    pre_arm_rc_checks();
    if(!ap.pre_arm_rc_check) {
        cliSerial->print_P(PSTR("radio not calibrated\n"));
        return 0;
    }

    // check compass is enabled
    if( !g.compass_enabled ) {
        cliSerial->print_P(PSTR("compass disabled\n"));
        return 0;
    }

    // initialise compass
    init_compass();

    // disable motor compensation
    compass.motor_compensation_type(AP_COMPASS_MOT_COMP_DISABLED);
    compass.set_motor_compensation(Vector3f(0,0,0));

    // print warning that motors will spin
    // ask user to raise throttle
    // inform how to stop test
    cliSerial->print_P(PSTR("This records the impact on the compass of running the motors.  Motors will spin!\nHold throttle low, then raise to mid for 5 sec, then quickly back to low.\nPress any key to exit.\n\nmeasuring compass vs "));

    // inform what type of compensation we are attempting
    if( comp_type == AP_COMPASS_MOT_COMP_CURRENT ) {
        cliSerial->print_P(PSTR("CURRENT\n"));
    }else{
        cliSerial->print_P(PSTR("THROTTLE\n"));
    }

    // clear out user input
    while( cliSerial->available() ) {
        cliSerial->read();
    }

    // disable throttle and battery failsafe
    g.failsafe_throttle = FS_THR_DISABLED;
    g.failsafe_battery_enabled = FS_BATT_DISABLED;

    // read radio
    read_radio();

    // exit immediately if throttle is not zero
    if( g.rc_3.control_in != 0 ) {
        cliSerial->print_P(PSTR("throttle not zero\n"));
        return 0;
    }

    // get some initial compass readings
    last_run_time = millis();
    while( millis() - last_run_time < 2000 ) {
        compass.accumulate();
    }
    compass.read();

    // exit immediately if the compass is not healthy
    if( !compass.healthy() ) {
        cliSerial->print_P(PSTR("check compass\n"));
        return 0;
    }

    // store initial x,y,z compass values
    compass_base = compass.get_field();

    // initialise motor compensation
    motor_compensation = Vector3f(0,0,0);

    // clear out any user input
    while( cliSerial->available() ) {
        cliSerial->read();
    }

    // enable motors and pass through throttle
    init_rc_out();
    output_min();
    motors.armed(true);

    // initialise run time
    last_run_time = millis();

    // main run while there is no user input and the compass is healthy
    while(!cliSerial->available() && compass.healthy()) {

        // 50hz loop
        if( millis() - last_run_time > 20 ) {
            last_run_time = millis();

            // read radio input
            read_radio();

            // pass through throttle to motors
            motors.throttle_pass_through();

            // read some compass values
            compass.read();

            // read current
            read_battery();

            // calculate scaling for throttle
            throttle_pct = (float)g.rc_3.control_in / 1000.0f;
            throttle_pct = constrain_float(throttle_pct,0.0f,1.0f);

            // if throttle is zero, update base x,y,z values
            if( throttle_pct == 0.0f ) {
                compass_base = compass_base * 0.99f + compass.get_field() * 0.01f;

                // causing printing to happen as soon as throttle is lifted
                print_counter = 49;
            }else{

                // calculate diff from compass base and scale with throttle
                motor_impact = compass.get_field() - compass_base;

                // throttle based compensation
                if( comp_type == AP_COMPASS_MOT_COMP_THROTTLE ) {
                    // scale by throttle
                    motor_impact_scaled = motor_impact / throttle_pct;

                    // adjust the motor compensation to negate the impact
                    motor_compensation = motor_compensation * 0.99f - motor_impact_scaled * 0.01f;
                    updated = true;
                }else{
                    // current based compensation if more than 3amps being drawn
                    motor_impact_scaled = motor_impact / battery.current_amps();

                    // adjust the motor compensation to negate the impact if drawing over 3amps
                    if( battery.current_amps() >= 3.0f ) {
                        motor_compensation = motor_compensation * 0.99f - motor_impact_scaled * 0.01f;
                        updated = true;
                    }
                }

                // record maximum throttle and current
                throttle_pct_max = max(throttle_pct_max, throttle_pct);
                current_amps_max = max(current_amps_max, battery.current_amps());

                // display output at 1hz if throttle is above zero
                print_counter++;
                if(print_counter >= 50) {
                    print_counter = 0;
                    display_compassmot_info(motor_impact, motor_compensation);
                }
            }
        }else{
            // grab some compass values
            compass.accumulate();
        }
    }

    // stop motors
    motors.output_min();
    motors.armed(false);

    // clear out any user input
    while( cliSerial->available() ) {
        cliSerial->read();
    }

    // print one more time so the last thing printed matches what appears in the report_compass
    display_compassmot_info(motor_impact, motor_compensation);

    // set and save motor compensation
    if( updated ) {
        compass.motor_compensation_type(comp_type);
        compass.set_motor_compensation(motor_compensation);
        compass.save_motor_compensation();

        // calculate and display interference compensation at full throttle as % of total mag field
        if (comp_type == AP_COMPASS_MOT_COMP_THROTTLE) {
            // interference is impact@fullthrottle / mag field * 100
            interference_pct = motor_compensation.length() / (float)COMPASS_MAGFIELD_EXPECTED * 100.0f;
        }else{
            // interference is impact/amp * (max current seen / max throttle seen) / mag field * 100
            interference_pct = motor_compensation.length() * (current_amps_max/throttle_pct_max) / (float)COMPASS_MAGFIELD_EXPECTED * 100.0f;
        }
        cliSerial->printf_P(PSTR("\nInterference at full throttle is %d%% of mag field\n\n"),(int)interference_pct);
    }else{
        // compensation vector never updated, report failure
        cliSerial->printf_P(PSTR("Failed! Compensation disabled.  Did you forget to raise the throttle high enough?"));
        compass.motor_compensation_type(AP_COMPASS_MOT_COMP_DISABLED);
    }

    // display new motor offsets and save
    report_compass();

    return 0;
}

// display_compassmot_info - displays a status line for compassmot process
static void display_compassmot_info(Vector3f& motor_impact, Vector3f& motor_compensation)
{
    // print one more time so the last thing printed matches what appears in the report_compass
    cliSerial->printf_P(PSTR("thr:%d cur:%4.2f mot x:%4.1f y:%4.1f z:%4.1f  comp x:%4.2f y:%4.2f z:%4.2f\n"),(int)g.rc_3.control_in, (float)battery.current_amps(), (float)motor_impact.x, (float)motor_impact.y, (float)motor_impact.z, (float)motor_compensation.x, (float)motor_compensation.y, (float)motor_compensation.z);
}

static int8_t
setup_optflow(uint8_t argc, const Menu::arg *argv)
{
 #if OPTFLOW == ENABLED
    if (!strcmp_P(argv[1].str, PSTR("on"))) {
        g.optflow_enabled = true;
        init_optflow();

    } else if (!strcmp_P(argv[1].str, PSTR("off"))) {
        g.optflow_enabled = false;

    }else{
        cliSerial->printf_P(PSTR("\nOp:[on, off]\n"));
        report_optflow();
        return 0;
    }

    g.optflow_enabled.save();
    report_optflow();
 #endif     // OPTFLOW == ENABLED
    return 0;
}

static int8_t
setup_declination(uint8_t argc, const Menu::arg *argv)
{
    compass.set_declination(radians(argv[1].f));
    report_compass();
    return 0;
}

static int8_t
setup_level(uint8_t argc, const Menu::arg *argv)
{
    startup_ground();
    return 0;
}

/***************************************************************************/
// CLI reports
/***************************************************************************/

static void report_batt_monitor()
{
    cliSerial->printf_P(PSTR("\nBatt Mon:\n"));
    print_divider();
    if (battery.monitoring() == AP_BATT_MONITOR_DISABLED) print_enabled(false);
    if (battery.monitoring() == AP_BATT_MONITOR_VOLTAGE_ONLY) cliSerial->printf_P(PSTR("volts"));
    if (battery.monitoring() == AP_BATT_MONITOR_VOLTAGE_AND_CURRENT) cliSerial->printf_P(PSTR("volts and cur"));
    print_blanks(2);
}

static void report_sonar()
{
    cliSerial->printf_P(PSTR("Sonar\n"));
    print_divider();
    print_enabled(g.sonar_enabled.get());
    cliSerial->printf_P(PSTR("Type: %d (0=XL, 1=LV, 2=XLL, 3=HRLV)"), (int)g.sonar_type);
    print_blanks(2);
}

static void report_frame()
{
    cliSerial->printf_P(PSTR("Frame\n"));
    print_divider();

 #if FRAME_CONFIG == QUAD_FRAME
    cliSerial->printf_P(PSTR("Quad frame\n"));
 #elif FRAME_CONFIG == TRI_FRAME
    cliSerial->printf_P(PSTR("TRI frame\n"));
 #elif FRAME_CONFIG == HEXA_FRAME
    cliSerial->printf_P(PSTR("Hexa frame\n"));
 #elif FRAME_CONFIG == Y6_FRAME
    cliSerial->printf_P(PSTR("Y6 frame\n"));
 #elif FRAME_CONFIG == OCTA_FRAME
    cliSerial->printf_P(PSTR("Octa frame\n"));
 #elif FRAME_CONFIG == HELI_FRAME
    cliSerial->printf_P(PSTR("Heli frame\n"));
 #endif

    print_blanks(2);
}

static void report_radio()
{
    cliSerial->printf_P(PSTR("Radio\n"));
    print_divider();
    // radio
    print_radio_values();
    print_blanks(2);
}

static void report_ins()
{
    cliSerial->printf_P(PSTR("INS\n"));
    print_divider();

    print_gyro_offsets();
    print_accel_offsets_and_scaling();
    print_blanks(2);
}

static void report_compass()
{
    cliSerial->printf_P(PSTR("Compass\n"));

    print_divider();

    print_enabled(g.compass_enabled);

    // mag declination
    cliSerial->printf_P(PSTR("Mag Dec: %4.4f\n"),
                    degrees(compass.get_declination()));

    Vector3f offsets = compass.get_offsets();

    // mag offsets
    cliSerial->printf_P(PSTR("Mag off: %4.4f, %4.4f, %4.4f\n"),
                    offsets.x,
                    offsets.y,
                    offsets.z);

    // motor compensation
    cliSerial->print_P(PSTR("Motor Comp: "));
    if( compass.motor_compensation_type() == AP_COMPASS_MOT_COMP_DISABLED ) {
        cliSerial->print_P(PSTR("Off\n"));
    }else{
        if( compass.motor_compensation_type() == AP_COMPASS_MOT_COMP_THROTTLE ) {
            cliSerial->print_P(PSTR("Throttle"));
        }
        if( compass.motor_compensation_type() == AP_COMPASS_MOT_COMP_CURRENT ) {
            cliSerial->print_P(PSTR("Current"));
        }
        Vector3f motor_compensation = compass.get_motor_compensation();
        cliSerial->printf_P(PSTR("\nComp Vec: %4.2f, %4.2f, %4.2f\n"),
                        motor_compensation.x,
                        motor_compensation.y,
                        motor_compensation.z);
    }
    print_blanks(1);
}

static void report_flight_modes()
{

    cliSerial->printf_P(PSTR("Flight modes\n"));
    print_divider();

    for(int16_t i = 0; i < 6; i++ ) {
        print_switch(i, flight_modes[i], BIT_IS_SET(g.simple_modes, i));
    }
    print_blanks(2);
}

static void plane_report_flight_modes()
{
    //print_blanks(2);
    cliSerial->printf_P(PSTR("Flight modes\n"));
    print_divider();

    for(int16_t i = 0; i < 6; i++ ) {
        plane_print_switch(i, plane_flight_modes[i]);
    }
    print_blanks(2);
}

void report_optflow()

{
 #if OPTFLOW == ENABLED
    cliSerial->printf_P(PSTR("OptFlow\n"));

    print_divider();

    print_enabled(g.optflow_enabled);

    print_blanks(2);
 #endif     // OPTFLOW == ENABLED
}

/***************************************************************************/
// CLI utilities
/***************************************************************************/

static void
print_radio_values()
{
    cliSerial->printf_P(PSTR("CH1: %d | %d\n"), (int)g.rc_1.radio_min, (int)g.rc_1.radio_max);
    cliSerial->printf_P(PSTR("CH2: %d | %d\n"), (int)g.rc_2.radio_min, (int)g.rc_2.radio_max);
    cliSerial->printf_P(PSTR("CH3: %d | %d\n"), (int)g.rc_3.radio_min, (int)g.rc_3.radio_max);
    cliSerial->printf_P(PSTR("CH4: %d | %d\n"), (int)g.rc_4.radio_min, (int)g.rc_4.radio_max);
    cliSerial->printf_P(PSTR("CH5: %d | %d\n"), (int)g.rc_5.radio_min, (int)g.rc_5.radio_max);
    cliSerial->printf_P(PSTR("CH6: %d | %d\n"), (int)g.rc_6.radio_min, (int)g.rc_6.radio_max);
    cliSerial->printf_P(PSTR("CH7: %d | %d\n"), (int)g.rc_7.radio_min, (int)g.rc_7.radio_max);
    cliSerial->printf_P(PSTR("CH8: %d | %d\n"), (int)g.rc_8.radio_min, (int)g.rc_8.radio_max);
}

static void
print_switch(uint8_t p, uint8_t m, bool b)
{
    cliSerial->printf_P(PSTR("Pos %d:\t"),p);
    print_flight_mode(cliSerial, m);
    cliSerial->printf_P(PSTR(",\t\tSimple: "));
    if(b)
        cliSerial->printf_P(PSTR("ON\n"));
    else
        cliSerial->printf_P(PSTR("OFF\n"));
}

static void
plane_print_switch(uint8_t p, uint8_t m)
{
    cliSerial->printf_P(PSTR("Pos %d: "),p);
    print_flight_mode(cliSerial, m);

    cliSerial->println();
}

static void
print_done()
{
    cliSerial->printf_P(PSTR("\nSaved\n"));
}

static void zero_eeprom(void)
{
    cliSerial->printf_P(PSTR("\nErasing EEPROM\n"));

    for (uint16_t i = 0; i < EEPROM_MAX_ADDR; i++) {
        hal.storage->write_byte(i, 0);
    }

    cliSerial->printf_P(PSTR("done\n"));
}

static void
print_accel_offsets_and_scaling(void)
{
    const Vector3f &accel_offsets = ins.get_accel_offsets();
    const Vector3f &accel_scale = ins.get_accel_scale();
    cliSerial->printf_P(PSTR("A_off: %4.2f, %4.2f, %4.2f\nA_scale: %4.2f, %4.2f, %4.2f\n"),
                    (float)accel_offsets.x,                           // Pitch
                    (float)accel_offsets.y,                           // Roll
                    (float)accel_offsets.z,                           // YAW
                    (float)accel_scale.x,                             // Pitch
                    (float)accel_scale.y,                             // Roll
                    (float)accel_scale.z);                            // YAW
}

static void
print_gyro_offsets(void)
{
    const Vector3f &gyro_offsets = ins.get_gyro_offsets();
    cliSerial->printf_P(PSTR("G_off: %4.2f, %4.2f, %4.2f\n"),
                    (float)gyro_offsets.x,
                    (float)gyro_offsets.y,
                    (float)gyro_offsets.z);
}

#endif // CLI_ENABLED

static void
print_blanks(int16_t num)
{
    while(num > 0) {
        num--;
        cliSerial->println("");
    }
}

static void
print_divider(void)
{
    for (int i = 0; i < 40; i++) {
        cliSerial->print_P(PSTR("-"));
    }
    cliSerial->println();
}

static void print_enabled(bool b)
{
    if(b)
        cliSerial->print_P(PSTR("en"));
    else
        cliSerial->print_P(PSTR("dis"));
    cliSerial->print_P(PSTR("abled\n"));
}

static void
init_esc()

{
    // reduce update rate to motors to 50Hz
    motors.set_update_rate(50);

    // we enable the motors directly here instead of calling output_min because output_min would send a low signal to the ESC and disrupt the calibration process
    motors.enable();
    motors.armed(true);
    while(1) {
        read_radio();
        delay(100);
        AP_Notify::flags.esc_calibration = true;
        motors.throttle_pass_through();
    }
}

static void report_version()
{
    cliSerial->printf_P(PSTR("FW Ver: %d\n"),(int)g.k_format_version);
    print_divider();
    print_blanks(2);
}

static void report_tuning()
{
    cliSerial->printf_P(PSTR("\nTUNE:\n"));
    print_divider();
    if (g.radio_tuning == 0) {
        print_enabled(g.radio_tuning.get());
    }else{
        float low  = (float)g.radio_tuning_low.get() / 1000;
        float high = (float)g.radio_tuning_high.get() / 1000;
        cliSerial->printf_P(PSTR(" %d, Low:%1.4f, High:%1.4f\n"),(int)g.radio_tuning.get(), low, high);
    }
    print_blanks(2);
}

static int8_t
radio_input_switch(void)
{
    static int8_t bouncer = 0;


    if (int16_t(channel_roll->radio_in - channel_roll->radio_trim) > 100) {
        bouncer = 10;
    }
    if (int16_t(channel_roll->radio_in - channel_roll->radio_trim) < -100) {
        bouncer = -10;
    }
    if (bouncer >0) {
        bouncer--;
    }
    if (bouncer <0) {
        bouncer++;
    }

    if (bouncer == 1 || bouncer == -1) {
        return bouncer;
    } else {
        return 0;
    }
}
#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/system.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/

#if CLI_ENABLED == ENABLED
// Functions called from the top-level menu
static int8_t   process_logs(uint8_t argc, const Menu::arg *argv);      // in Log.pde
static int8_t   setup_mode(uint8_t argc, const Menu::arg *argv);        // in setup.pde
static int8_t   test_mode(uint8_t argc, const Menu::arg *argv);         // in test.cpp
static int8_t   reboot_board(uint8_t argc, const Menu::arg *argv);

// This is the help function
static int8_t   main_menu_help(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf_P(PSTR("Commands:\n"
                         "  logs\n"
                         "  setup\n"
                         "  test\n"
                         "  reboot\n"
                         "\n"));
    return(0);
}

// Command/function table for the top-level menu.
const struct Menu::command main_menu_commands[] PROGMEM = {
//   command		function called
//   =======        ===============
    {"logs",                process_logs},
    {"setup",               setup_mode},
    {"test",                test_mode},
    {"reboot",              reboot_board},
    {"help",                main_menu_help},
};

// Create the top-level menu object.
MENU(main_menu, THISFIRMWARE, main_menu_commands);

static int8_t reboot_board(uint8_t argc, const Menu::arg *argv)
{
    hal.scheduler->reboot(false);
    return 0;
}

// the user wants the CLI. It never exits
static void run_cli(AP_HAL::UARTDriver *port)
{
    cliSerial = port;
    Menu::set_port(port);
    port->set_blocking_writes(true);

    // disable the mavlink delay callback
    hal.scheduler->register_delay_callback(NULL, 5);

    // disable main_loop failsafe
    failsafe_disable();

    // cut the engines
    if(motors.armed()) {
        motors.armed(false);
        motors.output();
    }

    while (1) {
        main_menu.run();
    }
}

#endif // CLI_ENABLED

static void init_ardupilot()
{
    if (!hal.gpio->usb_connected()) {
        // USB is not connected, this means UART0 may be a Xbee, with
        // its darned bricking problem. We can't write to it for at
        // least one second after powering up. Simplest solution for
        // now is to delay for 1 second. Something more elegant may be
        // added later
        delay(1000);
    }

    // Console serial port
    //
    // The console port buffers are defined to be sufficiently large to support
    // the MAVLink protocol efficiently
    //
#if HIL_MODE != HIL_MODE_DISABLED
    // we need more memory for HIL, as we get a much higher packet rate
    hal.uartA->begin(SERIAL0_BAUD, 256, 256);
#else
    // use a bit less for non-HIL operation
    hal.uartA->begin(SERIAL0_BAUD, 512, 128);
#endif

    // GPS serial port.
    //
#if GPS_PROTOCOL != GPS_PROTOCOL_IMU
    // standard gps running. Note that we need a 256 byte buffer for some
    // GPS types (eg. UBLOX)
    hal.uartB->begin(38400, 256, 16);
#endif

    cliSerial->printf_P(PSTR("\n\nInit " FIRMWARE_STRING
                         "\n\nFree RAM: %u\n"),
                        hal.util->available_memory());

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    /*
      run the timer a bit slower on APM2 to reduce the interrupt load
      on the CPU
     */
    hal.scheduler->set_timer_speed(500);
#endif

    //
    // Report firmware version code expect on console (check of actual EEPROM format version is done in load_parameters function)
    //
    report_version();

    // load parameters from EEPROM
    load_parameters();

    BoardConfig.init();

    // allow servo set on all channels except first 4
	ServoRelayEvents.set_channel_mask(0xFFF0);

    set_control_channels();	// Plane

    relay.init();

    // init EPM cargo gripper
#if EPM_ENABLED == ENABLED
    epm.init();
#endif

    // initialise notify system
    notify.init(false);

    // initialise battery monitor
    battery.init();

#if CONFIG_SONAR == ENABLED
 #if CONFIG_SONAR_SOURCE == SONAR_SOURCE_ADC
    sonar_analog_source = new AP_ADC_AnalogSource(
            &adc, CONFIG_SONAR_SOURCE_ADC_CHANNEL, 0.25);
 #elif CONFIG_SONAR_SOURCE == SONAR_SOURCE_ANALOG_PIN
    sonar_analog_source = hal.analogin->channel(
            CONFIG_SONAR_SOURCE_ANALOG_PIN);
 #else
  #warning "Invalid CONFIG_SONAR_SOURCE"
 #endif
    sonar = new AP_RangeFinder_MaxsonarXL(sonar_analog_source,
            &sonar_mode_filter);
#endif

    rssi_analog_source      = hal.analogin->channel(g.rssi_pin);
    board_vcc_analog_source = hal.analogin->channel(ANALOG_INPUT_BOARD_VCC);

    // keep a record of how many resets have happened. This can be
    // used to detect in-flight resets
    g.num_resets.set_and_save(g.num_resets+1);

#if HIL_MODE != HIL_MODE_ATTITUDE
    barometer.init();
#endif

    // init the GCS
    gcs[0].init(hal.uartA);

    // Register the mavlink service callback. This will run
    // anytime there are more than 5ms remaining in a call to
    // hal.scheduler->delay.
    hal.scheduler->register_delay_callback(mavlink_delay_cb, 5);

    // we start by assuming USB connected, as we initialed the serial
    // port with SERIAL0_BAUD. check_usb_mux() fixes this if need be.
    ap.usb_connected = true;
    check_usb_mux();

#if CONFIG_HAL_BOARD != HAL_BOARD_APM2
    // we have a 2nd serial port for telemetry on all boards except
    // APM2. We actually do have one on APM2 but it isn't necessary as
    // a MUX is used
    hal.uartC->begin(map_baudrate(g.serial1_baud, SERIAL1_BAUD), 128, 128);
    gcs[1].init(hal.uartC);
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 2
    if (hal.uartD != NULL) {
        hal.uartD->begin(map_baudrate(g.serial2_baud, SERIAL2_BAUD), 128, 128);
        gcs[2].init(hal.uartD);
    }
#endif

    // identify ourselves correctly with the ground station
    mavlink_system.sysid = g.sysid_this_mav;
    mavlink_system.type = 2; //MAV_QUADROTOR;

#if LOGGING_ENABLED == ENABLED
    DataFlash.Init(log_structure, sizeof(log_structure)/sizeof(log_structure[0]));
    if (!DataFlash.CardInserted()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("No dataflash inserted"));
        g.log_bitmask.set(0);
    } else if (DataFlash.NeedErase()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("ERASING LOGS"));
        do_erase_logs();
        for (uint8_t i=0; i<num_gcs; i++) {
            gcs[i].reset_cli_timeout();
        }
    }
#endif

    init_rc_in();               // sets up rc channels from radio
    init_rc_out();              // sets up motors and output to escs

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check, 1000);

#if HIL_MODE != HIL_MODE_ATTITUDE
 #if CONFIG_ADC == ENABLED
    // begin filtering the ADC Gyros
    adc.Init();           // APM ADC library initialization
 #endif // CONFIG_ADC
#endif // HIL_MODE

    // Do GPS init
    g_gps = &g_gps_driver;
    // GPS Initialization
    g_gps->init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_1G);

    if(g.compass_enabled)
        init_compass();

    // init the optical flow sensor
    if(g.optflow_enabled) {
        init_optflow();
    }
		
    // initialise airspeed sensor
    airspeed.init();

    // give AHRS the airspeed sensor
    ahrs.set_airspeed(&airspeed);

#if FENCE_TRIGGERED_PIN > 0
    hal.gpio->pinMode(FENCE_TRIGGERED_PIN, OUTPUT);
    digitalWrite(FENCE_TRIGGERED_PIN, LOW);
#endif

    // initialise inertial nav
    inertial_nav.init();

#ifdef USERHOOK_INIT
    USERHOOK_INIT
#endif

#if CLI_ENABLED == ENABLED
    const prog_char_t *msg = PSTR("\nPress ENTER 3 times to start interactive setup\n");
    cliSerial->println_P(msg);
    if (gcs[1].initialised) {
        hal.uartC->println_P(msg);
    }
    if (num_gcs > 2 && gcs[2].initialised) {
        hal.uartD->println_P(msg);
    }
#endif // CLI_ENABLED

#if HIL_MODE != HIL_MODE_DISABLED
    while (!barometer.healthy) {
        // the barometer becomes healthy when we get the first
        // HIL_STATE message
        gcs_send_text_P(SEVERITY_LOW, PSTR("Waiting for first HIL_STATE message"));
        delay(1000);
    }
#endif

#if HIL_MODE != HIL_MODE_ATTITUDE
    // read Baro pressure at ground
    //-----------------------------
    init_barometer(true);
#endif

    // initialise sonar
#if CONFIG_SONAR == ENABLED
    init_sonar();
#endif

    // initialize commands
    // -------------------
    init_commands();
    plane_init_commands();

    reset_control_switch();
	
    init_aux_switches();
	
#if LOGGING_ENABLED == ENABLED
    Log_Write_Startup();
#endif


    // choose the nav controller
    set_nav_controller();
	
	// Makes the servos wiggle once
    if (!g.skip_gyro_cal) { demo_servos(1); }
	
	    // read the radio to set trims
    trim_radio();

	    // reset last heartbeat time, so we don't trigger failsafe on slow startup
    plane_failsafe.last_heartbeat_ms = millis();
	
    plane_set_mode(PLANE_MANUAL);

	//INS ground start
    startup_ground();
	
	// Makes the servos wiggle - 3 times signals ready to fly
	    if (!g.skip_gyro_cal) { demo_servos(3); }
	
	// Save the settings for in-air restart
    // ------------------------------------
    //save_EEPROM_groundstart();
	
	// we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we are
    // ready to fly
    hal.uartA->set_blocking_writes(false);
    hal.uartC->set_blocking_writes(false);
    if (hal.uartD != NULL) {
        hal.uartD->set_blocking_writes(false);
    }

#if 0
    // leave GPS blocking until we have support for correct handling
    // of GPS config in uBlox when non-blocking
    hal.uartB->set_blocking_writes(false);
#endif
	
    cliSerial->print_P(PSTR("\nReady to FLY "));
}

//******************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//******************************************************************************
static void startup_ground()
{
    gcs_send_text_P(SEVERITY_LOW,PSTR("GROUND START"));

    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    ahrs.init();
    ahrs.set_fast_gains(true);

    // Warm up and read Gyro offsets
    // -----------------------------
    ins.init(AP_InertialSensor::COLD_START,ins_sample_rate);
			 
 #if CLI_ENABLED == ENABLED
    report_ins();
 #endif

    // set landed flag
    set_land_complete(true);
	
 // ahrs.reset();  // necessary??
	  
	  
    if (airspeed.enabled()) {
        // initialize airspeed sensor
        // --------------------------
        zero_airspeed();
    } else {
        gcs_send_text_P(SEVERITY_LOW,PSTR("NO airspeed"));
    }
}

// set_mode - change flight mode and perform any necessary initialisation
// optional force parameter used to force the flight mode change (used only first time mode is set)
// returns true if mode was succesfully set
// ACRO, STABILIZE, ALTHOLD, LAND, DRIFT and SPORT can always be set successfully but the return state of other flight modes should be checked and the caller should deal with failures appropriately
static bool set_mode(uint8_t mode)
{
    // boolean to record if flight mode could be set
    bool success = false;
    bool ignore_checks = !motors.armed();   // allow switching to any mode if disarmed.  We rely on the arming check to perform

    // return immediately if we are already in the desired mode
    if (mode == control_mode) {
        return true;
    }

    switch(mode) {
        case ACRO:
            success = true;
            set_yaw_mode(ACRO_YAW);
            set_roll_pitch_mode(ACRO_RP);
            set_throttle_mode(ACRO_THR);
            set_nav_mode(NAV_NONE);
            break;

        case STABILIZE:
            success = true;
            set_yaw_mode(STABILIZE_YAW);
            set_roll_pitch_mode(STABILIZE_RP);
            set_throttle_mode(STABILIZE_THR);
            set_nav_mode(NAV_NONE);
            break;

        case ALT_HOLD:
            success = true;
            set_yaw_mode(ALT_HOLD_YAW);
            set_roll_pitch_mode(ALT_HOLD_RP);
            set_throttle_mode(ALT_HOLD_THR);
            set_nav_mode(NAV_NONE);
            break;

        case AUTO:
            // check we have a GPS and at least one mission command (note the home position is always command 0)
            if ((GPS_ok() && g.command_total > 1) || ignore_checks) {
                success = true;
                // roll-pitch, throttle and yaw modes will all be set by the first nav command
                init_commands();            // clear the command queues. will be reloaded when "run_autopilot" calls "update_commands" function
            }
            break;

        case CIRCLE:
            if (GPS_ok() || ignore_checks) {
                success = true;
                set_roll_pitch_mode(CIRCLE_RP);
                set_throttle_mode(CIRCLE_THR);
                set_nav_mode(CIRCLE_NAV);
                set_yaw_mode(CIRCLE_YAW);
            }
            break;

        case LOITER:
            if (GPS_ok() || ignore_checks) {
                success = true;
                set_yaw_mode(LOITER_YAW);
                set_roll_pitch_mode(LOITER_RP);
                set_throttle_mode(LOITER_THR);
                set_nav_mode(LOITER_NAV);
            }
            break;

        case POSITION:
            if (GPS_ok() || ignore_checks) {
                success = true;
                set_yaw_mode(POSITION_YAW);
                set_roll_pitch_mode(POSITION_RP);
                set_throttle_mode(POSITION_THR);
                set_nav_mode(POSITION_NAV);
            }
            break;

        case GUIDED:
            if (GPS_ok() || ignore_checks) {
                success = true;
                set_yaw_mode(get_wp_yaw_mode(false));
                set_roll_pitch_mode(GUIDED_RP);
                set_throttle_mode(GUIDED_THR);
                set_nav_mode(GUIDED_NAV);
            }
            break;

        case LAND:
            success = true;
            do_land(NULL);  // land at current location
            break;

        case RTL:
            if (GPS_ok() || ignore_checks) {
                success = true;
                do_RTL();
            }
            break;

        case OF_LOITER:
            if (g.optflow_enabled || ignore_checks) {
                success = true;
                set_yaw_mode(OF_LOITER_YAW);
                set_roll_pitch_mode(OF_LOITER_RP);
                set_throttle_mode(OF_LOITER_THR);
                set_nav_mode(OF_LOITER_NAV);
            }
            break;

#if DRIFT == ENABLED
        case DRIFT:
            success = true;
            set_yaw_mode(YAW_DRIFT);
            set_roll_pitch_mode(ROLL_PITCH_DRIFT);
            set_nav_mode(NAV_NONE);
            set_throttle_mode(DRIFT_THR);
            break;
#endif

        case SPORT:
            success = true;
            set_yaw_mode(SPORT_YAW);
            set_roll_pitch_mode(SPORT_RP);
            set_throttle_mode(SPORT_THR);
            set_nav_mode(NAV_NONE);
            // reset acro angle targets to current attitude
            acro_roll = ahrs.roll_sensor;
            acro_pitch = ahrs.pitch_sensor;
            control_yaw = ahrs.yaw_sensor;
            break;

        default:
            success = false;
            break;
    }

    // update flight mode
    if (success) {
        control_mode = mode;
        Log_Write_Mode(control_mode);
    }else{
        // Log error that we failed to enter desired flight mode
        Log_Write_Error(ERROR_SUBSYSTEM_FLIGHT_MODE,mode);
    }

    // return success or failure
    return success;
}

static void plane_set_mode(enum FlightMode mode)
{
    if(plane_control_mode == mode) {
        // don't switch modes if we are already in the correct mode.
        return;
    }
    if(g.auto_trim > 0 && plane_control_mode == PLANE_MANUAL)
        trim_control_surfaces();

    plane_control_mode = mode;

    switch(plane_control_mode)
    {
    case PLANE_INITIALISING:
    case PLANE_MANUAL:
    case PLANE_STABILIZE:
    case PLANE_TRAINING:
    case PLANE_FLY_BY_WIRE_A:
        break;

    case PLANE_ACRO:
        acro_state.locked_roll = false;
        acro_state.locked_pitch = false;
        break;

    case PLANE_CRUISE:
        cruise_state.locked_heading = false;
        cruise_state.lock_timer_ms = 0;
        target_altitude_cm = current_loc.alt;
        break;

    case PLANE_FLY_BY_WIRE_B:
        target_altitude_cm = current_loc.alt;
        break;

    case PLANE_CIRCLE:
        // the altitude to circle at is taken from the current altitude
        next_WP.alt = current_loc.alt;
        break;

    case PLANE_AUTO:
        prev_WP = current_loc;
        update_auto();
        break;

    case PLANE_RTL:
        prev_WP = current_loc;
        plane_do_RTL();
        break;

    case PLANE_LOITER:
        do_loiter_at_location();
        break;

    case PLANE_GUIDED:
        guided_throttle_passthru = false;
        set_guided_WP();
        break;

    default:
        prev_WP = current_loc;
        plane_do_RTL();
        break;
    }

    // if in an auto-throttle mode, start with throttle suppressed for
    // safety. suppress_throttle() will unsupress it when appropriate
    if (plane_control_mode == PLANE_CIRCLE || plane_control_mode >= PLANE_FLY_BY_WIRE_B) {
        auto_throttle_mode = true;
        throttle_suppressed = true;
    } else {
        auto_throttle_mode = false;        
        throttle_suppressed = false;
    }

    // reset attitude integrators on mode change
    rollController.reset_I();
    pitchController.reset_I();
    yawController.reset_I();    
}

// returns true if the GPS is ok and home position is set
static bool GPS_ok()
{
    if (g_gps != NULL && ap.home_is_set && g_gps->status() == GPS::GPS_OK_FIX_3D && !gps_glitch.glitching() && !failsafe.gps) {
        return true;
    }else{
        return false;
    }
}

// returns true or false whether mode requires GPS
static bool mode_requires_GPS(uint8_t mode) {
    switch(mode) {
        case AUTO:
        case GUIDED:
        case LOITER:
        case RTL:
        case CIRCLE:
        case POSITION:
#if DRIFT == ENABLED
        case DRIFT:
            return true;
#endif
			default:
            return false;
    }

    return false;
}

// manual_flight_mode - returns true if flight mode is completely manual (i.e. roll, pitch and yaw controlled by pilot)
static bool manual_flight_mode(uint8_t mode) {
    switch(mode) {
        case ACRO:
        case STABILIZE:
#if DRIFT == ENABLED
        case DRIFT:
#endif
        case SPORT:
            return true;
        default:
            return false;
    }

    return false;
}

// update_auto_armed - update status of auto_armed flag
static void update_auto_armed()
{
    // disarm checks
    if(ap.auto_armed){
        // if motors are disarmed, auto_armed should also be false
        if(!motors.armed()) {
            set_auto_armed(false);
            return;
        }
        // if in stabilize or acro flight mode and throttle is zero, auto-armed should become false
        if(manual_flight_mode(control_mode) && g.rc_3.control_in == 0 && !failsafe.radio) {
            set_auto_armed(false);
        }
    }else{
        // arm checks
        
#if FRAME_CONFIG == HELI_FRAME
        // for tradheli if motors are armed and throttle is above zero and the motor is started, auto_armed should be true
        if(motors.armed() && g.rc_3.control_in != 0 && motors.motor_runup_complete()) {
            set_auto_armed(true);
        }
#else
        // if motors are armed and throttle is above zero auto_armed should be true
        if(motors.armed() && g.rc_3.control_in != 0) {
            set_auto_armed(true);
        }
#endif // HELI_FRAME
    }
}

static void check_long_failsafe()
{
    uint32_t tnow = millis();
    // only act on changes
    // -------------------
    if(plane_failsafe.state != FAILSAFE_LONG && plane_failsafe.state != FAILSAFE_GCS) {
        if (plane_failsafe.rc_override_active && (tnow - plane_failsafe.last_heartbeat_ms) > g.long_fs_timeout*1000) {
            failsafe_long_on_event(FAILSAFE_LONG);
        } else if (!plane_failsafe.rc_override_active && 
                   plane_failsafe.state == FAILSAFE_SHORT && 
                   (tnow - plane_failsafe.ch3_timer_ms) > g.long_fs_timeout*1000) {
            failsafe_long_on_event(FAILSAFE_LONG);
        } else if (g.gcs_heartbeat_fs_enabled != GCS_FAILSAFE_OFF && 
                   plane_failsafe.last_heartbeat_ms != 0 &&
                   (tnow - plane_failsafe.last_heartbeat_ms) > g.long_fs_timeout*1000) {
            failsafe_long_on_event(FAILSAFE_GCS);
        } else if (g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HB_RSSI && 
                   plane_failsafe.last_radio_status_remrssi_ms != 0 &&
                   (tnow - plane_failsafe.last_radio_status_remrssi_ms) > g.long_fs_timeout*1000) {
            failsafe_long_on_event(FAILSAFE_GCS);
        }
    } else {
        // We do not change state but allow for user to change mode
        if (plane_failsafe.state == FAILSAFE_GCS && 
            (tnow - plane_failsafe.last_heartbeat_ms) < g.short_fs_timeout*1000) {
            plane_failsafe.state = FAILSAFE_NONE;
        } else if (plane_failsafe.state == FAILSAFE_LONG && 
                   plane_failsafe.rc_override_active && 
                   (tnow - plane_failsafe.last_heartbeat_ms) < g.short_fs_timeout*1000) {
            plane_failsafe.state = FAILSAFE_NONE;
        } else if (plane_failsafe.state == FAILSAFE_LONG && 
                   !plane_failsafe.rc_override_active && 
                   !plane_failsafe.ch3_failsafe) {
            plane_failsafe.state = FAILSAFE_NONE;
        }
    }
}

static void check_short_failsafe()
{
    // only act on changes
    // -------------------
    if(plane_failsafe.state == FAILSAFE_NONE) {
        if(plane_failsafe.ch3_failsafe) {                                              // The condition is checked and the flag ch3_failsafe is set in radio.pde
            failsafe_short_on_event(FAILSAFE_SHORT);
        }
    }

    if(plane_failsafe.state == FAILSAFE_SHORT) {
        if(!plane_failsafe.ch3_failsafe) {
            failsafe_short_off_event();
        }
    }
}

static void resetPerfData(void) {
    mainLoop_count                  = 0;
    perf_info_max_time              = 0;
    ahrs.renorm_range_count         = 0;
    ahrs.renorm_blowup_count        = 0;
    gps_fix_count                   = 0;
    perf_mon_timer                  = millis();
}

static void print_comma(void)
{
    cliSerial->print_P(PSTR(","));
}

#if LOGGING_ENABLED == ENABLED
static bool should_log(uint32_t mask)
{
    if (!(mask & g.log_bitmask) || in_mavlink_delay) {
        return false;
    }
    bool armed;
    if (arming.arming_required() == AP_Arming::NO) {
        // for logging purposes consider us armed if we either don't
        // have a safety switch, or we have one and it is disarmed
        armed = (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED);
    } else {
        armed = arming.is_armed();
    }
    bool ret = armed || (g.log_bitmask & MASK_LOG_WHEN_DISARMED) != 0;
    if (ret && !DataFlash.logging_started() && !in_log_download) {
        // we have to set in_mavlink_delay to prevent logging while
        // writing headers
        in_mavlink_delay = true;
        start_logging();
        in_mavlink_delay = false;
    }
    return ret;
}
#endif

/*
 *  map from a 8 bit EEPROM baud rate to a real baud rate
 */
static uint32_t map_baudrate(int8_t rate, uint32_t default_baud)
{
    switch (rate) {
    case 1:    return 1200;
    case 2:    return 2400;
    case 4:    return 4800;
    case 9:    return 9600;
    case 19:   return 19200;
    case 38:   return 38400;
    case 57:   return 57600;
    case 111:  return 111100;
    case 115:  return 115200;
    }
    cliSerial->println_P(PSTR("Invalid baudrate"));
    return default_baud;
}

static void check_usb_mux(void)
{
    bool usb_check = hal.gpio->usb_connected();
    if (usb_check == ap.usb_connected) {
        return;
    }

    // the user has switched to/from the telemetry port
    ap.usb_connected = usb_check;

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    // the APM2 has a MUX setup where the first serial port switches
    // between USB and a TTL serial connection. When on USB we use
    // SERIAL0_BAUD, but when connected as a TTL serial port we run it
    // at SERIAL1_BAUD.
    if (ap.usb_connected) {
        hal.uartA->begin(SERIAL0_BAUD);
    } else {
        hal.uartA->begin(map_baudrate(g.serial1_baud, SERIAL1_BAUD));
    }
#endif
}

/*
 * Read Vcc vs 1.1v internal reference
 */
uint16_t board_voltage(void)
{
    return board_vcc_analog_source->voltage_latest() * 1000;
}

//
// print_flight_mode - prints flight mode to serial port.
//
static void
print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode)
{
    switch (mode) {
    case 0:
        if (isplane){port->print_P(PSTR("MANUAL"));}
		else {port->print_P(PSTR("STABILIZE"));}
        break;
    case 1:
        if (isplane){port->print_P(PSTR("CIRCLE"));}
		else {port->print_P(PSTR("ACRO"));}
        break;
    case 2:
        if (isplane){port->print_P(PSTR("STABILIZE"));}
		else {port->print_P(PSTR("ALT_HOLD"));}
        break;
    case 3:
        if (isplane){port->print_P(PSTR("TRAINING"));}
		else {port->print_P(PSTR("AUTO"));}
        break;
    case 4:
        if (isplane){port->print_P(PSTR("ACRO"));}
		else {port->print_P(PSTR("GUIDED"));}
        break;
    case 5:
        if (isplane){port->print_P(PSTR("FBW_A"));}
		else {port->print_P(PSTR("LOITER"));}
        break;
    case 6:
        if (isplane){port->print_P(PSTR("FBW_B"));}
		else {port->print_P(PSTR("RTL"));}
        break;
    case 7:
        if (isplane){port->print_P(PSTR("CRUISE"));}
		else {port->print_P(PSTR("CIRCLE"));}
        break;
    case 8:
        port->print_P(PSTR("POSITION"));
        break;
    case 9:
        port->print_P(PSTR("LAND"));
        break;
    case 10:
        if (isplane){port->print_P(PSTR("AUTO"));}
		else {port->print_P(PSTR("OF_LOITER"));}
        break;
    case 11:
        if (isplane){port->print_P(PSTR("RTL"));}
		else {port->print_P(PSTR("DRIFT"));}
        break;
    case 12:
        port->print_P(PSTR("LOITER"));
        break;
    case 13:
        port->print_P(PSTR("SPORT"));
        break;
    case 15:
        port->print_P(PSTR("GUIDED"));
        break;
    case 16:
        port->print_P(PSTR("INITIALIZING"));
        break;

		
    default:
        port->printf_P(PSTR("Mode(%u)"), (unsigned)mode);
        break;
    }
}
#line 1 "/d/Copter/GIT/ArduHybrid/ArduHybrid/test.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
#if HIL_MODE != HIL_MODE_ATTITUDE && HIL_MODE != HIL_MODE_SENSORS
static int8_t   test_baro(uint8_t argc,                 const Menu::arg *argv);
#endif
static int8_t   test_compass(uint8_t argc,              const Menu::arg *argv);
static int8_t   test_gps(uint8_t argc,                  const Menu::arg *argv);
static int8_t   test_ins(uint8_t argc,                  const Menu::arg *argv);
static int8_t   test_logging(uint8_t argc,              const Menu::arg *argv);
static int8_t   test_motors(uint8_t argc,               const Menu::arg *argv);
static int8_t   test_motorsync(uint8_t argc,            const Menu::arg *argv);
static int8_t   test_optflow(uint8_t argc,              const Menu::arg *argv);
static int8_t   test_radio_pwm(uint8_t argc,            const Menu::arg *argv);
static int8_t   test_radio(uint8_t argc,                const Menu::arg *argv);
static int8_t   test_relay(uint8_t argc,                const Menu::arg *argv);
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
static int8_t   test_shell(uint8_t argc,                const Menu::arg *argv);
#endif
#if HIL_MODE != HIL_MODE_ATTITUDE && HIL_MODE != HIL_MODE_SENSORS
static int8_t   test_sonar(uint8_t argc,                const Menu::arg *argv);
#endif

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
const struct Menu::command test_menu_commands[] PROGMEM = {
#if HIL_MODE != HIL_MODE_ATTITUDE && HIL_MODE != HIL_MODE_SENSORS
    {"baro",                test_baro},
#endif
    {"compass",             test_compass},
    {"gps",                 test_gps},
    {"ins",                 test_ins},
    {"logging",             test_logging},
    {"motors",              test_motors},
    {"motorsync",           test_motorsync},
    {"optflow",             test_optflow},
    {"pwm",                 test_radio_pwm},
    {"radio",               test_radio},
    {"relay",               test_relay},
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    {"shell", 				test_shell},
#endif
#if HIL_MODE != HIL_MODE_ATTITUDE && HIL_MODE != HIL_MODE_SENSORS
    {"sonar",               test_sonar},
#endif
};

// A Macro to create the Menu
MENU(test_menu, "test", test_menu_commands);

static int8_t
test_mode(uint8_t argc, const Menu::arg *argv)
{
    test_menu.run();
    return 0;
}

#if HIL_MODE != HIL_MODE_ATTITUDE && HIL_MODE != HIL_MODE_SENSORS
static int8_t
test_baro(uint8_t argc, const Menu::arg *argv)
{
    int32_t alt;
    print_hit_enter();
    init_barometer(true);

    while(1) {
        delay(100);
        alt = read_barometer();

        if (!barometer.healthy) {
            cliSerial->println_P(PSTR("not healthy"));
        } else {
            cliSerial->printf_P(PSTR("Alt: %0.2fm, Raw: %f Temperature: %.1f\n"),
                                alt / 100.0,
                                barometer.get_pressure(), 
                                barometer.get_temperature());
        }
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
    return 0;
}
#endif

static int8_t
test_compass(uint8_t argc, const Menu::arg *argv)
{
    uint8_t delta_ms_fast_loop;
    uint8_t medium_loopCounter = 0;

    if (!g.compass_enabled) {
        cliSerial->printf_P(PSTR("Compass: "));
        print_enabled(false);
        return (0);
    }

    if (!compass.init()) {
        cliSerial->println_P(PSTR("Compass initialisation failed!"));
        return 0;
    }

    ahrs.init();
    ahrs.set_fly_forward(true);
    ahrs.set_compass(&compass);
    report_compass();

    // we need the AHRS initialised for this test
    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate);
    ahrs.reset();
    int16_t counter = 0;
    float heading = 0;

    print_hit_enter();

    while(1) {
        delay(20);
        if (millis() - fast_loopTimer > 19) {
            delta_ms_fast_loop      = millis() - fast_loopTimer;
            G_Dt                    = (float)delta_ms_fast_loop / 1000.f;                       // used by DCM integrator
            fast_loopTimer          = millis();

            // INS
            // ---
            ahrs.update();

            medium_loopCounter++;
            if(medium_loopCounter == 5) {
                if (compass.read()) {
                    // Calculate heading
                    const Matrix3f &m = ahrs.get_dcm_matrix();
                    heading = compass.calculate_heading(m);
                    compass.null_offsets();
                }
                medium_loopCounter = 0;
            }

            counter++;
            if (counter>20) {
                if (compass.healthy()) {
                    const Vector3f &mag_ofs = compass.get_offsets();
                    const Vector3f &mag = compass.get_field();
                    cliSerial->printf_P(PSTR("Heading: %ld, XYZ: %.0f, %.0f, %.0f,\tXYZoff: %6.2f, %6.2f, %6.2f\n"),
                                        (wrap_360_cd(ToDeg(heading) * 100)) /100,
                                        mag.x,
                                        mag.y,
                                        mag.z,
                                        mag_ofs.x,
                                        mag_ofs.y,
                                        mag_ofs.z);
                } else {
                    cliSerial->println_P(PSTR("compass not healthy"));
                }
                counter=0;
            }
        }
        if (cliSerial->available() > 0) {
            break;
        }
    }

    // save offsets. This allows you to get sane offset values using
    // the CLI before you go flying.
    cliSerial->println_P(PSTR("saving offsets"));
    compass.save_offsets();
    return (0);
}

static int8_t
test_gps(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        delay(100);

        g_gps->update();

        if (g_gps->new_data) {
            cliSerial->printf_P(PSTR("Lat: "));
            print_latlon(cliSerial, g_gps->latitude);
            cliSerial->printf_P(PSTR(", Lon "));
            print_latlon(cliSerial, g_gps->longitude);
            cliSerial->printf_P(PSTR(", Alt: %ldm, #sats: %d\n"),
                            g_gps->altitude_cm/100,
                            g_gps->num_sats);
            g_gps->new_data = false;
        }else{
            cliSerial->print_P(PSTR("."));
        }
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
    return 0;
}

static int8_t
test_ins(uint8_t argc, const Menu::arg *argv)
{
    Vector3f gyro, accel;
    print_hit_enter();
    cliSerial->printf_P(PSTR("INS\n"));
    delay(1000);

    ahrs.init();
    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate);
    cliSerial->printf_P(PSTR("...done\n"));

    delay(50);

    while(1) {
        ins.update();
        gyro = ins.get_gyro();
        accel = ins.get_accel();

        float test = accel.length() / GRAVITY_MSS;

        cliSerial->printf_P(PSTR("a %7.4f %7.4f %7.4f g %7.4f %7.4f %7.4f t %7.4f \n"),
            accel.x, accel.y, accel.z,
            gyro.x, gyro.y, gyro.z,
            test);

        delay(40);
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

/*
 *  test the dataflash is working
 */
static int8_t
test_logging(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->println_P(PSTR("Testing dataflash logging"));
    DataFlash.ShowDeviceInfo(cliSerial);
    return 0;
}

static int8_t
test_motors(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf_P(PSTR(
                        "Connect battery for this test.\n"
                        "Motors will spin by frame position order.\n"
                        "Front (& right of centerline) motor first, then in clockwise order around frame.\n"
                        "Remember to disconnect battery after this test.\n"
                        "Any key to exit.\n"));

    // ensure all values have been sent to motors
    motors.set_update_rate(g.rc_speed);
    motors.set_frame_orientation(g.frame_orientation);
    motors.set_min_throttle(g.throttle_min);
    motors.set_mid_throttle(g.throttle_mid);

    // enable motors
    init_rc_out();

    while(1) {
        delay(20);
        read_radio();
        motors.output_test();
        if(cliSerial->available() > 0) {
            g.esc_calibrate.set_and_save(0);
            return(0);
        }
    }
}


// test_motorsync - suddenly increases pwm output to motors to test if ESC loses sync
static int8_t
test_motorsync(uint8_t argc, const Menu::arg *argv)
{
    bool     test_complete = false;
    bool     spin_motors = false;
    uint32_t spin_start_time = 0;
    uint32_t last_run_time;
    int16_t  last_throttle = 0;
    int16_t  c;

    // check if radio is calibration
    pre_arm_rc_checks();
    if (!ap.pre_arm_rc_check) {
        cliSerial->print_P(PSTR("radio not calibrated\n"));
        return 0;
    }

    // print warning that motors will spin
    // ask user to raise throttle
    // inform how to stop test
    cliSerial->print_P(PSTR("This sends sudden outputs to the motors based on the pilot's throttle to test for ESC loss of sync. Motors will spin so mount props up-side-down!\n   Hold throttle low, then raise throttle stick to desired level and press A.  Motors will spin for 2 sec and then return to low.\nPress any key to exit.\n"));

    // clear out user input
    while (cliSerial->available()) {
        cliSerial->read();
    }

    // disable throttle and battery failsafe
    g.failsafe_throttle = FS_THR_DISABLED;
    g.failsafe_battery_enabled = FS_BATT_DISABLED;

    // read radio
    read_radio();

    // exit immediately if throttle is not zero
    if( g.rc_3.control_in != 0 ) {
        cliSerial->print_P(PSTR("throttle not zero\n"));
        return 0;
    }

    // clear out any user input
    while (cliSerial->available()) {
        cliSerial->read();
    }

    // enable motors and pass through throttle
    init_rc_out();
    output_min();
    motors.armed(true);

    // initialise run time
    last_run_time = millis();

    // main run while the test is not complete
    while(!test_complete) {
        // 50hz loop
        if( millis() - last_run_time > 20 ) {
            last_run_time = millis();

            // read radio input
            read_radio();

            // display throttle value
            if (abs(g.rc_3.control_in-last_throttle) > 10) {
                cliSerial->printf_P(PSTR("\nThr:%d"),g.rc_3.control_in);
                last_throttle = g.rc_3.control_in;
            }

            // decode user input
            if (cliSerial->available()) {
                c = cliSerial->read();
                if (c == 'a' || c == 'A') {
                    spin_motors = true;
                    spin_start_time = millis();
                    // display user's throttle level
                    cliSerial->printf_P(PSTR("\nSpin motors at:%d"),(int)g.rc_3.control_in);
                    // clear out any other use input queued up
                    while (cliSerial->available()) {
                        cliSerial->read();
                    }
                }else{
                    // any other input ends the test
                    test_complete = true;
                    motors.armed(false);
                }
            }

            // check if time to stop motors
            if (spin_motors) {
                if ((millis() - spin_start_time) > 2000) {
                    spin_motors = false;
                    cliSerial->printf_P(PSTR("\nMotors stopped"));
                }
            }

            // output to motors
            if (spin_motors) {
                // pass pilot throttle through to motors
                motors.throttle_pass_through();
            }else{
                // spin motors at minimum
                output_min();
            }
        }
    }

    // stop motors
    motors.output_min();
    motors.armed(false);

    // clear out any user input
    while( cliSerial->available() ) {
        cliSerial->read();
    }

    // display completion message
    cliSerial->printf_P(PSTR("\nTest complete\n"));

    return 0;
}

static int8_t
test_optflow(uint8_t argc, const Menu::arg *argv)
{
#if OPTFLOW == ENABLED
    if(g.optflow_enabled) {
        cliSerial->printf_P(PSTR("man id: %d\t"),optflow.read_register(ADNS3080_PRODUCT_ID));
        print_hit_enter();

        while(1) {
            delay(200);
            optflow.update();
            cliSerial->printf_P(PSTR("dx:%d\t dy:%d\t squal:%d\n"),
                            optflow.dx,
                            optflow.dy,
                            optflow.surface_quality);

            if(cliSerial->available() > 0) {
                return (0);
            }
        }
    } else {
        cliSerial->printf_P(PSTR("OptFlow: "));
        print_enabled(false);
    }
    return (0);
#else
    return (0);
#endif      // OPTFLOW == ENABLED
}

static int8_t
test_radio_pwm(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        delay(20);

        // Filters radio input - adjust filters in the radio.pde file
        // ----------------------------------------------------------
        read_radio();

        // servo Yaw
        //APM_RC.OutputCh(CH_7, g.rc_4.radio_out);

        cliSerial->printf_P(PSTR("IN: 1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\t8: %d\n"),
                        g.rc_1.radio_in,
                        g.rc_2.radio_in,
                        g.rc_3.radio_in,
                        g.rc_4.radio_in,
                        g.rc_5.radio_in,
                        g.rc_6.radio_in,
                        g.rc_7.radio_in,
                        g.rc_8.radio_in);

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

static int8_t
test_radio(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        delay(20);
        read_radio();


        cliSerial->printf_P(PSTR("IN  1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\n"),
                        g.rc_1.control_in,
                        g.rc_2.control_in,
                        g.rc_3.control_in,
                        g.rc_4.control_in,
                        g.rc_5.control_in,
                        g.rc_6.control_in,
                        g.rc_7.control_in);

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

static int8_t test_relay(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        cliSerial->printf_P(PSTR("Relay on\n"));
        relay.on(0);
        delay(3000);
        if(cliSerial->available() > 0) {
            return (0);
        }

        cliSerial->printf_P(PSTR("Relay off\n"));
        relay.off(0);
        delay(3000);
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
/*
 *  run a debug shell
 */
static int8_t
test_shell(uint8_t argc, const Menu::arg *argv)
{
    hal.util->run_debug_shell(cliSerial);
    return 0;
}
#endif

#if HIL_MODE != HIL_MODE_ATTITUDE && HIL_MODE != HIL_MODE_SENSORS
/*
 *  test the sonar
 */
static int8_t
test_sonar(uint8_t argc, const Menu::arg *argv)
{
#if CONFIG_SONAR == ENABLED
    if(g.sonar_enabled == false) {
        cliSerial->printf_P(PSTR("Sonar disabled\n"));
        return (0);
    }

    // make sure sonar is initialised
    init_sonar();

    print_hit_enter();
    while(1) {
        delay(100);

        cliSerial->printf_P(PSTR("Sonar: %d cm\n"), sonar->read());

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
#endif
    return (0);
}
#endif

static void print_hit_enter()
{
    cliSerial->printf_P(PSTR("Hit Enter to exit.\n\n"));
}

#endif // CLI_ENABLED
