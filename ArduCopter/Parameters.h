// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <AP_Common.h>

// Global parameter class.
//
class Parameters {
public:
    // The version of the layout as described by the parameter enum.
    //
    // When changing the parameter enum in an incompatible fashion, this
    // value should be incremented by one.
    //
    // The increment will prevent old parameters from being used incorrectly
    // by newer code.
	static const uint16_t		k_format_version = 120;

    // The parameter software_type is set up solely for ground station use
    // and identifies the software type (eg ArduPilotMega versus
    // ArduCopterMega)
    // GCS will interpret values 0-9 as ArduPilotMega.  Developers may use
    // values within that range to identify different branches.
    //
	static const uint16_t		k_software_type = 10;          // 0 for APM trunk
                                                                
    // Parameter identities.
    //
    // The enumeration defined here is used to ensure that every parameter
    // or parameter group has a unique ID number.	This number is used by
    // AP_Param to store and locate parameters in EEPROM.
    //
    // Note that entries without a number are assigned the next number after
    // the entry preceding them.	When adding new entries, ensure that they
    // don't overlap.
    //
    // Try to group related variables together, and assign them a set
    // range in the enumeration.	Place these groups in numerical order
    // at the end of the enumeration.
    //
    // WARNING: Care should be taken when editing this enumeration as the
    //			AP_Param load/save code depends on the values here to identify
    //			variables saved in EEPROM.
    //
    //
    enum {


















































	// Layout version number, always key zero.
        k_param_format_version = 0,
        k_param_software_type,
        k_param_ins,				// libraries/AP_InertialSensor variables
		k_param_num_resets,						// Plane
        // simulation
        k_param_sitl = 5,
        // EPM object
        k_param_epm = 7,

        // Misc
		k_param_relay = 10,
        k_param_scheduler,
		k_param_log_bitmask,
        k_param_BoardConfig,
        k_param_rssi_pin,
		k_param_battery,
		k_param_fs_batt_mah,
        k_param_fs_batt_voltage,
		k_param_rssi_range,
		k_param_wp_yaw_behavior,
        k_param_acro_trainer,
        k_param_pilot_velocity_z_max,
        k_param_circle_rate,
        k_param_sonar_gain,
        k_param_ch8_option,
        k_param_arming_check,
        k_param_sprayer,
        k_param_angle_max,
        k_param_gps_hdop_good,
        k_param_angle_rate_max,
        k_param_rc_feel_rp,
		k_param_auto_trim,						// Plane
        k_param_pitch_trim_cd,					// Plane
        k_param_mix_mode,						// Plane
        k_param_reverse_elevons,				// Plane
        k_param_reverse_ch1_elevon,				// Plane
        k_param_reverse_ch2_elevon,				// Plane
        k_param_flap_1_percent,					// Plane
        k_param_flap_1_speed,					// Plane
        k_param_flap_2_percent,					// Plane
        k_param_flap_2_speed,					// Plane
        k_param_reset_switch_chan,				// Plane
        k_param_land_pitch_cd,					// Plane
        k_param_stick_mixing,					// Plane
        k_param_reset_mission_chan,				// Plane
        k_param_land_flare_alt,					// Plane
        k_param_land_flare_sec,					// Plane
        k_param_throttle_nudge,					// Plane
        k_param_alt_offset,						// Plane
        k_param_takeoff_throttle_min_speed,		// Plane
        k_param_takeoff_throttle_min_accel,		// Plane
        k_param_level_roll_limit,				// Plane
        k_param_hil_servos,						// Plane
        k_param_vtail_output,					// Plane
        k_param_nav_controller,					// Plane
        k_param_elevon_output,					// Plane
        k_param_att_controller,					// Plane
        k_param_mixing_gain,					// Plane
        k_param_takeoff_throttle_delay,			// Plane
        k_param_skip_gyro_cal,					// Plane
        k_param_auto_fbw_steer,					// Plane
        k_param_waypoint_max_radius,			// Plane
        k_param_ground_steer_alt,        		// Plane
        k_param_ground_steer_dps,				// Plane
        k_param_rally_limit_km,					// Plane
        k_param_hil_err_limit,					// Plane
        k_param_sonar,							// Plane // 66


        // 70: AP_Limits Library
        k_param_fence = 70,
        k_param_gps_glitch,
        // 73: Singlecopter
        k_param_single_servo_1 = 73,
        k_param_single_servo_2,
        k_param_single_servo_3,
        k_param_single_servo_4,
        // 78: Heli
        k_param_heli_servo_1 = 78,
        k_param_heli_servo_2,
        k_param_heli_servo_3,
        k_param_heli_servo_4,
        k_param_heli_pitch_ff,
        k_param_heli_roll_ff,
        k_param_heli_yaw_ff,
        k_param_heli_stab_col_min,
        k_param_heli_stab_col_max,
        // 87: Motors
        k_param_motors = 87,
        // 90: Inertial Nav
        k_param_inertial_nav = 90,
        k_param_wp_nav,
        // 93: Arming parameters
        k_param_arming = 93,							// Plane
        // 95: Extra parameters
        k_param_fence_retalt = 95,					// Plane
        // 97: Telemetry control
        k_param_gcs0 = 97,
        k_param_gcs1,
        k_param_sysid_this_mav,
        k_param_sysid_my_gcs,
        k_param_serial1_baud,
        k_param_telem_delay,
        k_param_gcs2,
        k_param_serial0_baud,					// Plane
        k_param_serial2_baud,
        // 107: Fly-by-wire control
        k_param_airspeed_min = 107,				// Plane
        k_param_airspeed_max,					// Plane
        k_param_FBWB_min_altitude_cm,			// Plane // 0=disabled, minimum value for altitude in cm (for first time try 30 meters = 3000 cm)
        k_param_flybywire_elev_reverse,			// Plane
        k_param_alt_control_algorithm,			// Plane
        k_param_flybywire_climb_rate,			// Plane
        k_param_acro_roll_ratep,				// Plane
        k_param_acro_pitch_ratep,				// Plane
        k_param_acro_locking,					// Plane

        // 118: Sensor parameters
        k_param_altitude_mix = 118,				// Plane
        k_param_compass_enabled,
        k_param_compass,
        k_param_sonar_enabled,
        k_param_frame_orientation,
        k_param_optflow_enabled,
        k_param_ch7_option,
        k_param_sonar_type,
        k_param_super_simple,
        k_param_ahrs, // AHRS group
        k_param_barometer,   // barometer ground calibration
        k_param_airspeed,  						// Plane  // AP_Airspeed parameters
        k_param_curr_amp_offset,				// Plane

        // 133: Navigation parameters
        k_param_rtl_altitude = 133,
        k_param_rtl_loiter_time,
        k_param_rtl_alt_final,
		k_param_roll_limit_cd,					// Plane
        k_param_pitch_limit_max_cd,				// Plane
        k_param_pitch_limit_min_cd,				// Plane
        k_param_airspeed_cruise_cm,				// Plane
        k_param_RTL_altitude_cm,				// Plane
        k_param_inverted_flight_ch,				// Plane
        k_param_min_gndspeed_cm,				// Plane

        // 145: Camera and mount parameters
        k_param_camera = 145,
        k_param_camera_mount,
        k_param_camera_mount2,

        // 150: Radio settings
        k_param_rc_1 = 150,
        k_param_rc_2,
        k_param_rc_3,
        k_param_rc_4,
        k_param_rc_5,
        k_param_rc_6,
        k_param_rc_7,
        k_param_rc_8,
        k_param_rc_9,
        k_param_rc_10,
        k_param_rc_11,
        k_param_rc_12,
        k_param_plthr_min,						// Plane
        k_param_plthr_max,						// Plane
        k_param_throttle_min,
        k_param_throttle_max,
        k_param_failsafe_throttle,
        k_param_failsafe_throttle_value,
        k_param_throttle_fs_enabled,			// Plane
        k_param_throttle_fs_value,				// Plane
        k_param_plthr_cruise,					// Plane
        k_param_throttle_cruise,
        k_param_short_fs_action,				// Plane
        k_param_long_fs_action,					// Plane
        k_param_gcs_heartbeat_fs_enabled,		// Plane
        k_param_throttle_slewrate,				// Plane
        k_param_throttle_suppress_manual,		// Plane
        k_param_throttle_passthru_stabilize,	// Plane
        k_param_short_fs_timeout,				// Plane
        k_param_long_fs_timeout,				// Plane
        k_param_esc_calibrate,
        k_param_radio_tuning,
        k_param_radio_tuning_high,
        k_param_radio_tuning_low,
        k_param_rc_speed,
        k_param_failsafe_battery_enabled,
        k_param_throttle_mid,
        k_param_failsafe_gps_enabled,
        k_param_failsafe_gcs, ////////duplicates with plane???!!!!!!!!!!!!!!!!!!!!!!!!!
        k_param_rcmap,

		
		
		
        // 195: Feed-forward gains
        k_param_kff_rudder_mix = 195,			// Plane
        k_param_kff_throttle_to_pitch,			// Plane
        k_param_scaling_speed,					// Plane

        // 200: flight modes
        k_param_flight_mode_channel = 200,		// Plane
        k_param_flight_mode1,
        k_param_flight_mode2,
        k_param_flight_mode3,
        k_param_flight_mode4,
        k_param_flight_mode5,
        k_param_flight_mode6,
        k_param_simple_modes,

        // 210: Waypoint data
        k_param_waypoint_mode = 210,			// Plane
        k_param_command_total,
        k_param_command_index,
        k_param_waypoint_radius,				// Plane
        k_param_loiter_radius,					// Plane
        k_param_fence_action,					// Plane
        k_param_fence_total,					// Plane
        k_param_fence_channel,					// Plane
        k_param_fence_minalt,					// Plane
        k_param_fence_maxalt,					// Plane
        k_param_circle_radius,
        k_param_land_speed,

		
        // 225: PI/D Controllers
        k_param_acro_rp_p = 225,
        k_param_pid_rate_roll,
        k_param_pid_rate_pitch,
        k_param_pid_rate_yaw,
        k_param_pi_stabilize_roll,
        k_param_pi_stabilize_pitch,
        k_param_pi_stabilize_yaw,
        k_param_pi_loiter_lat,
        k_param_pi_loiter_lon,
        k_param_pid_loiter_rate_lat,
        k_param_pid_loiter_rate_lon,
        k_param_pi_alt_hold,
        k_param_pid_throttle_rate,
        k_param_pid_optflow_roll,
        k_param_pid_optflow_pitch,
        k_param_pid_throttle_accel,
        k_param_acro_balance_roll,
        k_param_acro_balance_pitch,
        k_param_acro_yaw_p,
        // 245: other objects
        k_param_obc = 245,						// Plane
        k_param_rollController,					// Plane
        k_param_pitchController,				// Plane
        k_param_yawController,					// Plane
        k_param_L1_controller,					// Plane
        k_param_TECS_controller,				// Plane
        k_param_rally_total,					// Plane
        k_param_steerController,				// Plane
        // 254,255: reserved
    };

    AP_Int16		format_version;
    AP_Int8			software_type;

    // Telemetry control
    AP_Int16        sysid_this_mav;
    AP_Int16        sysid_my_gcs;
	AP_Int8			serial0_baud;				// Plane
	AP_Int8			serial1_baud;
#if MAVLINK_COMM_NUM_BUFFERS > 2
	AP_Int8			serial2_baud;
#endif
	AP_Int8			telem_delay;

    AP_Int16        rtl_altitude;
    AP_Int8         sonar_enabled;
    AP_Int8         sonar_type;       // 0 = XL, 1 = LV,
                                      // 2 = XLL (XL with 10m range)
                                      // 3 = HRLV
    AP_Float        sonar_gain;
    AP_Int8         failsafe_battery_enabled;   // battery failsafe enabled
    AP_Float        fs_batt_voltage;            // battery voltage below which failsafe will be triggered
    AP_Float        fs_batt_mah;                // battery capacity (in mah) below which failsafe will be triggered
    AP_Int8         failsafe_gps_enabled;       // gps failsafe enabled
    AP_Int8         failsafe_gcs;               // ground station failsafe behavior
    AP_Int16        gps_hdop_good;              // GPS Hdop value at or below this value represent a good position
    AP_Int8         compass_enabled;
    AP_Int8         optflow_enabled;
    AP_Int8         super_simple;
    AP_Int16        rtl_alt_final;
    AP_Int8         rssi_pin;
    AP_Float        rssi_range;                 // allows to set max voltage for rssi pin such as 5.0, 3.3 etc. 
    AP_Int8         wp_yaw_behavior;            // controls how the autopilot controls yaw during missions
    AP_Int16        angle_max;                  // maximum lean angle of the copter in centi-degrees
    AP_Int32        angle_rate_max;             // maximum rotation rate in roll/pitch axis requested by angle controller used in stabilize, loiter, rtl, auto flight modes
    AP_Int8         rc_feel_rp;                 // controls vehicle response to user input with 0 being extremely soft and 100 begin extremely crisp

    // Waypoints
    AP_Int8 		waypoint_mode;			// Plane
    AP_Int8         command_total;
    AP_Int8         command_index;
    AP_Int16		waypoint_radius;		// Plane
    AP_Int16 		waypoint_max_radius;			// Plane
    AP_Int16 		loiter_radius;					// Plane

    AP_Int16        circle_radius;
    AP_Float        circle_rate;                // Circle mode's turn rate in deg/s.  positive to rotate clockwise, negative for counter clockwise
    AP_Int32        rtl_loiter_time;
    AP_Int16        land_speed;
    AP_Int16        pilot_velocity_z_max;        // maximum vertical velocity the pilot may request

#if GEOFENCE_ENABLED == ENABLED
    AP_Int8 fence_action;					// Plane
    AP_Int8 fence_total;					// Plane
    AP_Int8 fence_channel;					// Plane
    AP_Int16 fence_minalt;					// Plane	// meters
    AP_Int16 fence_maxalt;					// Plane	// meters
    AP_Int16 fence_retalt;					// Plane	// meters
#endif

    AP_Int8 rally_total;					// Plane
    AP_Float rally_limit_km;				// Plane
#if HIL_MODE != HIL_MODE_DISABLED
    AP_Float hil_err_limit;					// Plane
#endif

    // Feed-forward gains
    AP_Float kff_rudder_mix;				// Plane
    AP_Float kff_pitch_to_throttle;			// Plane
    AP_Float kff_throttle_to_pitch;			// Plane
    AP_Float ground_steer_alt;				// Plane
    AP_Int16 ground_steer_dps;				// Plane

    // speed used for speed scaling
    AP_Float scaling_speed;					// Plane

    // navigation controller type. See AP_Navigation::ControllerType
    AP_Int8  nav_controller;				// Plane

    // attitude controller type.
    AP_Int8  att_controller;				// Plane

    // skip gyro calibration
    AP_Int8  skip_gyro_cal;					// Plane
    AP_Int8  auto_fbw_steer;				// Plane

    // Estimation
    AP_Float altitude_mix;					// Plane
    AP_Int8  alt_control_algorithm;			// Plane

    // Fly-by-wire
    AP_Int8 flybywire_elev_reverse;			// Plane
    AP_Int8 flybywire_climb_rate;			// Plane


    // Throttle
    AP_Int16        throttle_min;
    AP_Int16        throttle_max;
    AP_Int8         failsafe_throttle;
    AP_Int16        failsafe_throttle_value;
    AP_Int16        throttle_cruise;
    AP_Int16        throttle_mid;
    AP_Int8 throttle_suppress_manual;		// Plane
    AP_Int8 throttle_passthru_stabilize;	// Plane
    AP_Int8 throttle_fs_enabled;			// Plane
    AP_Int16 throttle_fs_value;				// Plane
    AP_Int8 throttle_nudge;					// Plane

    // Failsafe
    AP_Int8 short_fs_action;				// Plane
    AP_Int8 long_fs_action;					// Plane
    AP_Float short_fs_timeout;				// Plane
    AP_Float long_fs_timeout;				// Plane
    AP_Int8 gcs_heartbeat_fs_enabled;		// Plane

    // Flight modes
    AP_Int8 		flight_mode_channel;	// Plane
    AP_Int8         flight_mode1;
    AP_Int8         flight_mode2;
    AP_Int8         flight_mode3;
    AP_Int8         flight_mode4;
    AP_Int8         flight_mode5;
    AP_Int8         flight_mode6;
    AP_Int8         simple_modes;

    // Misc
    AP_Int16		log_bitmask;
    AP_Int8         esc_calibrate;
    AP_Int8         radio_tuning;
    AP_Int16        radio_tuning_high;
    AP_Int16        radio_tuning_low;
    AP_Int8         frame_orientation;
    AP_Int8         ch7_option;
    AP_Int8         ch8_option;
    AP_Int8         arming_check;
    AP_Int8 auto_trim;						// Plane
    AP_Int8 mix_mode;						// Plane
    AP_Int8 vtail_output;					// Plane
    AP_Int8 elevon_output;					// Plane
    AP_Float mixing_gain;					// Plane
    AP_Int8 reverse_elevons;				// Plane
    AP_Int8 reverse_ch1_elevon;				// Plane
    AP_Int8 reverse_ch2_elevon;				// Plane
    AP_Int16 num_resets;					// Plane
    AP_Int8 reset_switch_chan;				// Plane
    AP_Int8 reset_mission_chan;				// Plane
    AP_Int32 airspeed_cruise_cm;			// Plane
    AP_Int32 RTL_altitude_cm;				// Plane
    AP_Int16 land_pitch_cd;					// Plane
    AP_Float land_flare_alt;				// Plane
    AP_Float land_flare_sec;				// Plane
    AP_Int32 min_gndspeed_cm;				// Plane
    AP_Int16 pitch_trim_cd;					// Plane
    AP_Int16 FBWB_min_altitude_cm;			// Plane
#if HIL_MODE != HIL_MODE_DISABLED
    AP_Int8  hil_servos;					// Plane
#endif
    AP_Int8 flap_1_percent;					// Plane
    AP_Int8 flap_1_speed;					// Plane
    AP_Int8 flap_2_percent;					// Plane
    AP_Int8 flap_2_speed;					// Plane
    AP_Int8 inverted_flight_ch;				// Plane             // 0=disabled, 1-8 is channel for inverted flight trigger
    AP_Int8 stick_mixing;					// Plane
    AP_Float takeoff_throttle_min_speed;	// Plane
    AP_Float takeoff_throttle_min_accel;	// Plane
    AP_Int8 takeoff_throttle_delay;			// Plane
    AP_Int8 level_roll_limit;				// Plane

    // Navigational maneuvering limits
    AP_Int16 roll_limit_cd;					// Plane
    AP_Int16 alt_offset;					// Plane
    AP_Int16 acro_roll_ratep;				// Plane
    AP_Int16 acro_pitch_ratep;				// Plane
    AP_Int8  acro_locking;					// Plane

#if FRAME_CONFIG ==     HELI_FRAME
    // Heli
    RC_Channel      heli_servo_1, heli_servo_2, heli_servo_3, heli_servo_4;     // servos for swash plate and tail
    AP_Float        heli_pitch_ff;												// pitch rate feed-forward
    AP_Float        heli_roll_ff;												// roll rate feed-forward
    AP_Float        heli_yaw_ff;												// yaw rate feed-forward
    AP_Int16        heli_stab_col_min;                                          // min collective while pilot directly controls collective in stabilize mode
    AP_Int16        heli_stab_col_max;                                          // min collective while pilot directly controls collective in stabilize mode
#endif
#if FRAME_CONFIG ==     SINGLE_FRAME
    // Single
    RC_Channel      single_servo_1, single_servo_2, single_servo_3, single_servo_4;     // servos for four flaps
#endif

    // RC channels
    RC_Channel              rc_1;
    RC_Channel              rc_2;
    RC_Channel              rc_3;
    RC_Channel              rc_4;
    RC_Channel_aux          rc_5;
    RC_Channel_aux          rc_6;
    RC_Channel_aux          rc_7;
    RC_Channel_aux          rc_8;
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    RC_Channel_aux          rc_9;
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2 || CONFIG_HAL_BOARD == HAL_BOARD_PX4
    RC_Channel_aux          rc_10;
    RC_Channel_aux          rc_11;
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    RC_Channel_aux          rc_12;
#endif

    AP_Int16                rc_speed; // speed of fast RC Channels in Hz

    // Acro parameters
    AP_Float                acro_rp_p;
    AP_Float                acro_yaw_p;
    AP_Float                acro_balance_roll;
    AP_Float                acro_balance_pitch;
    AP_Int8                 acro_trainer;

    // PI/D controllers
    AC_PID                  pid_rate_roll;
    AC_PID                  pid_rate_pitch;
    AC_PID                  pid_rate_yaw;
    AC_PID                  pid_loiter_rate_lat;
    AC_PID                  pid_loiter_rate_lon;

    AC_PID                  pid_throttle_rate;
    AC_PID                  pid_throttle_accel;
    AC_PID                  pid_optflow_roll;
    AC_PID                  pid_optflow_pitch;

    APM_PI                  pi_loiter_lat;
    APM_PI                  pi_loiter_lon;
    APM_PI                  pi_stabilize_roll;
    APM_PI                  pi_stabilize_pitch;
    APM_PI                  pi_stabilize_yaw;
    APM_PI                  pi_alt_hold;

    // Note: keep initializers here in the same order as they are declared above.
    Parameters() :

#if FRAME_CONFIG ==     HELI_FRAME
        heli_servo_1        (CH_1),
        heli_servo_2        (CH_2),
        heli_servo_3        (CH_3),
        heli_servo_4        (CH_4),
#endif
#if FRAME_CONFIG ==     SINGLE_FRAME
        single_servo_1        (CH_1),
        single_servo_2        (CH_2),
        single_servo_3        (CH_3),
        single_servo_4        (CH_4),
#endif

        // variable				default
        //----------------------------------------
        rc_1                (CH_1),
        rc_2                (CH_2),
        rc_3                (CH_3),
        rc_4                (CH_4),
        rc_5                (CH_5),
        rc_6                (CH_6),
        rc_7                (CH_7),
        rc_8                (CH_8),
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
        rc_9                (CH_9),
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2 || CONFIG_HAL_BOARD == HAL_BOARD_PX4
        rc_10               (CH_10),
        rc_11               (CH_11),
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
        rc_12               (CH_12),
#endif

        // PID controller	initial P	        initial I		    initial D
        //          initial imax
        //-----------------------------------------------------------------------------------------------------
        pid_rate_roll           (RATE_ROLL_P,           RATE_ROLL_I,            RATE_ROLL_D,            RATE_ROLL_IMAX),
        pid_rate_pitch          (RATE_PITCH_P,          RATE_PITCH_I,           RATE_PITCH_D,           RATE_PITCH_IMAX),
        pid_rate_yaw            (RATE_YAW_P,            RATE_YAW_I,             RATE_YAW_D,             RATE_YAW_IMAX),

        pid_loiter_rate_lat     (LOITER_RATE_P,         LOITER_RATE_I,          LOITER_RATE_D,          LOITER_RATE_IMAX),
        pid_loiter_rate_lon     (LOITER_RATE_P,         LOITER_RATE_I,          LOITER_RATE_D,          LOITER_RATE_IMAX),

        pid_throttle_rate       (THROTTLE_RATE_P,       THROTTLE_RATE_I,        THROTTLE_RATE_D,        THROTTLE_RATE_IMAX),
        pid_throttle_accel      (THROTTLE_ACCEL_P,      THROTTLE_ACCEL_I,       THROTTLE_ACCEL_D,       THROTTLE_ACCEL_IMAX),
        pid_optflow_roll        (OPTFLOW_ROLL_P,        OPTFLOW_ROLL_I,         OPTFLOW_ROLL_D,         OPTFLOW_IMAX),
        pid_optflow_pitch       (OPTFLOW_PITCH_P,       OPTFLOW_PITCH_I,        OPTFLOW_PITCH_D,        OPTFLOW_IMAX),

        // PI controller	initial P			initial I			initial
        // imax
        //----------------------------------------------------------------------
        pi_loiter_lat           (LOITER_P,              LOITER_I,               LOITER_IMAX),
        pi_loiter_lon           (LOITER_P,              LOITER_I,               LOITER_IMAX),

        pi_stabilize_roll       (STABILIZE_ROLL_P,      STABILIZE_ROLL_I,       STABILIZE_ROLL_IMAX),
        pi_stabilize_pitch      (STABILIZE_PITCH_P,     STABILIZE_PITCH_I,      STABILIZE_PITCH_IMAX),
        pi_stabilize_yaw        (STABILIZE_YAW_P,       STABILIZE_YAW_I,        STABILIZE_YAW_IMAX),

        pi_alt_hold             (ALT_HOLD_P,            ALT_HOLD_I,             ALT_HOLD_IMAX)
    {
    }
};

extern const AP_Param::Info		var_info[];

#endif // PARAMETERS_H