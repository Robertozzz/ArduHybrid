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
	
	// @Param: hybridswitching_radio_in
    // @DisplayName: Hybrid switching radio in channel
    // @Description: Radio in channel for switching hybrid mode.
    // @Values:1-8
    // @User: Standard
    GSCALAR(hybridswitching_radio_in, "HYBSWITCHIN",               HYBRIDSWITCHING_IN),

	// @Param: hybridswitching_radio_out
    // @DisplayName: Hybrid switching radio out channel
    // @Description: Servo out channel for switching hybrid mode.
    // @Values: Tri/Quad: 5-13  Y6/HEX: 7-13  Octa: 9-13
    // @User: Standard
    GSCALAR(hybridswitching_radio_out, "HYBSWITCHIN",              HYBRIDSWITCHING_OUT),

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
