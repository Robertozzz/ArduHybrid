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
    case STABILIZE:
        port->print_P(PSTR("STABILIZE"));
        break;
    case ACRO:
        port->print_P(PSTR("ACRO"));
        break;
    case ALT_HOLD:
        port->print_P(PSTR("ALT_HOLD"));
        break;
    case AUTO:
        port->print_P(PSTR("AUTO"));
        break;
    case GUIDED:
        port->print_P(PSTR("GUIDED"));
        break;
    case LOITER:
        port->print_P(PSTR("LOITER"));
        break;
    case RTL:
        port->print_P(PSTR("RTL"));
        break;
    case CIRCLE:
        port->print_P(PSTR("CIRCLE"));
        break;
    case POSITION:
        port->print_P(PSTR("POSITION"));
        break;
    case LAND:
        port->print_P(PSTR("LAND"));
        break;
    case OF_LOITER:
        port->print_P(PSTR("OF_LOITER"));
        break;
#if DRIFT == ENABLED
    case DRIFT:
        port->print_P(PSTR("DRIFT"));
        break;
#endif
    case SPORT:
        port->print_P(PSTR("SPORT"));
        break;
    case PLANE_MANUAL:
        port->print_P(PSTR("Manual"));
        break;
    case PLANE_STABILIZE:
        port->print_P(PSTR("PLANE_STABILIZE"));
        break;
    case PLANE_TRAINING:
        port->print_P(PSTR("Training"));
        break;
    case PLANE_ACRO:
        port->print_P(PSTR("PLANE_ACRO"));
        break;
    case PLANE_FLY_BY_WIRE_A:
        port->print_P(PSTR("FBW_A"));
        break;
    case PLANE_FLY_BY_WIRE_B:
        port->print_P(PSTR("FBW_B"));
        break;
    case PLANE_CRUISE:
        port->print_P(PSTR("PLANE_CRUISE"));
        break;
    case PLANE_AUTO:
        port->print_P(PSTR("PLANE_AUTO"));
        break;
    case PLANE_LOITER:
        port->print_P(PSTR("PLANE_LOITER"));
        break;
    case PLANE_RTL:
        port->print_P(PSTR("PLANE_RTL"));
        break;
    case PLANE_CIRCLE:
        port->print_P(PSTR("PLANE_CIRCLE"));
        break;
    default:
        port->printf_P(PSTR("Mode(%u)"), (unsigned)mode);
        break;
    }
}