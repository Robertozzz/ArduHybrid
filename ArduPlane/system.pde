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
    // disable the failsafe code in the CLI
    hal.scheduler->register_timer_failsafe(NULL,1);

    // disable the mavlink delay callback
    hal.scheduler->register_delay_callback(NULL, 5);

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
	
    set_control_channels();

    relay.init();

    // init EPM cargo gripper
#if EPM_ENABLED == ENABLED
    epm.init();
#endif

    // initialise notify system
    notify.init(false);

    // initialise battery monitor
    battery.init();

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
    hal.scheduler->register_delay_callback(plane_mavlink_delay_cb, 5);

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
    mavlink_system.type = MAV_TYPE_FIXED_WING;
    mavlink_system.compid = 1;          //MAV_COMP_ID_IMU;   // We do not check for comp id

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
    g_gps->init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_4G);

    if(g.compass_enabled)
        init_compass();
		
    // initialise airspeed sensor
    airspeed.init();

    // give AHRS the airspeed sensor
    ahrs.set_airspeed(&airspeed);

#if FENCE_TRIGGERED_PIN > 0
    hal.gpio->pinMode(FENCE_TRIGGERED_PIN, OUTPUT);
    digitalWrite(FENCE_TRIGGERED_PIN, LOW);
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

    // initialize commands
    // -------------------
    init_commands();

    reset_control_switch();
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
	
    plane_set_mode(MANUAL);

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
    ahrs.set_fly_forward(true);
    ahrs.set_wind_estimation(true);
	
    // Warm up and read Gyro offsets
    // -----------------------------
    ins.init(AP_InertialSensor::COLD_START,ins_sample_rate);

	
 // ahrs.reset();  // necessary??
	  
	  
    if (airspeed.enabled()) {
        // initialize airspeed sensor
        // --------------------------
        zero_airspeed();
    } else {
        gcs_send_text_P(SEVERITY_LOW,PSTR("NO airspeed"));
    }
}

// plane_set_mode - change flight mode and perform any necessary initialisation
// optional force parameter used to force the flight mode change (used only first time mode is set)
// returns true if mode was succesfully set
// ACRO, STABILIZE, ALTHOLD, LAND, DRIFT and SPORT can always be set successfully but the return state of other flight modes should be checked and the caller should deal with failures appropriately
static void plane_set_mode(enum FlightMode mode)
{
    if(control_mode == mode) {
        // don't switch modes if we are already in the correct mode.
        return;
    }
    if(g.auto_trim > 0 && control_mode == MANUAL)
        trim_control_surfaces();

    control_mode = mode;

    switch(control_mode)
    {
    case INITIALISING:
    case MANUAL:
    case STABILIZE:
    case TRAINING:
    case FLY_BY_WIRE_A:
        break;

    case ACRO:
        acro_state.locked_roll = false;
        acro_state.locked_pitch = false;
        break;

    case CRUISE:
        cruise_state.locked_heading = false;
        cruise_state.lock_timer_ms = 0;
        target_altitude_cm = current_loc.alt;
        break;

    case FLY_BY_WIRE_B:
        target_altitude_cm = current_loc.alt;
        break;

    case CIRCLE:
        // the altitude to circle at is taken from the current altitude
        next_WP.alt = current_loc.alt;
        break;

    case AUTO:
        prev_WP = current_loc;
        update_auto();
        break;

    case RTL:
        prev_WP = current_loc;
        plane_do_RTL();
        break;

    case LOITER:
        do_loiter_at_location();
        break;

    case GUIDED:
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
    if (control_mode == CIRCLE || control_mode >= FLY_BY_WIRE_B) {
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

/*
  should we log a message type now?
 */
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
    case MANUAL:
        port->print_P(PSTR("Manual"));
        break;
    case STABILIZE:
        port->print_P(PSTR("STABILIZE"));
        break;
    case TRAINING:
        port->print_P(PSTR("Training"));
        break;
    case ACRO:
        port->print_P(PSTR("ACRO"));
        break;
    case FLY_BY_WIRE_A:
        port->print_P(PSTR("FBW_A"));
        break;
    case FLY_BY_WIRE_B:
        port->print_P(PSTR("FBW_B"));
        break;
    case CRUISE:
        port->print_P(PSTR("CRUISE"));
        break;
    case AUTO:
        port->print_P(PSTR("AUTO"));
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
    default:
        port->printf_P(PSTR("Mode(%u)"), (unsigned)mode);
        break;
    }
}