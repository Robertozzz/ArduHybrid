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
                Log_Arm_Disarm();
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
    read_radio();
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
        //We cannot use radio_in[CH_ROLL] and radio_in[CH_PITCH] values from read_radio() because the elevon trim values have changed
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
        read_radio();
    }

    g.rc_1.trim();      // roll
    g.rc_2.trim();      // pitch
    g.rc_4.trim();      // yaw

    g.rc_1.save_eeprom();
    g.rc_2.save_eeprom();
    g.rc_4.save_eeprom();
    trim_control_surfaces();
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
