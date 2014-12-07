// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//Function that will read the radio data, limit servos and trigger a failsafe
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
    // update assigned functions for auxiliary servos
    aux_servos_update_fn();

    // set default dead zones
    default_dead_zones();
}

/*
  initialise RC output channels
 */
static void init_rc_out()
{
    channel_roll->enable_out();
    channel_pitch->enable_out();
    if (arming.arming_required() != AP_Arming::YES_ZERO_PWM) {
        channel_throttle->enable_out();
    }
    channel_rudder->enable_out();
    enable_aux_servos();

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
                Log_Arm_Disarm();
            }                
        }
    } else { 
        // not at full right rudder
        rudder_arm_timer = 0;
    }
}

static void read_radio()
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
    
    if (control_mode == TRAINING) {
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
    for (uint8_t y = 0; y < 30; y++) {
        read_radio();
    }

    trim_control_surfaces();
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
