// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


static void failsafe_short_on_event(enum failsafe_state fstype)
{
    // This is how to handle a short loss of control signal plane_failsafe.
    plane_failsafe.state = fstype;
    plane_failsafe.ch3_timer_ms = millis();
    gcs_send_text_P(SEVERITY_LOW, PSTR("Failsafe - Short event on, "));
    switch(plane_control_mode)
    {
    case MANUAL:
    case STABILIZE:
    case ACRO:
    case FLY_BY_WIRE_A:
    case FLY_BY_WIRE_B:
    case CRUISE:
    case TRAINING:
        plane_failsafe.saved_mode = plane_control_mode;
        plane_failsafe.saved_mode_set = 1;
        if(g.short_fs_action == 2) {
            plane_set_mode(FLY_BY_WIRE_A);
        } else {
            plane_set_mode(CIRCLE);
        }
        break;

    case AUTO:
    case GUIDED:
    case LOITER:
        if(g.short_fs_action != 0) {
            plane_failsafe.saved_mode = plane_control_mode;
            plane_failsafe.saved_mode_set = 1;
            if(g.short_fs_action == 2) {
                plane_set_mode(FLY_BY_WIRE_A);
            } else {
                plane_set_mode(CIRCLE);
            }
        }
        break;

    case CIRCLE:
    case RTL:
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
    case MANUAL:
    case STABILIZE:
    case ACRO:
    case FLY_BY_WIRE_A:
    case FLY_BY_WIRE_B:
    case CRUISE:
    case TRAINING:
    case CIRCLE:
        if(g.long_fs_action == 2) {
            plane_set_mode(FLY_BY_WIRE_A);
        } else {
            plane_set_mode(RTL);
        }
        break;

    case AUTO:
    case GUIDED:
    case LOITER:
        if(g.long_fs_action == 2) {
            plane_set_mode(FLY_BY_WIRE_A);
        } else if (g.long_fs_action == 1) {
            plane_set_mode(RTL);
        }
        break;

    case RTL:
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
    if (plane_control_mode == CIRCLE && plane_failsafe.saved_mode_set) {
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
    plane_set_mode(RTL);
    aparm.plthr_cruise.load();
    plane_failsafe.low_battery = true;
    AP_Notify::flags.failsafe_battery = true;
}

static void update_events(void)
{
    ServoRelayEvents.update_events();
}
