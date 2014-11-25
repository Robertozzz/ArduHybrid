/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  write to a servo
 */
static void servo_write(uint8_t ch, uint16_t pwm)
{
#if HIL_MODE != HIL_MODE_DISABLED
    if (!g.hil_servos) {
        if (ch < 8) {
            RC_Channel::rc_channel(ch)->radio_out = pwm;
        }
        return;
    }
#endif
    hal.rcout->enable_ch(ch);
    hal.rcout->write(ch, pwm);
}