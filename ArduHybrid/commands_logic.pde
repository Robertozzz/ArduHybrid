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
