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

