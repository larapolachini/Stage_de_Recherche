/* TODO
 */

#include "pogobase.h"
#include "math.h"
#include "util.h"
#include "colors.h"
#include "ssr.h"
#include <stdlib.h>

static void transition_run_tumble(uint32_t);

/* ---------- run-and-tumble global parameters ---------- */
uint32_t run_duration_min = 200; 
uint32_t run_duration_max = 1200;
uint32_t tumble_duration_min = 100;
uint32_t tumble_duration_max = 1100;
uint8_t enable_backward_dir = 0;
float    test_vect[4]        = {1.f, 2.f, 3.f, 4.f};
/* ------------------------------------------------------ */


// Don't forget to call this macro in the main .c file of your project (only once!)
REGISTER_USERDATA(USERDATA);
// Now, members of the USERDATA struct can be accessed through mydata->MEMBER. E.g. mydata->age
//  On real robots, the compiler will automatically optimize the code to access member variables as if they were true globals.


uint8_t wait_for_min_nb_neighbors = 1;

fp_t initial_s_max_val = FROM_FLOAT(1.f);

fp_t diffusion_convergence_threshold = FROM_FLOAT(0.1f);
uint16_t diffusion_min_nb_points = 3;
fp_t diffusion_min_abs_s = FROM_FLOAT(0.e-05f);


#ifdef ENABLE_TAU_INCREASE
fp_t tau = FROM_FLOAT((0.01f)); // 1.f/30.f
fp_t tau_inc = FROM_FLOAT(2.0e-3f);
fp_t tau_max = FROM_FLOAT((0.12));
//fp_t min_s_val_to_update_led = FROM_FLOAT(0.05f);
fp_t min_s_val_to_update_led = FROM_FLOAT(0.01f);

//#define MAX_AGE (0.07e3)
//#define MAX_AGE (0.40e3)
//#define MAX_AGE (3.00e3)
//#define MAX_AGE (0.90e3)
//#define MAX_AGE (0.50e3)
#define MAX_AGE (1.20e3)
uint32_t max_age = MAX_AGE;
uint32_t ms_initial_run_tumble                = MAX_AGE * 0;
uint32_t ms_run_tumble_choice                 = MAX_AGE * 10;
uint32_t ms_run_tumble                        = MAX_AGE * 0;
uint32_t ms_handshake                         = MAX_AGE * 10;
uint32_t ms_diffusion                         = MAX_AGE * 100;
uint32_t ms_diffusion_it                      = MAX_AGE * 1;
uint32_t ms_diffusion_burnin                  = MAX_AGE * 10;
uint32_t ms_collective_avg_lambda             = MAX_AGE * 10;
uint32_t ms_collective_avg_lambda_it          = MAX_AGE;
uint32_t ms_collective_final_lambda           = MAX_AGE * 10;
uint32_t ms_collective_final_lambda_it        = MAX_AGE;
uint32_t ms_start_it_waiting_time             = MAX_AGE * 1;
uint32_t ms_iteration = 0; // Set in ``setup()``

#else
fp_t tau = FROM_FLOAT((1.f/15.f));
fp_t min_s_val_to_update_led = FROM_FLOAT(0.03f);

#define MAX_AGE (1.20e3)
uint32_t max_age = MAX_AGE;
uint32_t ms_initial_run_tumble                = MAX_AGE * 0;
uint32_t ms_run_tumble_choice                 = MAX_AGE * 10;
uint32_t ms_run_tumble                        = MAX_AGE * 0;
uint32_t ms_handshake                         = MAX_AGE * 5;
uint32_t ms_diffusion                         = MAX_AGE * 100;
uint32_t ms_diffusion_it                      = MAX_AGE;
uint32_t ms_diffusion_burnin                  = MAX_AGE * 10;
uint32_t ms_collective_avg_lambda             = MAX_AGE * 20;
uint32_t ms_collective_avg_lambda_it          = MAX_AGE;
uint32_t ms_collective_final_lambda           = MAX_AGE * 20;
uint32_t ms_collective_final_lambda_it        = MAX_AGE;
uint32_t ms_start_it_waiting_time             = MAX_AGE * 1;
uint32_t ms_iteration = 0; // Set in ``setup()``
#endif


///////////////////// MAIN CODE ///////////////////// {{{1

/**
 * @brief Generates a random duration for the run phase.
 *
 * The run phase duration is randomly chosen
 *
 * @return uint32_t Random run duration in milliseconds.
 */
static uint32_t get_run_duration(void) {
    return run_duration_min + (rand() % (run_duration_max - run_duration_min + 1));
}

/**
 * @brief Generates a random duration for the tumble phase.
 *
 * The tumble phase duration is randomly chosen
 *
 * @return uint32_t Random tumble duration in milliseconds.
 */
static uint32_t get_tumble_duration(void) {
    return tumble_duration_min + (rand() % (tumble_duration_max - tumble_duration_min + 1));
}

/**
 * @brief Set the direction of the robot to go forward or backward
 *
 * @param dir false for forward, true for backward
 */
static void set_robot_direction(bool dir) {
    if (!enable_backward_dir)
        return;
    if (dir) {
        pogobot_motor_dir_set(motorL, (mydata->motor_dir_left  == 0 ? 1 : 0));
        pogobot_motor_dir_set(motorR, (mydata->motor_dir_right == 0 ? 1 : 0));
    } else {
        pogobot_motor_dir_set(motorL, mydata->motor_dir_left);
        pogobot_motor_dir_set(motorR, mydata->motor_dir_right);
    }
}


/**
 * @brief Initialization function for the robot.
 *
 * This function is executed once at startup (cf 'pogobot_start' call in main()).
 * It seeds the random number generator, initializes timers and system parameters,
 * sets up the main loop frequency, and configures the initial state for the
 * run-and-tumble behavior.
 */


    

/**
 * @brief Main control loop for executing behavior.
 *
 * This function is called continuously at the frequency defined in user_init().
 * It checks if the current phase duration has elapsed and, if so, transitions to
 * the next phase. Depending on the current phase, it sets the robot's motors to
 * either move straight (run phase) or rotate (tumble phase). It also provides periodic
 * debugging output.
 */

    



inline static bool is_number_valid(fp_t nb) {
    return IS_NUMBER_VALID(nb);
}


void set_behavior(behavior_t behavior) {
    mydata->current_behavior = behavior;

#ifdef ENABLE_SHOW_BEHAVIOR_COLOR
    switch(mydata->current_behavior) {
        case INIT_BEHAVIOR:
            pogobot_led_setColors(1, 0, 0, 2);
            break;

        case INIT_RUN_TUMBLE:
            pogobot_led_setColors(1, 0, 0, 2);
            break;

        case RUN_TUMBLE:
            pogobot_led_setColors(10, 0, 0, 2);
            break;

        case PRE_DIFFUSION:
            pogobot_led_setColors(0, 3, 0, 2);
            break;

        case DIFFUSION:
            pogobot_led_setColors(0, 0, 3, 2);
            break;

        case CONSENSUS_LAMBDA:
            pogobot_led_setColors(1, 1, 0, 2);
            break;

#ifdef ENABLE_FINAL_LAMBDA
        case FINAL_LAMBDA:
            pogobot_led_setColors(10, 10, 0, 2);
            break;
#endif

        case WAITING_TIME:
            pogobot_led_setColors(3, 3, 3, 2);
            break;

#ifdef ENABLE_HANDSHAKES
        case HANDSHAKE:
            pogobot_led_setColors(3, 3, 3, 2);
            break;
#endif

        default:
            break;
    }
#endif
}


// Go through the list of neighbors, remove entries older than a threshold.
void purge_old_neighbors(void) {
    uint32_t const current_time = current_time_milliseconds();
    for(int8_t i = mydata->nb_neighbors-1; i >= 0; i--) {
        if(current_time - mydata->neighbors[i].timestamp > mydata->neighbors_age_max) {
            // Too old
            mydata->neighbors[i] = mydata->neighbors[mydata->nb_neighbors-1]; // Replace it by the last entry
            mydata->nb_neighbors--;
        }
    }
}

void clear_all_neighbors(void) {
    mydata->nb_neighbors = 0;
}

#ifdef ENABLE_HANDSHAKES

void clear_known_neighbors(void) {
    mydata->nb_known_neighbors = 0;
    mydata->current_peer_index = 0;
}

bool is_neighbor_known(uint16_t uid) {
    for(int8_t i = mydata->nb_known_neighbors-1; i >= 0; i--) {
        if(uid == mydata->known_neighbors_uid[i])
            return true;
    }
    return false;
}

#endif



void setup_diff(diffusion_session_t* diff) {
    diff->t = FROM_FLOAT(0);

    for(uint8_t i = 0; i < NUMBER_DIFF; i++) {
        diff->s[i] = FROM_FLOAT(0);
        diff->s0[i] = FROM_FLOAT(0);
#if defined(ENABLE_INIT_REMOVE_SUM_S)
        diff->sum_s0[i] = FROM_FLOAT(0);
#endif
        diff->lambda_[i] = FROM_FLOAT(0);
        diff->sum_t[i] = FROM_FLOAT(0);
        diff->sum_t2[i] = FROM_FLOAT(0);
        diff->sum_logs[i] = FROM_FLOAT(0);
        diff->sum_tlogs[i] = FROM_FLOAT(0);
        diff->ls_nb_points[i] = 0;
    }

    diff->lambda = FROM_FLOAT(0);
    diff->sum_lambda = FROM_FLOAT(0);
    diff->avg_lambda = FROM_FLOAT(0);
    diff->current_diffusion_it = 0;

    diff->diffusion_valid = false;
    for(uint8_t i = 0; i < NUMBER_DIFF; i++) {
        diff->stopped_diffusion[i] = false;
    }
    diff->current_diffusion_it = 0;
    diff->current_avg_it = 0;
    diff->time_last_diff_it = 0;
}



void setup(void) {
    init_rand();

    // Init ms_iteration
    ms_iteration = ms_run_tumble + ms_diffusion + ms_collective_avg_lambda + ms_start_it_waiting_time
#ifdef ENABLE_FINAL_LAMBDA
        + ms_collective_final_lambda
#endif
#ifdef ENABLE_PRE_DIFFUSION
        + ms_diffusion
#endif
#ifdef ENABLE_HANDSHAKES
        + ms_handshake
#endif
        ;
    init_transitions();

    // Init local variables
    mydata->time_last_it = 0;
    mydata->current_it = 0;
    mydata->ms_start_it = 0;
    set_behavior(INIT_BEHAVIOR);
    mydata->behavior_start_ms = 0;
    setup_diff(&mydata->diff1);
    mydata->curr_diff = &mydata->diff1;

    // Init communication
    for(uint8_t i = 0; i < NUMBER_DIFF; i++) {
        mydata->data_to_send.val[i] = q6_10_from_float(0.f);
    }
    mydata->data_to_send.sender_id = pogobot_helper_getid();
    mydata->data_to_send.data_type = DATA_NULL;
	pogobot_infrared_set_power(2);
    mydata->neighbors_age_max = max_age;
    mydata->enable_message_sending = false;
    clear_all_neighbors();
#ifdef ENABLE_HANDSHAKES
    clear_known_neighbors();
    mydata->hmsg_to_send.sender_id = pogobot_helper_getid();
    mydata->hmsg_to_send.data_type = DATA_HANDSHAKE;
#endif

#ifdef ENABLE_PHOTO_START
    mydata->last_data_b  = pogobot_photosensors_read(0);
    mydata->last_data_fl = pogobot_photosensors_read(1);
    mydata->last_data_fr = pogobot_photosensors_read(2);
#endif

    // Set main loop frequency, message sending frequency, message processing frequency
    main_loop_hz = MAIN_LOOP_HZ;
    max_nb_processed_msg_per_tick = MAX_NB_MSGS_PROCESSED_PER_TICK;
    percent_msgs_sent_per_ticks = PERCENT_MSG_SENT;
    // Specify functions to send/transmit messages
    msg_rx_fn = process_message;
    msg_tx_fn = send_message;

    // Set led index to show error codes
    error_codes_led_idx = 3; // Default value, negative values to disable

    for (uint8_t i = 0; i < 5; i++) {
        pogobot_led_setColors(0, 0, 0, i);
    }
    printf0("\ninit ok\n");
}


void behav_run_tumble(void)
{
    uint32_t const current_time = current_time_milliseconds();

    if (current_time - mydata->rt_phase_start >= mydata->rt_phase_duration) {
        if (mydata->rt_phase == RT_PHASE_RUN) {
            mydata->rt_phase          = RT_PHASE_TUMBLE;
            mydata->rt_phase_duration = get_tumble_duration();
            mydata->rt_tumble_dir     = rand() % 2;
            set_robot_direction(false);
        } else {
            mydata->rt_phase          = RT_PHASE_RUN;
            mydata->rt_phase_duration = get_run_duration();
            set_robot_direction((rand() % 2) == 0);
        }
        mydata->rt_phase_start = current_time;
    }

    if (mydata->rt_phase == RT_PHASE_RUN) {
        // Run phase: the robot moves forward.
        // Set LED color to green to indicate running.
        pogobot_led_setColor(0, 255, 0);
        // Set both motors to full speed for straight-line motion.
        pogobot_motor_set(motorL, motorFull);
        pogobot_motor_set(motorR, motorFull);          /* green */
    } else {
        // Tumble phase: the robot rotates in place.
        // Set LED color to red to indicate tumbling.
        pogobot_led_setColor(255, 0, 0);
        // Rotate the robot by driving one motor while stopping the other.
        if (mydata->rt_tumble_dir == 0) {
            // Tumble left: stop the left motor.
            pogobot_motor_set(motorL, motorStop);
            pogobot_motor_set(motorR, motorFull);
        } else {
            // Tumble right: stop the right motor.
            pogobot_motor_set(motorL, motorFull);
            pogobot_motor_set(motorR, motorStop);
        }
    }
}


void init_diffusion(diffusion_session_t* diff, fp_t* s, diffusion_type_t type) {
    mydata->data_to_send.data_type = DATA_NULL;
    clear_all_neighbors();

    // Set 'diff' as current diffusion session
    mydata->curr_diff = diff;

    // Initialize diffusion information
    diff->type = type;
    diff->t = FROM_FLOAT(0);

    for(uint8_t i = 0; i < NUMBER_DIFF; i++) {
        diff->s[i] = s[i];
        diff->s0[i] = diff->s[i];
        diff->lambda_[i] = FROM_FLOAT(0.);

        diff->sum_t[i] = FROM_FLOAT(0.);
        diff->sum_t2[i] = FROM_FLOAT(0.);
        diff->sum_logs[i] = FROM_FLOAT(0.);
        diff->sum_tlogs[i] = FROM_FLOAT(0.);
        diff->ls_nb_points[i] = 0;

        for(uint8_t j = 0; j < DIFFUSION_WINDOW_SIZE; j++) {
            diff->hist_logs[i][j] = 1000;
            diff->hist_t[i][j] = -1;
            diff->hist_mse[i][j] = -1;
        }
        diff->best_mse[i] = 1000;

        mydata->data_to_send.val[i] = TO_Q6_10(diff->s[i]);
    }
    diff->next_diff_to_compute = -1;
    diff->lambda = FROM_FLOAT(0);
    diff->current_diffusion_it = 0;
    diff->time_last_diff_it = current_time_milliseconds();
    diff->diffusion_valid = true;
    for(uint8_t i = 0; i < NUMBER_DIFF; i++) {
        diff->stopped_diffusion[i] = false;
    }

#ifdef ENABLE_TAU_INCREASE
    diff->tau = tau;
#endif

    // Broadcast s to neighboring agents
    switch(type) {
        case PRE_DIFFUSION_TYPE:
            mydata->data_to_send.data_type = DATA_PRE_S;
            break;
        case NORMAL_DIFFUSION_TYPE:
            mydata->data_to_send.data_type = DATA_S;
            break;
        default:
            mydata->data_to_send.data_type = DATA_NULL;
            break;
    }
    mydata->neighbors_age_max = ms_diffusion_it;
    mydata->enable_message_sending = true;

#ifdef ENABLE_COLOR_FROM_S0
    set_color_from_s(diff->s0[0]);
#elif defined(ENABLE_COLOR_FROM_S)
    set_color_from_s(diff->s0[0]);
#elif defined(ENABLE_COLOR_FROM_SIGNS)
    set_color_from_signs(diff->s0[0]);
#elif defined(ENABLE_COLOR_SIGNS_AND_CONSENSUSLAMBDA)
    set_color_from_signs(diff->s0[0]);
#elif defined(ENABLE_COLOR_NB_NEIGHBOURS)
    set_color_from_nb_neighbours();
#elif defined(ENABLE_COLOR_FROM_SIGNDELTAX)
    set_color_from_signs(diff->s0[0]);
#endif

    //    printf0("BEGIN DIFFUSION ! it=%d\n", mydata->current_it);
}


void compute_next_s(void) {
    diffusion_session_t *const diff = mydata->curr_diff;

    fp_t new_s[NUMBER_DIFF] = {FROM_FLOAT(0)};

    for(uint8_t i = 0; i < mydata->nb_neighbors; i++) {
        if(    (diff->type == NORMAL_DIFFUSION_TYPE && mydata->neighbors[i].data_type == DATA_S)
            || (diff->type == PRE_DIFFUSION_TYPE && mydata->neighbors[i].data_type == DATA_PRE_S)) {
            for(uint8_t j = 0; j < NUMBER_DIFF; j++) {
                fp_t const val = mydata->neighbors[i].val[j];
                // Step Kernel
                if(is_number_valid(val)) {
                    //new_s[j] += (fp_t)(mydata->neighbors[i].val[j] - diff->s[j]);
                    new_s[j] = ADD(new_s[j], (fp_t)(SUB(mydata->neighbors[i].val[j], diff->s[j])));
                }
                //printf0("    for next_s i=%d j=%d val=%Q16.16 new_s[j]=%Q16.16\n", i, j, TO_Q16_16(val), TO_Q16_16(new_s[j]));
            }
        }
    }

#ifdef ENABLE_STOP_DIFFUSION_AFTER_THRESHOLD
    for(uint8_t j = 0; j < NUMBER_DIFF; j++) {
        if (ABS(new_s[j]) <= FROM_FLOAT(1e-7f)) {
            diff->stopped_diffusion[j] = true;
        }
    }
#endif

#ifdef ENABLE_SINGLE_DIFF
    printf0("    next_s nb_neighbors=%d new_s=(%Q16.16)\n", mydata->nb_neighbors, TO_Q16_16(new_s[0]));
#elif defined(ENABLE_DOUBLE_DIFF)
    printf0("    next_s nb_neighbors=%d new_s=(%Q16.16, %Q16.16)\n", mydata->nb_neighbors, TO_Q16_16(new_s[0]), TO_Q16_16(new_s[1]));
#elif defined(ENABLE_TRIPLE_DIFF)
    printf0("    next_s nb_neighbors=%d new_s=(%Q16.16, %Q16.16, %Q16.16)\n", mydata->nb_neighbors, TO_Q16_16(new_s[0]), TO_Q16_16(new_s[1]), TO_Q16_16(new_s[2]));
#elif defined(ENABLE_HEXA_DIFF)
    printf0("    next_s nb_neighbors=%d new_s=(%Q16.16, %Q16.16, %Q16.16, %Q16.16, %Q16.16, %Q16.16)\n", mydata->nb_neighbors, TO_Q16_16(new_s[0]), TO_Q16_16(new_s[1]), TO_Q16_16(new_s[2]), TO_Q16_16(new_s[3]), TO_Q16_16(new_s[4]), TO_Q16_16(new_s[5]));
#endif

#if defined(ENABLE_COLOR_NB_NEIGHBOURS)
    set_color_from_nb_neighbours();
    //printf0("next_s: new_s %Q16.16\n", new_s);
#endif
    clear_all_neighbors();

    for(uint8_t j = 0; j < NUMBER_DIFF; j++) {
        //diff->s[j] += new_s[j] / inv_tau;
        //diff->s[j] = ADD(diff->s[j], DIV(new_s[j], inv_tau));
#ifdef ENABLE_TAU_INCREASE
        diff->s[j] = ADD(diff->s[j], MUL(new_s[j], diff->tau));
#else
        diff->s[j] = ADD(diff->s[j], MUL(new_s[j], tau));
#endif
    }

    // Check that s is not out of bounds
    for(uint8_t j = 0; j < NUMBER_DIFF; j++) {
        //if(ABS(diff->s[j]) > 2* initial_s_max_val) {
        if(ABS(diff->s[j]) > ADD(initial_s_max_val, initial_s_max_val)) {
            diff->diffusion_valid = false;
            diff->s[j] = NOT_A_NUMBER;
        }
    }
}


fp_t compute_MSE(uint8_t i) {
    diffusion_session_t *const diff = mydata->curr_diff;

    fp_t mse = FROM_FLOAT(0);
    uint8_t nb_points = 0;
    for(uint8_t j = 0; j < DIFFUSION_WINDOW_SIZE; j++) {
        //if(is_number_valid(err)) {
        fp_t const mse_inst = diff->hist_mse[i][j];
        if(mse_inst >= FROM_FLOAT(0)) {
            mse = ADD(mse, mse_inst);
            nb_points += 1;
        }
    }
    if(nb_points > 0)
        return DIV(mse, FROM_INT(nb_points));
    else
        return FROM_FLOAT(2000.f);
}


void compute_lambda_v_leastsquaresMSE(void) {
    diffusion_session_t *const diff = mydata->curr_diff;
    fp_t const t = diff->t;

    //if(!diff->diffusion_valid || diff->stopped_diffusion || diff->current_diffusion_it < ms_diffusion_burnin / ms_diffusion_it) {
    if(!diff->diffusion_valid || diff->current_diffusion_it < ms_diffusion_burnin / ms_diffusion_it) {
        return;
    }

    uint8_t const i = diff->next_diff_to_compute;

#ifdef ENABLE_TAU_INCREASE
    printf0_debug("DEBUG compute_lambda_v_leastsquaresMSE: t=%f  i=%d  s[i]=%Q16.16  stopped=%d  tau=%f\n", t, i, TO_Q16_16(diff->s[i]), diff->stopped_diffusion[i], diff->tau);
#else
    printf0_debug("DEBUG compute_lambda_v_leastsquaresMSE: i=%d  s[i]=%Q16.16  stopped=%d  tau=%Q16.16\n", i, TO_Q16_16(diff->s[i]), diff->stopped_diffusion[i], TO_Q16_16(tau));
#endif
    if (diff->stopped_diffusion[i])
        return;
#ifdef ENABLE_STOP_DIFFUSION_AFTER_THRESHOLD
    if (ABS(diff->s[i]) <= FROM_FLOAT(1e-7f)) {
        diff->stopped_diffusion[i] = true;
        return;
    }
#endif
    fp_t const logs = LOG(ABS(diff->s[i]));
#ifdef ENABLE_STOP_DIFFUSION_AFTER_THRESHOLD
    if (!is_number_valid(logs) || ABS(logs) <= FROM_FLOAT(0.f)) {
        diff->stopped_diffusion[i] = true;
        return;
    }
#endif
    diff->sum_t[i] = ADD(diff->sum_t[i], t);
    diff->sum_t2[i] = ADD(diff->sum_t2[i], MUL(t, t));
    diff->sum_logs[i] = ADD(diff->sum_logs[i], logs);
    diff->sum_tlogs[i] = ADD(diff->sum_tlogs[i], MUL(t, logs));
    diff->ls_nb_points[i] += 1;

    uint8_t const hist_idx = (uint8_t)(diff->ls_nb_points[i]) % DIFFUSION_WINDOW_SIZE;
    diff->hist_logs[i][hist_idx] = logs;
    diff->hist_t[i][hist_idx] = t;

    // Compute lambda and v
    //fp_t const _lambda = -(FROM_INT(diff->ls_nb_points[i]) * diff->sum_tlogs[i] - diff->sum_t[i] * diff->sum_logs[i]) / (FROM_INT(diff->ls_nb_points[i]) * diff->sum_t2[i] - diff->sum_t[i] * diff->sum_t[i]);
    fp_t const nb_points = FROM_INT(diff->ls_nb_points[i]);
    fp_t const _lambda = DIV(
              - SUB( MUL(nb_points, diff->sum_tlogs[i]), MUL(diff->sum_t[i], diff->sum_logs[i]) ),
                SUB( MUL(nb_points, diff->sum_t2[i]),    MUL(diff->sum_t[i], diff->sum_t[i]) )
            );
    //fp_t const _v = EXP( (diff->sum_logs[i] - _lambda * diff->sum_t[i]) / FROM_INT(diff->ls_nb_points[i]) );
    fp_t const _v = EXP( DIV(SUB(diff->sum_logs[i], MUL(_lambda, diff->sum_t[i])), FROM_INT(diff->ls_nb_points[i])) );

    // DEBUG
//    printf0("DEBUG compute_lambda_v_leastsquaresMSE 2: _lambda=%Q16.16  _v=%Q16.16\n", TO_Q16_16(_lambda), TO_Q16_16(_v));
//    fp_t const _a = -SUB( MUL(nb_points, diff->sum_tlogs[i]), MUL(diff->sum_t[i], diff->sum_logs[i]) );
//    fp_t const _b = SUB( MUL(nb_points, diff->sum_t2[i]),    MUL(diff->sum_t[i], diff->sum_t[i]) );
//    fp_t const _c = MUL(nb_points, diff->sum_t2[i]);
//    fp_t const _d = MUL(diff->sum_t[i], diff->sum_t[i]);
//    printf0("DEBUG compute_lambda_v_leastsquaresMSE 3: nb_points=%Q16.16  _a=%Q16.16  _b=%Q16.16  _c=%Q16.16  _d=%Q16.16\n", TO_Q16_16(nb_points), TO_Q16_16(_a), TO_Q16_16(_b), TO_Q16_16(_c), TO_Q16_16(_d));

    // Are lambda and v valid?
    if(!is_number_valid(_lambda) || !is_number_valid(_v)) {
        return;
    }

    // Compute instantaneous error
    //fp_t const err = (-_lambda * t + LOG(_v)) - logs;
    fp_t const err = SUB(ADD(MUL(-_lambda, t), LOG(_v)), logs);
    diff->hist_mse[i][hist_idx] = is_number_valid(err) ? err * err : FROM_FLOAT(-1);

    // Do we have enough points to compute stats?
    if(diff->ls_nb_points[i] <= 3) {
        return;
    }

    // Compute MSE. If the fit is better than previous fits, keep it
    if(diff->ls_nb_points[i] >= DIFFUSION_WINDOW_SIZE) {
        fp_t const mse = compute_MSE(i);
        if(mse < diff->best_mse[i]) {
            diff->best_mse[i] = mse;
            diff->lambda_[i] = _lambda;
            //diff->v[i] = _v;
        }
    } else {
        diff->lambda_[i] = _lambda;
        //diff->v[i] = _v;
    }

    //printf0("DEBUG diff: valid=%d t=%Q16.16 s=%Q16.16 logs=%Q16.16 lambda=%Q16.16 \n", diff->diffusion_valid, TO_Q16_16(t), TO_Q16_16(diff->s), TO_Q16_16(logs), TO_Q16_16(diff->lambda));
}


void behav_diffusion(void) {
//    time_reference_t timer_debug1;
//    pogobot_stopwatch_reset(&timer_debug1);

    diffusion_session_t *const diff = mydata->curr_diff;
    uint32_t const current_time = current_time_milliseconds();

    // Compute next s, if we are at the end of a diffusion step
    if (current_time - diff->time_last_diff_it >= ms_diffusion_it) {
#ifdef ENABLE_TAU_INCREASE
        if (diff->tau <= SUB(tau_max, tau_inc))
            diff->tau += tau_inc;
        //printf0("tau increase: %f\n", diff->tau);
#endif

        compute_next_s();
        if (diff->type != PRE_DIFFUSION_TYPE) {
            diff->next_diff_to_compute = 0;
        }

        // Update broadcasting information
        for (uint8_t i = 0; i < NUMBER_DIFF; i++) {
            mydata->data_to_send.val[i] = q6_10_from_float(diff->s[i]);
        }

        diff->current_diffusion_it++;
        diff->time_last_diff_it = current_time;
#ifdef ENABLE_TAU_INCREASE
        diff->t += diff->tau;
#else
        diff->t += tau;
#endif
        // Disable message I/O, to reduce computational costs at this step
        msg_rx_fn = NULL;
        msg_tx_fn = NULL;
    } else {
        // Reactivate message I/O
        msg_rx_fn = process_message;
        msg_tx_fn = send_message;
    }

//    printf0("next_s: elapsed:%uµs   ", pogobot_stopwatch_get_elapsed_microseconds(&timer_debug1));
//    pogobot_stopwatch_reset(&timer_debug1);

    // Compute lambda of next diffusion, if needed
    if (diff->next_diff_to_compute >= 0) {
        compute_lambda_v_leastsquaresMSE();
        diff->next_diff_to_compute++;
    }

//    printf0("lstMSE: elapsed:%uµs   ", pogobot_stopwatch_get_elapsed_microseconds(&timer_debug1));
//    pogobot_stopwatch_reset(&timer_debug1);

    // If we have computed the lambdas of all diffusion
    if (diff->next_diff_to_compute >= NUMBER_DIFF) {
        diff->next_diff_to_compute = -1; // Disable lambda computation for now, wait for next diffusion step

        // Compute lambda from the estimates of all diffusions
        if(diff->diffusion_valid && diff->current_diffusion_it >= ms_diffusion_burnin / ms_diffusion_it) {
            fp_t nb_valid_lambda = FROM_FLOAT(0);
            fp_t sum_all_lambda = FROM_FLOAT(0);
            for(uint8_t i = 0; i < NUMBER_DIFF; i++) {
                if(is_number_valid(diff->lambda_[i]) && diff->best_mse[i] < FROM_FLOAT(100.)) {
                    //sum_all_lambda += diff->lambda_[i];
                    sum_all_lambda = ADD(sum_all_lambda, diff->lambda_[i]);
                    nb_valid_lambda++;
                }
            }
            //fp_t const _lambda = sum_all_lambda / nb_valid_lambda;
            fp_t const _lambda = DIV(sum_all_lambda, nb_valid_lambda);
            if(!is_number_valid(_lambda) || nb_valid_lambda == 0) {
                //diff->diffusion_valid = false;
            } else {
                diff->lambda = _lambda;
            }
        }

//    printf0("allLambda: elapsed:%uµs   ", pogobot_stopwatch_get_elapsed_microseconds(&timer_debug1));
//    pogobot_stopwatch_reset(&timer_debug1);

#if defined(ENABLE_TRIPLE_DIFF)
        printf0("    step %d t=%Q16.16 s_0=%Q16.16 ls_nb_points_0=%u  stopped=%d,%d,%d  lambda=%Q16.16\n", diff->current_diffusion_it, TO_Q16_16(diff->t), TO_Q16_16(diff->s[0]), (unsigned int) diff->ls_nb_points[0], diff->stopped_diffusion[0], diff->stopped_diffusion[1], diff->stopped_diffusion[2], TO_Q16_16(diff->lambda));
#elif defined(ENABLE_HEXA_DIFF)
        printf0("    step %d t=%Q16.16 s_0=%Q16.16 ls_nb_points_0=%u  stopped=%d,%d,%d,%d,%d,%d  lambda=%Q16.16\n", diff->current_diffusion_it, TO_Q16_16(diff->t), TO_Q16_16(diff->s[0]), (unsigned int) diff->ls_nb_points[0], diff->stopped_diffusion[0], diff->stopped_diffusion[1], diff->stopped_diffusion[2], diff->stopped_diffusion[3], diff->stopped_diffusion[4], diff->stopped_diffusion[5], TO_Q16_16(diff->lambda));
#endif

#if defined(ENABLE_COLOR_FROM_S)
        //if (ABS(diff->s[0]) >= min_s_val_to_update_led && !diff->stopped_diffusion[0])
        if (ABS(diff->s[0]) >= min_s_val_to_update_led)
            set_color_from_s(diff->s[0]);
#elif defined(ENABLE_COLOR_FROM_SIGNS)
        //if (ABS(diff->s[0]) >= min_s_val_to_update_led && !diff->stopped_diffusion[0])
        if (ABS(diff->s[0]) >= min_s_val_to_update_led)
            set_color_from_signs(diff->s[0]);
#elif defined(ENABLE_COLOR_SIGNS_AND_CONSENSUSLAMBDA)
        //if (ABS(diff->s[0]) >= min_s_val_to_update_led && !diff->stopped_diffusion[0])
        if (ABS(diff->s[0]) >= min_s_val_to_update_led)
            set_color_from_signs(diff->s[0]);
//#elif defined(ENABLE_COLOR_NB_NEIGHBOURS)
//        set_color_from_nb_neighbours();
#endif
    }

//    printf0("led color: elapsed:%uµs   ", pogobot_stopwatch_get_elapsed_microseconds(&timer_debug1));
}



void end_diffusion(void) {
    diffusion_session_t *const diff = mydata->curr_diff;

    // Check if diffusion converged close to 0.
    if( (diffusion_convergence_threshold > FROM_FLOAT(0.f) && ABS(diff->s[0]) - FROM_FLOAT(0.f) > diffusion_convergence_threshold)
            || diff->ls_nb_points[0] < diffusion_min_nb_points
#if defined(ENABLE_DOUBLE_DIFF)
            || diff->ls_nb_points[1] < diffusion_min_nb_points
#elif defined(ENABLE_TRIPLE_DIFF)
            || diff->ls_nb_points[1] < diffusion_min_nb_points
            || diff->ls_nb_points[2] < diffusion_min_nb_points
#elif defined(ENABLE_HEXA_DIFF)
            || diff->ls_nb_points[1] < diffusion_min_nb_points
            || diff->ls_nb_points[2] < diffusion_min_nb_points
            || diff->ls_nb_points[3] < diffusion_min_nb_points
            || diff->ls_nb_points[4] < diffusion_min_nb_points
            || diff->ls_nb_points[5] < diffusion_min_nb_points
#endif
            || !is_number_valid(diff->lambda)
            ) {
        diff->diffusion_valid = false;
    }
    if(!is_number_valid(diff->lambda)) {
        diff->lambda = FROM_FLOAT(0.f);
        diff->diffusion_valid = false;
    }

    if(!diff->diffusion_valid) {
        diff->lambda = NOT_A_NUMBER;
    }

    printf0("END DIFFUSION ! it=%d id=%d diffit=%d lambda=%Q16.16 s[0]=%Q16.16 ls_nb_points_0=%u diffusion_valid=%d\n", mydata->current_it, pogobot_helper_getid(), diff->current_diffusion_it, TO_Q16_16(diff->lambda), TO_Q16_16(diff->s[0]), (unsigned int)diff->ls_nb_points[0], diff->diffusion_valid);
    printf0("  diffusion_convergence_threshold=%Q16.16  condition_convergence=%Q16.16\n", TO_Q16_16(diffusion_convergence_threshold), TO_Q16_16(ABS(diff->s[0]) - FROM_FLOAT(0.f)));
}


void init_coll_avg_lambda(void) {
    mydata->data_to_send.data_type = DATA_NULL;
    clear_all_neighbors();
    mydata->time_last_coll_avg_lambda_it = current_time_milliseconds();

    diffusion_session_t *const diff = mydata->curr_diff;
    if(diff->diffusion_valid) {
        mydata->data_to_send.val[0] = q6_10_from_float(diff->lambda);
        mydata->data_to_send.data_type = DATA_LAMBDA;
    } else {
        mydata->data_to_send.data_type = DATA_NULL;
    }
    mydata->neighbors_age_max = ms_collective_avg_lambda_it;
    mydata->enable_message_sending = true;
}

void behav_coll_avg_lambda(void) {
    uint32_t const current_time = current_time_milliseconds();
    if(current_time - mydata->time_last_coll_avg_lambda_it >= ms_collective_avg_lambda_it) {
        // Compute lambda through collective averaging
        uint8_t nb_neighbors = mydata->nb_neighbors;
        uint8_t used_neighbors = nb_neighbors;
        fp_t tmp = FROM_FLOAT(0.f);
        if(mydata->curr_diff->diffusion_valid) {
            tmp = ADD(tmp, mydata->curr_diff->lambda);
            ++used_neighbors;
        }
        for(uint8_t i = 0; i < nb_neighbors; i++) {
            if(mydata->neighbors[i].data_type == DATA_LAMBDA) {
                tmp = ADD(tmp, mydata->neighbors[i].val[0]);
            } else {
                --used_neighbors;
            }
        }

        clear_all_neighbors();
        if(used_neighbors > 0) {
            //tmp /= (fp_t)(used_neighbors);
            tmp = DIV(tmp, FROM_INT(used_neighbors));
            if(is_number_valid(tmp)) {
                mydata->curr_diff->lambda = tmp;
                mydata->data_to_send.val[0] = q6_10_from_float(mydata->curr_diff->lambda);
                mydata->data_to_send.data_type = DATA_LAMBDA;
            }
        }

        mydata->time_last_coll_avg_lambda_it = current_time;

        printf0("    avg_lambda: lambda=%Q16.16 diffusion_valid=%d\n", TO_Q16_16(mydata->curr_diff->lambda), mydata->curr_diff->diffusion_valid);
    }
}

void end_coll_avg_lambda(void) {
#ifndef ENABLE_FINAL_LAMBDA
    diffusion_session_t *const diff = mydata->curr_diff;
    if(is_number_valid(diff->lambda)) {
        diff->sum_lambda = ADD(diff->sum_lambda, diff->lambda);
        ++diff->current_avg_it;
        diff->avg_lambda = DIV(diff->sum_lambda, FROM_INT(diff->current_avg_it));
#if defined(ENABLE_COLOR_SIGNS_AND_AVGLAMBDA)
        set_color_from_lambda(diff->avg_lambda, 0);
#endif
    }

    printf0("    end_coll_avg_lambda: sum_lambda=%Q16.16 avg_lambda=%Q16.16\n", TO_Q16_16(diff->sum_lambda), TO_Q16_16(diff->avg_lambda));
#endif
}


#ifdef ENABLE_FINAL_LAMBDA
void init_coll_final_lambda(void) {
    mydata->data_to_send.data_type = DATA_NULL;
    clear_all_neighbors();

    diffusion_session_t *const diff = mydata->curr_diff;
    mydata->time_last_coll_final_lambda_it = current_time_milliseconds();

    if(is_number_valid(diff->lambda)) {
        //diff->sum_lambda += diff->lambda * 1;
        diff->sum_lambda = ADD(diff->sum_lambda, diff->lambda);
        ++diff->current_avg_it;
        //diff->avg_lambda = (fp_t) (diff->sum_lambda / (mydata->current_it + 1));
        diff->avg_lambda = DIV(diff->sum_lambda, FROM_INT(mydata->current_it + 1));
    }

    if(is_number_valid(diff->avg_lambda)) {
        mydata->data_to_send.val[0] = q6_10_from_float(diff->avg_lambda);
        mydata->data_to_send.data_type = DATA_CONSENSUS_LAMBDA;
    } else {
        mydata->data_to_send.data_type = DATA_NULL;
    }
    mydata->neighbors_age_max = ms_collective_final_lambda_it;
    mydata->enable_message_sending = true;
}

void behav_coll_final_lambda(void) {
    diffusion_session_t *const diff = mydata->curr_diff;
    uint32_t const current_time = current_time_milliseconds();

    if(current_time - mydata->time_last_coll_final_lambda_it >= ms_collective_final_lambda_it) {
        uint8_t nb_neighbors = mydata->nb_neighbors;
        uint8_t used_neighbors = nb_neighbors;
        fp_t tmp = FROM_FLOAT(0);
        if(is_number_valid(diff->avg_lambda)) {
            tmp = ADD(tmp, diff->avg_lambda);
            ++used_neighbors;
        }
        for(uint8_t i = 0; i < nb_neighbors; i++) {
            if(mydata->neighbors[i].data_type == DATA_CONSENSUS_LAMBDA) {
                tmp = ADD(tmp, mydata->neighbors[i].val[0]);
            } else {
                --used_neighbors;
            }
        }

        clear_all_neighbors();
        if(used_neighbors > 0) {
            tmp = DIV(tmp, FROM_INT(used_neighbors));
            diff->avg_lambda = tmp;
            if(is_number_valid(diff->avg_lambda)) {
                mydata->data_to_send.val[0] = q6_10_from_float(diff->avg_lambda);
                mydata->data_to_send.data_type = DATA_CONSENSUS_LAMBDA;
            }
        }
        mydata->time_last_coll_final_lambda_it = current_time;

#if defined(ENABLE_COLOR_SIGNS_AND_CONSENSUSLAMBDA)
        set_color_from_lambda(diff->avg_lambda, 0);
#endif
        printf0("    final_lambda: avg_lambda=%Q16.16\n", TO_Q16_16(diff->avg_lambda));
    }

}
#endif


#ifdef ENABLE_HANDSHAKES
void behav_handshake(void) {
    mydata->enable_message_sending = true;

    // Set number of peers to include in the message
    uint8_t nb_peers = mydata->nb_neighbors;
    mydata->hmsg_to_send.nb_peers = nb_peers;

    // Add peers in the message
    for(uint8_t i = 0; i < nb_peers; i++) {
        if(mydata->current_peer_index >= mydata->nb_neighbors)
            mydata->current_peer_index = 0;
        mydata->hmsg_to_send.peers[i] = mydata->neighbors[mydata->current_peer_index].id;
        mydata->current_peer_index++;
    }
    printf0_debug("Exchanging Handshakes. nb_peers=%d nb_known_neighbors=%d \n", nb_peers, mydata->nb_known_neighbors);
}
#endif



void end_iteration(void) {
#if !defined(ENABLE_FINAL_LAMBDA)
    end_coll_avg_lambda();
#endif

#ifdef ENABLE_HANDSHAKES
    clear_known_neighbors();
#endif

#ifdef ENABLE_INIT_REMOVE_SUM_S
    if(mydata->current_it == 0) {
        diffusion_session_t *const diff = mydata->curr_diff;
        diff->sum_s0[0] = diff->s[0];
        diff->sum_s0[1] = diff->s[1];
        diff->sum_s0[2] = diff->s[2];
        diff->sum_lambda = FROM_FLOAT(0.f);
        diff->avg_lambda = FROM_FLOAT(0.f);
        diff->current_avg_it = FROM_FLOAT(0.f);
//            printf0("SUM(S(0)):%Q16.16  x:%Q16.16  lambda:%Q16.16\n", diff->sum_s0, diff->s[0], diff->lambda);
    }
#endif

#if defined(ENABLE_COLOR_SIGNS_AND_CONSENSUSLAMBDA)
    set_color_from_lambda(mydata->curr_diff->avg_lambda, 0);
#else
    pogobot_led_setColors(3, 3, 3, 0);
#endif
    mydata->current_it++;

    printf0("END ITERATION: avg_lambda:%Q16.16\n", TO_Q16_16(mydata->curr_diff->avg_lambda));
    uint32_t const current_time = current_time_milliseconds();
    mydata->time_last_it = current_time;
    mydata->ms_start_it = current_time - ms_initial_run_tumble;
}



// Transition function prototypes:
static void transition_waiting_time(uint32_t ct);
static void transition_run_tumble(uint32_t ct);
#ifdef ENABLE_HANDSHAKES
static void transition_handshake(uint32_t ct);
#endif
#ifdef ENABLE_PRE_DIFFUSION
static void transition_pre_diffusion(uint32_t ct);
#endif
static void transition_diffusion(uint32_t ct);
static void transition_avg_lambda(uint32_t ct);
#ifdef ENABLE_FINAL_LAMBDA
static void transition_final_lambda(uint32_t ct);
#endif

// Structure that defines a behavior transition.
typedef struct {
    uint32_t duration;            // Duration for this transition phase.
    behavior_t new_behavior;      // New behavior to transition to.
    void (*transition)(uint32_t); // Transition function (called with current time).
} transition_t;

//// Transition table
//static const transition_t transitions[] = {
//    { ms_start_it_waiting_time, WAITING_TIME, transition_waiting_time },
//    { ms_randow_walk,           RANDOM_WALK,  transition_random_walk },
//#ifdef ENABLE_HANDSHAKES
//    { ms_handshake,             HANDSHAKE,    transition_handshake },
//#endif
//#ifdef ENABLE_PRE_DIFFUSION
//    { ms_diffusion,             PRE_DIFFUSION,transition_pre_diffusion },
//#endif
//    { ms_diffusion,             DIFFUSION,    transition_diffusion },
//    { ms_collective_avg_lambda, CONSENSUS_LAMBDA,   transition_avg_lambda },
//#ifdef ENABLE_FINAL_LAMBDA
//    { ms_collective_final_lambda, FINAL_LAMBDA, transition_final_lambda },
//#endif
//};
// Define maximum number of possible transitions
#define MAX_TRANSITIONS 10
static transition_t transitions[MAX_TRANSITIONS];
static int num_transitions = 0;

void init_transitions(void) {
    // Reset the count
    num_transitions = 0;


    // Add transitions one by one
    transitions[num_transitions++] = (transition_t){ ms_start_it_waiting_time, WAITING_TIME, transition_waiting_time };
    transitions[num_transitions++] = (transition_t){ ms_run_tumble, RUN_TUMBLE, transition_run_tumble };
#ifdef ENABLE_HANDSHAKES
    transitions[num_transitions++] = (transition_t){ ms_handshake, HANDSHAKE, transition_handshake };
#endif
#ifdef ENABLE_PRE_DIFFUSION
    transitions[num_transitions++] = (transition_t){ ms_diffusion, PRE_DIFFUSION, transition_pre_diffusion };
#endif
    transitions[num_transitions++] = (transition_t){ ms_diffusion, DIFFUSION, transition_diffusion };
    transitions[num_transitions++] = (transition_t){ ms_collective_avg_lambda, CONSENSUS_LAMBDA, transition_avg_lambda };
#ifdef ENABLE_FINAL_LAMBDA
    transitions[num_transitions++] = (transition_t){ ms_collective_final_lambda, FINAL_LAMBDA, transition_final_lambda };
#endif
}

// --- Transition Functions ---
static void transition_waiting_time(uint32_t ct) {
    if (mydata->current_behavior != WAITING_TIME) {
        mydata->behavior_start_ms = ct;
        //set_motion(STOP);
        mydata->enable_message_sending = false;
        printf0("BEGIN ITERATION it=%d\n", mydata->current_it);
        printf0("  it=%d WAITING_TIME\n", mydata->current_it);
        pogobot_led_setColors(3, 3, 3, 0);
        set_behavior(WAITING_TIME);
    }
}

static void transition_run_tumble(uint32_t ct)
{
    if (mydata->current_behavior != RUN_TUMBLE) {
        mydata->behavior_start_ms = ct;
        mydata->enable_message_sending = false;
        printf0("  it=%d RUN_TUMBLE\n", mydata->current_it);
        set_behavior(RUN_TUMBLE);
    }
}

#ifdef ENABLE_HANDSHAKES
static void transition_handshake(uint32_t ct) {
    if (mydata->current_behavior != HANDSHAKE) {
        //set_motion(STOP);
        printf0("Exchanging Handshakes\n");
        clear_all_neighbors();
        clear_known_neighbors();
        set_behavior(HANDSHAKE);
    }
}
#endif

#ifdef ENABLE_PRE_DIFFUSION
static void transition_pre_diffusion(uint32_t ct) {
    if (mydata->current_behavior != PRE_DIFFUSION) {
#ifndef ENABLE_ALWAYS_MOVING
    //set_motion(STOP);
#endif
        fp_t s[NUMBER_DIFF];
        for (uint8_t i = 0; i < NUMBER_DIFF; i++) {
            s[i] = ((uint32_t)(rand() + pogobot_helper_getid()) % 2 == 0)
                    ? -initial_s_max_val : initial_s_max_val;
        }
        printf0("  it=%d PRE_DIFFUSION\n", mydata->current_it);
        init_diffusion(&mydata->diff1, s, PRE_DIFFUSION_TYPE);
        set_behavior(PRE_DIFFUSION);
    }
}
#endif

static void transition_diffusion(uint32_t ct) {
    if (mydata->current_behavior != DIFFUSION) {
#ifndef ENABLE_ALWAYS_MOVING
    //set_motion(STOP);
#endif
        fp_t s[NUMBER_DIFF];
        for (uint8_t i = 0; i < NUMBER_DIFF; i++) {
            s[i] = (pogobot_helper_getid() % 2 == 0)
                   ? -initial_s_max_val : initial_s_max_val;
#ifdef ENABLE_INIT_REMOVE_SUM_S
            s[i] = SUB(s[i], mydata->diff1.sum_s0[i]);
#elif defined(ENABLE_PRE_DIFFUSION)
            s[i] = SUB(mydata->diff1.s0[i], mydata->diff1.s[i]);
#endif
        }
        printf0("  it=%d DIFFUSION\n", mydata->current_it);
        init_diffusion(&mydata->diff1, s, NORMAL_DIFFUSION_TYPE);
        set_behavior(DIFFUSION);
    }
}

static void transition_avg_lambda(uint32_t ct) {
#ifdef ENABLE_INIT_REMOVE_SUM_S
    if (mydata->current_it > 0 && mydata->current_behavior != CONSENSUS_LAMBDA) {
#else
    if (mydata->current_behavior != CONSENSUS_LAMBDA) {
#endif
#ifndef ENABLE_ALWAYS_MOVING
    //set_motion(STOP);
#endif
        end_diffusion();
        printf0("  it=%d CONSENSUS_LAMBDA\n", mydata->current_it);
        init_coll_avg_lambda();
        set_behavior(CONSENSUS_LAMBDA);
    }
}

#ifdef ENABLE_FINAL_LAMBDA
static void transition_final_lambda(uint32_t ct) {
#ifdef ENABLE_INIT_REMOVE_SUM_S
    if (mydata->current_it > 0 && mydata->current_behavior != FINAL_LAMBDA) {
#else
    if (mydata->current_behavior != FINAL_LAMBDA) {
#endif
#ifndef ENABLE_ALWAYS_MOVING
    //set_motion(STOP);
#endif
        end_coll_avg_lambda();
        printf0("  it=%d FINAL_LAMBDA\n", mydata->current_it);
        init_coll_final_lambda();
        set_behavior(FINAL_LAMBDA);
    }
}
#endif

// --- Main iteration function ---
void iteration(void) {

//    time_reference_t timer_debug2;
//    pogobot_stopwatch_reset(&timer_debug2);
    uint32_t current_time = current_time_milliseconds();

    // Example use of the USERDATA data array.
    //mydata->data_foo[0] = 42;
    uint32_t ticks_after_init_phase = current_time - ms_initial_run_tumble;
    uint32_t ms_elapsed_since_start_it = ticks_after_init_phase - mydata->ms_start_it;
    behavior_t current_behavior = mydata->current_behavior;

    // Debug: Print phase information periodically for robot with ID 0.
    if (pogobot_ticks % 1000 == 0 && pogobot_helper_getid() == 0) {
        uint32_t now = current_time_milliseconds();
        printf("Phase: %s, Phase Duration: %lums, Elapsed: %lums\n",
               (mydata->rt_phase == RT_PHASE_RUN) ? "RUN" : "TUMBLE",
               mydata->rt_phase_duration,
               now - mydata->rt_phase_start);
    }


#if defined(ENABLE_PHOTO_START)
    if (current_behavior == INIT_BEHAVIOR) {
        mydata->enable_message_sending = false;
        int16_t data_b  = pogobot_photosensors_read(0);
        int16_t data_fl = pogobot_photosensors_read(1);
        int16_t data_fr = pogobot_photosensors_read(2);
        int16_t diff_b  = data_b  - mydata->last_data_b;
        int16_t diff_fl = data_fl - mydata->last_data_fl;
        int16_t diff_fr = data_fr - mydata->last_data_fr;
        mydata->last_data_b  = data_b;
        mydata->last_data_fl = data_fl;
        mydata->last_data_fr = data_fr;
        printf0_debug("Current light level: %d %d %d \n", diff_b, diff_fl, diff_fr);
        if (diff_b >= LIGHT_THRESHOLD || diff_fl >= LIGHT_THRESHOLD || diff_fr >= LIGHT_THRESHOLD) {
            printf0("Detected light is above threshold. Starting experiment!\n");
            clear_all_neighbors();
            set_behavior(MISC_BEHAVIOR);
            _current_time_milliseconds = 0; // Reset time counter
        }
        return;
    }
#endif

#ifdef SHOW_CONSENSUSLAMBDA_IN_LED1
    set_color_from_lambda(mydata->curr_diff->avg_lambda, 1);
#endif

    // Special-case: initial random walk/ run tumble phase
    if (current_time < ms_initial_run_tumble) {
        if (current_behavior != INIT_RUN_TUMBLE) {
            mydata->behavior_start_ms = current_time;
            mydata->enable_message_sending = false;
            printf0("Initial run tumble\n");
            pogobot_led_setColors(0, 0, 0, 0);
            set_behavior(INIT_RUN_TUMBLE);
        }
        mydata->time_last_it = current_time;
    } else {
        // Table-driven transition: accumulate durations and select the proper transition.
        uint32_t cumulative = 0;
        bool transition_found = false;
        for (size_t i = 0; i < num_transitions; i++) {
            cumulative += transitions[i].duration;
            if (ms_elapsed_since_start_it < cumulative) {
                transitions[i].transition(current_time);
                transition_found = true;
                break;
            }
        }
        if (!transition_found) {
            end_iteration();
        }
    }
#ifdef ENABLE_ALWAYS_MOVING
    behav_run_tumble();
#endif

//    printf0("iteration1: elapsed:%uųs   ",
//             pogobot_stopwatch_get_elapsed_microseconds(&timer_debug2));
//    pogobot_stopwatch_reset(&timer_debug2);

    // Dispatch behavior function based on current behavior.
    switch (mydata->current_behavior) {
        case INIT_RUN_TUMBLE:
        case RUN_TUMBLE:
#ifndef ENABLE_ALWAYS_MOVING
            behav_run_tumble();
#endif
            break;
#ifdef ENABLE_HANDSHAKES
        case HANDSHAKE:
            behav_handshake();
            break;
#endif
#ifdef ENABLE_PRE_DIFFUSION
        case PRE_DIFFUSION:
#endif
        case DIFFUSION:
            behav_diffusion();
            break;
        case CONSENSUS_LAMBDA:
            behav_coll_avg_lambda();
            break;
#ifdef ENABLE_FINAL_LAMBDA
        case FINAL_LAMBDA:
            behav_coll_final_lambda();
            break;
#endif
        default:
            break;
    }
//    printf0("iteration2: elapsed:%uųs   ",
//             pogobot_stopwatch_get_elapsed_microseconds(&timer_debug2));
}





bool send_message(void) {
    if(!mydata->enable_message_sending) {
        // Not allowed to send messages!
        return false;
    }

#ifdef SIMULATE_FAKE_COMM
    return true;
#endif

    // Send message
    switch(mydata->current_behavior) {
#ifdef ENABLE_HANDSHAKES
        case HANDSHAKE:
#ifdef ENABLE_SYNC_TIME
            mydata->hmsg_to_send.time = current_time_milliseconds();
#endif
            pogobot_infrared_sendShortMessage_omni( (uint8_t *)(&mydata->hmsg_to_send), sizeof(mydata->hmsg_to_send) );
            break;
#endif

        default:
#ifdef ENABLE_SYNC_TIME
            mydata->data_to_send.time = current_time_milliseconds();
#endif
            //pogobot_infrared_sendMessageAllDirection( 0x1234, (uint8_t *)(&mydata->data_to_send), sizeof(mydata->data_to_send) );
            pogobot_infrared_sendShortMessage_omni( (uint8_t *)(&mydata->data_to_send), sizeof(mydata->data_to_send) );
            break;
    }
    return true;
}


void process_message(message_t* mr) {
    //static time_reference_t timer_debug;
    //pogobot_stopwatch_reset(&timer_debug);

    // Cache current time and own ID to avoid multiple function calls.
    uint32_t now = current_time_milliseconds();
    uint16_t my_id = pogobot_helper_getid();

    // Verify payload size.
    size_t payload_size = mr->header.payload_length;
    if (payload_size < sizeof(message_data_t))
        return;

    // Cache pointer to message data.
    message_data_t const* data = (message_data_t const*)&(mr->payload);
    uint16_t sender_id = data->sender_id;
    if (sender_id == my_id)
        return;

#ifdef ENABLE_HANDSHAKES
    // If not a handshake message and sender unknown, exit early.
    if (data->data_type != DATA_HANDSHAKE && !is_neighbor_known(sender_id)) {
        printf0_debug("    recv from unknown neighbor:%d  known_neighbors:%d\n", sender_id, mydata->nb_known_neighbors);

#ifdef ENABLE_SYNC_TIME
        if (data->time >= current_time_milliseconds() + 10) {
            _current_time_milliseconds = data->time;
            pogobot_stopwatch_reset(&_global_timer);
        }
#endif
        return;
    }
#endif

    // Validate message type against current behavior.
    switch (mydata->current_behavior) {
#ifdef ENABLE_HANDSHAKES
        case HANDSHAKE:
            if (data->data_type != DATA_HANDSHAKE)
                return;
            break;
#endif
        case PRE_DIFFUSION:
            if (data->data_type != DATA_PRE_S)
                return;
            break;
        case DIFFUSION:
            if (data->data_type != DATA_S)
                return;
            break;
        case CONSENSUS_LAMBDA:
            if (data->data_type != DATA_LAMBDA)
                return;
            break;
#ifdef ENABLE_FINAL_LAMBDA
        case FINAL_LAMBDA:
            if (data->data_type != DATA_CONSENSUS_LAMBDA)
                return;
            break;
#endif
        default:
            break;
    }

    // Cache pointer to the neighbors array and number of neighbors.
    neighbor_t* neighbors = mydata->neighbors;
    uint16_t nb_neighbors = mydata->nb_neighbors;
    uint16_t i;

    // Search for sender in neighbor list.
    for (i = 0; i < nb_neighbors; i++) {
        if (neighbors[i].id == sender_id)
            break;
    }

    // If sender is new, add it (if there is room).
    if (i == nb_neighbors) {
        if (nb_neighbors < MAXN - 1) {
            mydata->nb_neighbors = nb_neighbors + 1;
            nb_neighbors++; // Update local cache as well.
        }
    }

    // Cache pointer to the specific neighbor entry.
    neighbor_t* pNeighbor = &neighbors[i];
    pNeighbor->id = sender_id;
    pNeighbor->timestamp = now;
    pNeighbor->data_type = data->data_type;

    // Process internal message data based on current behavior.
    switch (mydata->current_behavior) {
#ifdef ENABLE_HANDSHAKES
        case HANDSHAKE: {
            handshake_message_data_t const* hmsg = (handshake_message_data_t const*)&(mr->payload);
            printf0_debug("    recv handshake from neighbor:%d/%d id=%u hmsg->nb_peers=%d nb_known_neighbors=%d\n", i, mydata->nb_neighbors, mydata->neighbors[i].id, hmsg->nb_peers, mydata->nb_known_neighbors);
#ifdef ENABLE_SYNC_TIME
            pNeighbor->time = hmsg->time;
#endif
            uint8_t nb_peers = hmsg->nb_peers;
            if (nb_peers > MAXN)
                nb_peers = MAXN;
            uint8_t am_i_a_peer = 0;
            for (uint8_t j = 0; j < nb_peers; j++) {
                if (hmsg->peers[j] == my_id) {
                    am_i_a_peer = 1;
                    break;
                }
            }
            if (am_i_a_peer && !is_neighbor_known(sender_id) && mydata->nb_known_neighbors < MAXN - 1) {
                mydata->known_neighbors_uid[mydata->nb_known_neighbors++] = sender_id;
            }
            break;
        }
#endif

        default: {
            // Convert fixed-point values to float using the precomputed reciprocal.
            for (uint8_t j = 0; j < NUMBER_DIFF; j++) {
                pNeighbor->val[j] = FROM_Q6_10(data->val[j]);
            }
#ifdef ENABLE_SYNC_TIME
            pNeighbor->time = data->time;
#endif

#if defined(ENABLE_SINGLE_DIFF)
            printf0_debug("    recv msg from neighbor:%d/%d id=%u val0=%Q16.16 type=%d\n", i, mydata->nb_neighbors, mydata->neighbors[i].id, TO_Q16_16(mydata->neighbors[i].val[0]), (int)mydata->neighbors[i].data_type);
#elif defined(ENABLE_DOUBLE_DIFF)
            printf0_debug("    recv msg from neighbor:%d/%d id=%u val0=%Q16.16 val1=%Q16.16 type=%d\n", i, mydata->nb_neighbors, mydata->neighbors[i].id, TO_Q16_16(mydata->neighbors[i].val[0]), TO_Q16_16(mydata->neighbors[i].val[1]), (int)mydata->neighbors[i].data_type);
#elif defined(ENABLE_TRIPLE_DIFF)
            printf0_debug("    recv msg from neighbor:%d/%d id=%u val0=%Q16.16 val1=%Q16.16 val2=%Q16.16 type=%d\n", i, mydata->nb_neighbors, mydata->neighbors[i].id, TO_Q16_16(mydata->neighbors[i].val[0]), TO_Q16_16(mydata->neighbors[i].val[1]), TO_Q16_16(mydata->neighbors[i].val[2]), (int)mydata->neighbors[i].data_type);
#elif defined(ENABLE_HEXA_DIFF)
            printf0_debug("    recv msg from neighbor:%d/%d id=%u val0=%Q16.16 val1=%Q16.16 val2=%Q16.16 val3=%Q16.16 val4=%Q16.16 val5=%Q16.16 type=%d\n", i, mydata->nb_neighbors, mydata->neighbors[i].id, TO_Q16_16(mydata->neighbors[i].val[0]), TO_Q16_16(mydata->neighbors[i].val[1]), TO_Q16_16(mydata->neighbors[i].val[2]), TO_Q16_16(mydata->neighbors[i].val[3]), TO_Q16_16(mydata->neighbors[i].val[4]), TO_Q16_16(mydata->neighbors[i].val[5]), (int)mydata->neighbors[i].data_type);
#endif
            break;
        }
    }

#ifdef ENABLE_SYNC_TIME
    // Adjust current time if it is too different from the neighbor's time
    //if (current_time_milliseconds() + 30 >= pNeighbor->time) {
    //    _current_time_milliseconds = pNeighbor->time + 1;
    //    pogobot_stopwatch_reset(&_global_timer);
    //}
    if (pNeighbor->time >= current_time_milliseconds() + 10) {
        _current_time_milliseconds = pNeighbor->time;
        pogobot_stopwatch_reset(&_global_timer);
    }
#endif


    //printf0("proc_msg:%uµs  ", pogobot_stopwatch_get_elapsed_microseconds(&timer_debug));
}



#ifdef SIMULATE_FAKE_COMM
void generate_fake_message(uint16_t fake_sender_id) {
    // Copy the message we are currently sending
    message_t m;
    //memcpy(&m.payload, &mydata->data_to_send, sizeof(mydata->data_to_send));
    m.header.payload_length = sizeof(mydata->data_to_send);
    message_data_t* data = (message_data_t*)(&( m.payload ));
    // Change sender ID
    data->sender_id = fake_sender_id;
    data->data_type = mydata->data_to_send.data_type;

//    // Apply random mutation
//    for (uint8_t i = 0; i < NUMBER_DIFF; i++) {
//        //q6_10_t rnd = q6_10_from_float(-0.1f + ((float)rand() / RAND_MAX) * (0.1f - (-0.1f)));
//        //data->val[i] = mydata->data_to_send.val[i] + rnd;
//        data->val[i] = mydata->data_to_send.val[i] + (q6_10_t)((int16_t)rand() >> 8);
//    }

    // Process fake message
    process_message(&m);
}

void simulate_fake_messages(void) {
    for (uint8_t i = 0; i < 10; i++) {
        generate_fake_message(pogobot_helper_getid() + i);
    }
}
#endif


void loop(void) {
    //static time_reference_t timer_debug;
    //pogobot_stopwatch_reset(&timer_debug);

#ifdef SIMULATE_FAKE_COMM
    simulate_fake_messages();
    //printf0("main loop fake msg: elapsed:%uµs   ", pogobot_stopwatch_get_elapsed_microseconds(&timer_debug));
    //pogobot_stopwatch_reset(&timer_debug);
#endif

    // Purge neighbors if needed
    purge_old_neighbors();

    //printf0("main loop prelim: elapsed:%uµs   ", pogobot_stopwatch_get_elapsed_microseconds(&timer_debug));
    //pogobot_stopwatch_reset(&timer_debug);

    // Main behavior of the robots
    iteration();
    //printf0("main loop iteration: elapsed:%uµs\n", pogobot_stopwatch_get_elapsed_microseconds(&timer_debug));
}


#ifdef SIMULATOR
void init_fp_from_configuration(fp_t* var, char const* name, fp_t const default_value) {
    float tmp;
    init_float_from_configuration(&tmp, name, TO_FLOAT(default_value));
    *var = FROM_FLOAT(tmp);
}

// Function called once to initialize global values (e.g. configuration-specified constants)
void global_setup() {
    init_from_configuration(wait_for_min_nb_neighbors);
    init_from_configuration(initial_s_max_val);
    init_from_configuration(diffusion_convergence_threshold);
    init_from_configuration(diffusion_min_nb_points);
    init_from_configuration(diffusion_min_abs_s);

    init_from_configuration(run_duration_min);
    init_from_configuration(run_duration_max);
    init_from_configuration(tumble_duration_min);
    init_from_configuration(tumble_duration_max);
    init_from_configuration(enable_backward_dir);
    init_array_from_configuration(test_vect);

    init_from_configuration(tau);
#ifdef ENABLE_TAU_INCREASE
    init_from_configuration(tau_inc);
    init_from_configuration(tau_max);
#else
#endif
    init_from_configuration(min_s_val_to_update_led);

    init_array_from_configuration(class_centroids);

    // Set temporal informations
    uint32_t age_initial_run_tumble               = 0;
    uint32_t age_run_tumble_choice                = 0;
    uint32_t age_run_tumble                       = 0;
    uint32_t age_handshake                         = 0;
    uint32_t age_diffusion                         = 0;
    uint32_t age_diffusion_it                      = 0;
    uint32_t age_diffusion_burnin                  = 0;
    uint32_t age_collective_avg_lambda             = 0;
    uint32_t age_collective_avg_lambda_it          = 0;
    uint32_t age_collective_final_lambda         = 0;
    uint32_t age_collective_final_lambda_it      = 0;
    uint32_t age_start_it_waiting_time             = 0;

    init_from_configuration(max_age);
    init_from_configuration(age_initial_run_tumble);
    init_from_configuration(age_run_tumble_choice);
    init_from_configuration(age_run_tumble);
    init_from_configuration(age_handshake);
    init_from_configuration(age_diffusion);
    init_from_configuration(age_diffusion_it);
    init_from_configuration(age_diffusion_burnin);
    init_from_configuration(age_collective_avg_lambda);
    init_from_configuration(age_collective_avg_lambda_it);
    init_from_configuration(age_collective_final_lambda);
    init_from_configuration(age_collective_final_lambda_it);
    init_from_configuration(age_start_it_waiting_time);

#define _init_if_not_zero(x, y) do { if (x > 0) { y = x * max_age; } } while(0)

    _init_if_not_zero(age_initial_run_tumble, ms_initial_run_tumble);
    _init_if_not_zero(age_run_tumble_choice, ms_run_tumble_choice);
    _init_if_not_zero(age_run_tumble, ms_run_tumble);
    _init_if_not_zero(age_handshake, ms_handshake);
    _init_if_not_zero(age_diffusion, ms_diffusion);
    _init_if_not_zero(age_diffusion_it, ms_diffusion_it);
    _init_if_not_zero(age_diffusion_burnin, ms_diffusion_burnin);
    _init_if_not_zero(age_collective_avg_lambda, ms_collective_avg_lambda);
    _init_if_not_zero(age_collective_avg_lambda_it, ms_collective_avg_lambda_it);
    _init_if_not_zero(age_collective_final_lambda, ms_collective_final_lambda);
    _init_if_not_zero(age_collective_final_lambda_it, ms_collective_final_lambda_it);
    _init_if_not_zero(age_start_it_waiting_time, ms_start_it_waiting_time);
}

// Function called once by the simulator to specify user-defined data fields to add to the exported data files
void create_data_schema() {
    data_add_column_int8("current_behavior");
    data_add_column_bool("diffusion_valid1");
    data_add_column_double("t");
    data_add_column_int8("nb_neighbors");
    data_add_column_int32("current_it");

    data_add_column_double("s"); // Here too
    data_add_column_double("lambda");
    data_add_column_double("avg_lambda");
}

// Function called periodically by the simulator each time data is saved (cf config parameter "save_data_period" in seconds)
void export_data() {
    data_set_value_int8("current_behavior", mydata->current_behavior);
    data_set_value_bool("diffusion_valid1", mydata->diff1.diffusion_valid);
    data_set_value_double("t", TO_FLOAT(mydata->curr_diff->t));
    data_set_value_int8("nb_neighbors", mydata->nb_neighbors);
    data_set_value_int32("current_it", mydata->current_it);

    data_set_value_double("s", TO_FLOAT(mydata->curr_diff->s[0]));    //Modify for s0,1,2 
    data_set_value_double("lambda", TO_FLOAT(mydata->diff1.lambda));
    data_set_value_double("avg_lambda", TO_FLOAT(mydata->diff1.avg_lambda));
}
#endif


int main(void) {
    pogobot_init();     // Initialization routine for the robots
    // Specify the user_init and user_step functions
    pogobot_start(setup, loop);
    // Specify the callback functions. Only called by the simulator.
    //  In particular, they serve to add data fields to the exported data files
    SET_CALLBACK(callback_global_setup, global_setup);              // Called once to initialize global values (e.g. configuration-specified constants)
    SET_CALLBACK(callback_create_data_schema, create_data_schema);  // Called once on each robot to specify the data format
    SET_CALLBACK(callback_export_data, export_data);                // Called at each configuration-specified period (e.g. every second) on each robot to register exported data
    return 0;
}

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker

