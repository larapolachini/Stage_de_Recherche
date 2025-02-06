/* TODO
 */

#include "pogosim/pogosim.h"
//#include <stdbool.h>
//#include <stdlib.h>

#include "math.h"
#include "util.h"
#include "colors.h"
#include "limmsswarm.h"
#include "dispersion.h"

REGISTER_USERDATA(USERDATA)

uint8_t const wait_for_min_nb_neighbors = 1; // = 1;

fp_t const initial_s_max_val = 1.f;
fp_t const inv_tau = 20.f; // 15.f; // 12.f; // Originally 10.f

fp_t const diffusion_convergence_threshold = 0.1;
uint16_t const diffusion_min_nb_points = 3;
fp_t const diffusion_min_abs_s = 0.e-05f;

//uint32_t const µs_initial_random_walk               = kiloticks_to_µs * 0; // 12400;
//uint32_t const µs_random_walk_choice                = kiloticks_to_µs * 1550;
//uint32_t const µs_randow_walk                       = kiloticks_to_µs * 0; // 6200;
//uint32_t const µs_handshake                         = kiloticks_to_µs * 1240; // 30;
//uint32_t const µs_diffusion                         = kiloticks_to_µs * 16275; // 31000; // 6200; // 1550; // 1860 // 930; // 465; // 6510;
//uint32_t const µs_diffusion_it                      = kiloticks_to_µs * 465; // 310; // 93;
//uint32_t const µs_diffusion_burnin                  = kiloticks_to_µs * 2325; // 1240;
//uint32_t const µs_collective_avg_lambda             = kiloticks_to_µs * 13950; // 1860;
//uint32_t const µs_collective_avg_lambda_it          = kiloticks_to_µs * 465;
//uint32_t const µs_collective_avg_avg_lambda         = kiloticks_to_µs * 13950; // 1860;
//uint32_t const µs_collective_avg_avg_lambda_it      = kiloticks_to_µs * 465;
//uint32_t const µs_start_it_waiting_time             = kiloticks_to_µs * 31; // 465;
//uint32_t µs_iteration = 0; // Set in ``setup()``

uint32_t const max_age = kiloticks_to_µs * 310; // 186; // 320; // 620; //620;

uint32_t const µs_initial_random_walk               = kiloticks_to_µs * 0; // 12400;
uint32_t const µs_random_walk_choice                = kiloticks_to_µs * 1550;
uint32_t const µs_randow_walk                       = kiloticks_to_µs * 0; // 6200;
uint32_t const µs_handshake                         = max_age * 5; // 30;
uint32_t const µs_diffusion                         = max_age * 100; // 31000; // 6200; // 1550; // 1860 // 930; // 465; // 6510;
uint32_t const µs_diffusion_it                      = max_age; // 310; // 93;
uint32_t const µs_diffusion_burnin                  = max_age * 20; // 1240;
uint32_t const µs_collective_avg_lambda             = max_age * 20; // 1860;
uint32_t const µs_collective_avg_lambda_it          = max_age;
uint32_t const µs_collective_avg_avg_lambda         = max_age * 20; // 1860;
uint32_t const µs_collective_avg_avg_lambda_it      = max_age;
uint32_t const µs_start_it_waiting_time             = kiloticks_to_µs * 31; // 465;
uint32_t µs_iteration = 0; // Set in ``setup()``


inline static bool is_number_valid(fp_t nb) {
    return !isnan(nb) && !isinf(nb);
}


void set_behavior(behavior_t behavior) {
    mydata->current_behavior = behavior;

#ifdef ENABLE_SHOW_BEHAVIOR_COLOR
    switch(mydata->current_behavior) {
        case INIT_BEHAVIOR:
            pogobot_led_setColors(1, 0, 0, 2);
            break;

        case INIT_RANDOM_WALK:
            pogobot_led_setColors(1, 0, 0, 2);
            break;

        case RANDOM_WALK:
            pogobot_led_setColors(10, 0, 0, 2);
            break;

        case PRE_DIFFUSION:
            pogobot_led_setColors(0, 3, 0, 2);
            break;

        case DIFFUSION:
            pogobot_led_setColors(0, 0, 3, 2);
            break;

        case AVG_LAMBDA:
            pogobot_led_setColors(1, 1, 0, 2);
            break;

#ifdef ENABLE_AVG_AVG_LAMBDA
        case AVG_AVG_LAMBDA:
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
    for(int8_t i = mydata->nb_neighbors-1; i >= 0; i--) {
        if(pogoticks - mydata->neighbors[i].timestamp > mydata->neighbors_age_max) {
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
    diff->t = 0;

    for(uint8_t i = 0; i < NUMBER_DIFF; i++) {
        diff->s[i] = 0;
        diff->s0[i] = 0;
#if defined(ENABLE_INIT_REMOVE_SUM_S)
        diff->sum_s0[i] = 0;
#endif
        diff->lambda_[i] = 0;
        diff->sum_t[i] = 0;
        diff->sum_t2[i] = 0;
        diff->sum_logs[i] = 0;
        diff->sum_tlogs[i] = 0;
        diff->ls_nb_points[i] = 0;
    }

    diff->lambda = 0;
    diff->sum_lambda = 0.;
    diff->avg_lambda = 0;
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
    init_pogoid();

    // Init µs_iteration
    µs_iteration = µs_randow_walk + µs_diffusion + µs_collective_avg_lambda + µs_start_it_waiting_time
#ifdef ENABLE_AVG_AVG_LAMBDA
        + µs_collective_avg_avg_lambda
#endif
#ifdef ENABLE_PRE_DIFFUSION
        + µs_diffusion
#endif
#ifdef ENABLE_HANDSHAKES
        + µs_handshake
#endif
        ;

    // Init local variables
    mydata->time_last_it = 0;
    mydata->current_it = 0;
    mydata->µs_start_it = 0;
    set_behavior(INIT_BEHAVIOR);
    mydata->behavior_start_µs = 0;
    setup_diff(&mydata->diff1);
    mydata->curr_diff = &mydata->diff1;

    // Init communication
    for(uint8_t i = 0; i < NUMBER_DIFF; i++) {
        mydata->data_to_send.val[i] = 0;
    }
    mydata->data_to_send.data_type = DATA_NULL;
	pogobot_infrared_set_power(2);
    mydata->neighbors_age_max = max_age;
    mydata->enable_message_sending = false;
    clear_all_neighbors();
#ifdef ENABLE_HANDSHAKES
    clear_known_neighbors();
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
    init_ticks();
    printf0("\ninit ok\n");
}


void behav_random_walk(void) {
    if(pogoticks - mydata->behavior_start_µs > µs_random_walk_choice) {
        mydata->behavior_start_µs = pogoticks;
        int r = rand() % 30;
        if(r==0) {
            set_motion(FORWARD);
        } else if(r==1) {
            set_motion(LEFT);
        } else if(r==2) {
            set_motion(RIGHT);
        } else {
            set_motion(STOP);
        }
    }

//    if ((uint32_t)(current_time_milliseconds() / (µs_random_walk_choice / 1000)) % 2 == 0) {
//        pogobot_led_setColors(0, 0, 255, 4);
//        pogobot_motor_set(motorL, motorFull / 2);
//        pogobot_motor_set(motorR, motorStop);
//    } else {
//        pogobot_led_setColors(255, 0, 0, 4);
//        pogobot_motor_set(motorL, motorStop);
//        pogobot_motor_set(motorR, motorFull / 2);
//    }
}


void init_diffusion(diffusion_session_t* diff, fp_t* s, diffusion_type_t type) {
    mydata->data_to_send.data_type = DATA_NULL;
    set_motion(STOP);
    clear_all_neighbors();

    // Set 'diff' as current diffusion session
    mydata->curr_diff = diff;

    // Initialize diffusion information
    diff->type = type;
    diff->t = 0;

    for(uint8_t i = 0; i < NUMBER_DIFF; i++) {
        diff->s[i] = s[i];
        diff->s0[i] = diff->s[i];
        diff->lambda_[i] = 0.;

        diff->sum_t[i] = 0.;
        diff->sum_t2[i] = 0.;
        diff->sum_logs[i] = 0.;
        diff->sum_tlogs[i] = 0.;
        diff->ls_nb_points[i] = 0;

        for(uint8_t j = 0; j < DIFFUSION_WINDOW_SIZE; j++) {
            diff->hist_logs[i][j] = 1000;
            diff->hist_t[i][j] = -1;
        }
        diff->best_mse[i] = 1000;

        mydata->data_to_send.val[i] = diff->s[i];
    }
    diff->lambda = 0;
    diff->current_diffusion_it = 0;
    diff->time_last_diff_it = pogoticks;
    diff->diffusion_valid = true;
    for(uint8_t i = 0; i < NUMBER_DIFF; i++) {
        diff->stopped_diffusion[i] = false;
    }

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
    mydata->neighbors_age_max = µs_diffusion_it;
    mydata->enable_message_sending = true;

#ifdef ENABLE_COLOR_FROM_S0
    set_color_from_s(diff->s0[0]);
#elif defined(ENABLE_COLOR_FROM_S)
    set_color_from_s(diff->s0[0]);
#elif defined(ENABLE_COLOR_FROM_SIGNS)
    set_color_from_signs(diff->s0[0]);
#elif defined(ENABLE_COLOR_FROM_AVGLAMBDA)
//#ifdef ENABLE_PRE_DIFFUSION
//    set_color_from_lambda(mydata->diff1.avg_lambda, 0);
//#else
    set_color_from_lambda(mydata->curr_diff->avg_lambda, 0);
//#endif
#elif defined(ENABLE_COLOR_SIGNS_AND_AVGLAMBDA) || defined(ENABLE_COLOR_SIGNS_AND_AVGLAMBDA_ALL_DIFF)
    set_color_from_signs(diff->s0[0]);
#elif defined(ENABLE_COLOR_NB_NEIGHBOURS)
    set_color_from_nb_neighbours();
#elif defined(ENABLE_COLOR_FROM_SIGNDELTAX)
    set_color_from_signs(diff->s0[0]);
#endif

    //    printf0("BEGIN DIFFUSION ! it=%d\n", mydata->current_it);

    //printf0("@@@@@@@@@@@@ NEW DIFFUSION ROUND @@@@@@@@@@@@\n");
}


void compute_next_s(void) {
    diffusion_session_t *const diff = mydata->curr_diff;

#if defined(ENABLE_SINGLE_DIFF)
    fp_t new_s[1] = {0};
#elif defined(ENABLE_DOUBLE_DIFF)
    fp_t new_s[2] = {0, 0};
#elif defined(ENABLE_TRIPLE_DIFF)
    fp_t new_s[3] = {0, 0, 0};
#elif defined(ENABLE_HEXA_DIFF)
    fp_t new_s[6] = {0, 0, 0, 0, 0, 0};
#endif

    for(uint8_t i = 0; i < mydata->nb_neighbors; i++) {
        if(    (diff->type == NORMAL_DIFFUSION_TYPE && mydata->neighbors[i].data_type == DATA_S)
            || (diff->type == PRE_DIFFUSION_TYPE && mydata->neighbors[i].data_type == DATA_PRE_S)) {
            for(uint8_t j = 0; j < NUMBER_DIFF; j++) {
                fp_t const val = mydata->neighbors[i].val[j];
                // Step Kernel
                if(is_number_valid(val)) {
                    new_s[j] += (fp_t)(mydata->neighbors[i].val[j] - diff->s[j]);
                }
                //printf0("    for next_s i=%d j=%d val=%d.%d new_s[j]=%d.%d\n", i, j, _float_to_2_int(val), _float_to_2_int(new_s[j])); // XXX HACK
            }

        }
    }

    for(uint8_t j = 0; j < NUMBER_DIFF; j++) {
        if (ABS(new_s[j]) <= 1e-3f) {
            diff->stopped_diffusion[j] = true;
        }
    }

#ifdef ENABLE_SINGLE_DIFF
    printf0("    next_s nb_neighbors=%d new_s=(%d.%d)\n", mydata->nb_neighbors, _float_to_2_int(new_s[0])); // XXX HACK
#elif defined(ENABLE_DOUBLE_DIFF)
    printf0("    next_s nb_neighbors=%d new_s=(%d.%d, %d.%d)\n", mydata->nb_neighbors, _float_to_2_int(new_s[0]), _float_to_2_int(new_s[1])); // XXX HACK
#elif defined(ENABLE_TRIPLE_DIFF)
    printf0("    next_s nb_neighbors=%d new_s=(%d.%d, %d.%d, %d.%d)\n", mydata->nb_neighbors, _float_to_2_int(new_s[0]), _float_to_2_int(new_s[1]), _float_to_2_int(new_s[2])); // XXX HACK
#endif

#if defined(ENABLE_COLOR_NB_NEIGHBOURS)
    set_color_from_nb_neighbours();
    //printf0("next_s: new_s %f\n", new_s);
#endif
    clear_all_neighbors();

    for(uint8_t j = 0; j < NUMBER_DIFF; j++) {
        diff->s[j] += new_s[j] / inv_tau;
    }

    // Check that s is not out of bounds
    for(uint8_t j = 0; j < NUMBER_DIFF; j++) {
        if(ABS(diff->s[j]) > 2* initial_s_max_val) {
            diff->diffusion_valid = false;
            diff->s[j] = 0./0.; // NaN
        }
    }
}



//fp_t compute_MSE(fp_t lambda, fp_t v, fp_t* hist_logs, fp_t* hist_t) {
//fp_t compute_MSE(fp_t lambda, fp_t v, fp_t hist_logs[DIFFUSION_WINDOW_SIZE], fp_t hist_t[DIFFUSION_WINDOW_SIZE]) {
fp_t compute_MSE(fp_t lambda, fp_t v, uint8_t j) {
    diffusion_session_t *const diff = mydata->curr_diff;

    fp_t mse = 0;
    fp_t nb_points = 0;
    for(uint8_t i = 0; i < DIFFUSION_WINDOW_SIZE; i++) {
        fp_t const logs = diff->hist_logs[j][i];
        fp_t const t = diff->hist_t[j][i];
        if(logs >= 0 || t < 0)
            continue;
        fp_t const err = (-lambda * t + LOG(v)) - (logs);
        //fp_t const err = (-lambda * t + v) - (logs);
        if(is_number_valid(err)) {
            mse += err * err;
            nb_points += 1;
        }
    }
    if(nb_points > 0)
        return mse/nb_points;
    else
        return 2000;
}


void compute_lambda_v_leastsquaresMSE(void) {
    diffusion_session_t *const diff = mydata->curr_diff;
    fp_t const t = (fp_t)diff->current_diffusion_it / inv_tau;
    diff->t = t;

    fp_t nb_valid_lambda = 0.f;
    fp_t sum_all_lambda = 0.f;

    //if(!diff->diffusion_valid || diff->stopped_diffusion || diff->current_diffusion_it < µs_diffusion_burnin / µs_diffusion_it) {
    if(!diff->diffusion_valid || diff->current_diffusion_it < µs_diffusion_burnin / µs_diffusion_it) {
        return;
    }

    for(uint8_t i = 0; i < NUMBER_DIFF; i++) {
        //printf0("DEBUG compute_lambda_v_leastsquaresMSE: i=%d  s[i]=%d.%d  stopped=%d", i, _float_to_2_int(diff->s[i]), diff->stopped_diffusion[i]);
        if (diff->stopped_diffusion[i])
            continue;
        fp_t mse = 1000.f;
        if (ABS(diff->s[i]) <= 1e-7f) {
            diff->stopped_diffusion[i] = true;
            continue;
        }
        fp_t const logs = LOG(ABS(diff->s[i]));
        if (!is_number_valid(logs) || ABS(logs) <= 0.f) {
            diff->stopped_diffusion[i] = true;
            continue;
        }
        diff->sum_t[i] += t;
        diff->sum_t2[i] += t * t;
        diff->sum_logs[i] += logs;
        diff->sum_tlogs[i] += t * logs;
        diff->ls_nb_points[i] += 1.0f;

        diff->hist_logs[i][(uint8_t)(diff->ls_nb_points[i]) % DIFFUSION_WINDOW_SIZE] = logs;
        diff->hist_t[i][(uint8_t)(diff->ls_nb_points[i]) % DIFFUSION_WINDOW_SIZE] = t;

        if(diff->ls_nb_points[i] > 3.0f) {
            fp_t const _lambda = -(diff->ls_nb_points[i] * diff->sum_tlogs[i] - diff->sum_t[i] * diff->sum_logs[i]) / (diff->ls_nb_points[i] * diff->sum_t2[i] - diff->sum_t[i] * diff->sum_t[i]);
            fp_t const _v = EXP( (diff->sum_logs[i] - _lambda * diff->sum_t[i]) / diff->ls_nb_points[i] );

            if(!is_number_valid(_lambda) || !is_number_valid(_v)) {
                // ...
                //diff->diffusion_valid = false;
            } else {
                if(diff->ls_nb_points[i] >= DIFFUSION_WINDOW_SIZE) {
                    //mse = compute_MSE(_lambda, _v, diff->hist_logs[i], diff->hist_t[i]);
                    mse = compute_MSE(_lambda, _v, i);
                    if(mse < diff->best_mse[i]) {
                        diff->best_mse[i] = mse;
                        diff->lambda_[i] = _lambda;
                        //diff->v[i] = _v;
                    }
                } else {
                    diff->lambda_[i] = _lambda;
                    //diff->v[i] = _v;
                }

                if(is_number_valid(diff->lambda_[i]) && diff->best_mse[i] < 100.) {
                    sum_all_lambda += diff->lambda_[i];
                    nb_valid_lambda++;
                }
            }

        }

    }

    fp_t const _lambda = sum_all_lambda / nb_valid_lambda;
    if(!is_number_valid(_lambda) || nb_valid_lambda == 0) {
//            diff->diffusion_valid = false;
    } else {
        diff->lambda = _lambda;
    }

//        printf0("DEBUG diff: valid=%d t=%f s=%f logs=%f orig_t=%f orig_logx=%f lambda=%f v=%f cv=%f \n", diff->diffusion_valid, (fp_t)t, (fp_t)diff->s, (fp_t)logs, (fp_t)diff->diffusion_orig_t, (fp_t)diff->diffusion_orig_logx, (fp_t)diff->lambda, (fp_t)diff->v, (fp_t)diff->cv);
}


void behav_diffusion(void) {
    diffusion_session_t *const diff = mydata->curr_diff;
    set_motion(STOP);

    if(pogoticks - diff->time_last_diff_it >= µs_diffusion_it) {
        compute_next_s();
        if(diff->type != PRE_DIFFUSION_TYPE) {
            compute_lambda_v_leastsquaresMSE();
        }

        // Update broadcasting information
        for(uint8_t i = 0; i < NUMBER_DIFF; i++) {
            mydata->data_to_send.val[i] = diff->s[i];
        }

        diff->current_diffusion_it++;
        diff->time_last_diff_it = pogoticks;

        printf0("    step %d t=%d.%d s_0=%d.%d ls_nb_points_0=%lu  stopped=%d,%d,%d\n", diff->current_diffusion_it, _float_to_2_int(diff->t), _float_to_2_int(diff->s[0]), (long unsigned int) diff->ls_nb_points[0], diff->stopped_diffusion[0], diff->stopped_diffusion[1], diff->stopped_diffusion[2]); // XXX HACK

#if defined(ENABLE_COLOR_FROM_LAMBDA)
        set_color_from_lambda(diff->lambda, 0);
#elif defined(ENABLE_COLOR_FROM_S)
        set_color_from_s(diff->s[0]);
#elif defined(ENABLE_COLOR_FROM_SIGNS)
        set_color_from_signs(diff->s[0]);
#elif defined(ENABLE_COLOR_FROM_AVGLAMBDA)
//#ifdef ENABLE_PRE_DIFFUSION
//        set_color_from_lambda(mydata->diff1.avg_lambda, 0);
//#else
        set_color_from_lambda(mydata->curr_diff->avg_lambda, 0);
//#endif
#elif defined(ENABLE_COLOR_SIGNS_AND_AVGLAMBDA)
        set_color_from_signs(diff->s[0]);
#elif defined(ENABLE_COLOR_SIGNS_AND_AVGLAMBDA_ALL_DIFF)
#if defined(ENABLE_SINGLE_DIFF)
        set_color_from_signs(diff->s[0]);
#elif defined(ENABLE_DOUBLE_DIFF)
        if((diff->current_diffusion_it / 7) % 2 == 0) {
            set_color_from_signs(diff->s[0]);
        } else {
            set_color_from_signs(diff->s[1]);
        }
#elif defined(ENABLE_TRIPLE_DIFF)
        if((diff->current_diffusion_it / 7) % 3 == 0) {
            set_color_from_signs(diff->s[0]);
        } else if((diff->current_diffusion_it / 7) % 3 == 1) {
            set_color_from_signs(diff->s[1]);
        } else {
            set_color_from_signs(diff->s[2]);
        }
#elif defined(ENABLE_HEXA_DIFF)
        if((diff->current_diffusion_it / 3) % 6 == 0) {
            set_color_from_signs(diff->s[0]);
        } else if((diff->current_diffusion_it / 3) % 6 == 1) {
            set_color_from_signs(diff->s[1]);
        } else if((diff->current_diffusion_it / 3) % 6 == 2) {
            set_color_from_signs(diff->s[2]);
        } else if((diff->current_diffusion_it / 3) % 6 == 3) {
            set_color_from_signs(diff->s[3]);
        } else if((diff->current_diffusion_it / 3) % 6 == 4) {
            set_color_from_signs(diff->s[4]);
        } else {
            set_color_from_signs(diff->s[5]);
        }
#endif

#elif defined(ENABLE_COLOR_NB_NEIGHBOURS)
        set_color_from_nb_neighbours();
#endif

    }
}



void end_diffusion(void) {
    diffusion_session_t *const diff = mydata->curr_diff;

    // Check if diffusion converged close to 0.
    //if(ABS(diff->s) - 0. > diffusion_convergence_threshold) {
    //if(ABS(diff->s) - 0. > diffusion_convergence_threshold || diff->ls_nb_points < diffusion_min_nb_points) {
    if( (diffusion_convergence_threshold > 0. && ABS(diff->s[0]) - 0. > diffusion_convergence_threshold)
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
        diff->lambda = 0;
        diff->diffusion_valid = false;
    }

    if(!diff->diffusion_valid) {
        diff->lambda = 0.0/0.0; // NaN
    }

    printf0("END DIFFUSION ! it=%d id=%d diffit=%d lambda=%d.%d s[0]=%d.%d ls_nb_points_0=%lu diffusion_valid=%d\n", mydata->current_it, pogoid, diff->current_diffusion_it, _float_to_2_int(diff->lambda), _float_to_2_int(diff->s[0]), (long unsigned int)diff->ls_nb_points[0], diff->diffusion_valid);
    fp_t const condition_convergence = ABS(diff->s[0]) - 0.;
    printf0("  diffusion_convergence_threshold=%d.%d  condition_convergence=%d.%d\n", _float_to_2_int(diffusion_convergence_threshold), _float_to_2_int(condition_convergence));
}


void init_coll_avg_lambda(void) {
    mydata->data_to_send.data_type = DATA_NULL;
    set_motion(STOP);
    clear_all_neighbors();
    mydata->time_last_coll_avg_lambda_it = pogoticks;

    diffusion_session_t *const diff = mydata->curr_diff;
    if(diff->diffusion_valid) {
        mydata->data_to_send.val[0] = diff->lambda;
        mydata->data_to_send.data_type = DATA_LAMBDA;
    } else {
        mydata->data_to_send.data_type = DATA_NULL;
    }
    mydata->neighbors_age_max = µs_collective_avg_lambda_it;
    mydata->enable_message_sending = true;
}

void behav_coll_avg_lambda(void) {
    set_motion(STOP);
    if(pogoticks - mydata->time_last_coll_avg_lambda_it >= µs_collective_avg_lambda_it) {
        // Compute lambda through collective averaging
        uint8_t nb_neighbors = mydata->nb_neighbors;
        uint8_t used_neighbors = nb_neighbors;
        fp_t tmp = 0;
        if(mydata->curr_diff->diffusion_valid) {
            tmp += mydata->curr_diff->lambda;
            ++used_neighbors;
        }
        for(uint8_t i = 0; i < nb_neighbors; i++) {
            if(mydata->neighbors[i].data_type == DATA_LAMBDA) {
                tmp += mydata->neighbors[i].val[0];
            } else {
                --used_neighbors;
            }
        }

        clear_all_neighbors();
        if(used_neighbors > 0) {
            tmp /= (fp_t)(used_neighbors);
            if(is_number_valid(tmp)) {
                mydata->curr_diff->lambda = tmp;
                mydata->data_to_send.val[0] = mydata->curr_diff->lambda;
                mydata->data_to_send.data_type = DATA_LAMBDA;
            }
        }

        mydata->time_last_coll_avg_lambda_it = pogoticks;

        printf0("    avg_lambda: lambda=%d.%d diffusion_valid=%d\n", _float_to_2_int(mydata->curr_diff->lambda), mydata->curr_diff->diffusion_valid);
    }
}

void end_coll_avg_lambda(void) {
#ifndef ENABLE_AVG_AVG_LAMBDA
    diffusion_session_t *const diff = mydata->curr_diff;
    if(is_number_valid(diff->lambda)) {
        diff->sum_lambda += diff->lambda * 1;
        ++diff->current_avg_it;
        diff->avg_lambda = (fp_t) diff->sum_lambda / (diff->current_avg_it);
#if defined(ENABLE_COLOR_SIGNS_AND_AVGLAMBDA) || defined(ENABLE_COLOR_SIGNS_AND_AVGLAMBDA_ALL_DIFF)
        set_color_from_lambda(diff->avg_lambda, 0);
#endif
    }

    printf0("    end_coll_avg_lambda: sum_lambda=%d.%d avg_lambda=%d.%d\n", _float_to_2_int(diff->sum_lambda), _float_to_2_int(diff->avg_lambda));
#endif
}


#ifdef ENABLE_AVG_AVG_LAMBDA
void init_coll_avg_avg_lambda(void) {
    mydata->data_to_send.data_type = DATA_NULL;
    set_motion(STOP);
    clear_all_neighbors();

    diffusion_session_t *const diff = mydata->curr_diff;
    mydata->time_last_coll_avg_avg_lambda_it = pogoticks;

    if(is_number_valid(diff->lambda)) {
        diff->sum_lambda += diff->lambda * 1;
        ++diff->current_avg_it;
        diff->avg_lambda = (fp_t) (diff->sum_lambda / (mydata->current_it + 1));
    }

    if(is_number_valid(diff->avg_lambda)) {
        mydata->data_to_send.val[0] = diff->avg_lambda;
        mydata->data_to_send.data_type = DATA_AVG_LAMBDA;
    } else {
        mydata->data_to_send.data_type = DATA_NULL;
    }
    mydata->neighbors_age_max = µs_collective_avg_avg_lambda_it;
    mydata->enable_message_sending = true;
}

void behav_coll_avg_avg_lambda(void) {
    diffusion_session_t *const diff = mydata->curr_diff;
    set_motion(STOP);

    if(pogoticks - mydata->time_last_coll_avg_avg_lambda_it >= µs_collective_avg_avg_lambda_it) {
        uint8_t nb_neighbors = mydata->nb_neighbors;
        uint8_t used_neighbors = nb_neighbors;
        fp_t tmp = 0;
        if(is_number_valid(diff->avg_lambda)) {
            tmp += diff->avg_lambda;
            ++used_neighbors;
        }
        for(uint8_t i = 0; i < nb_neighbors; i++) {
            if(mydata->neighbors[i].data_type == DATA_AVG_LAMBDA) {
                tmp += mydata->neighbors[i].val[0];
            } else {
                --used_neighbors;
            }
        }

        clear_all_neighbors();
        if(used_neighbors > 0) {
            tmp /= (fp_t)(used_neighbors);
            diff->avg_lambda = (fp_t)tmp;
            if(is_number_valid(diff->avg_lambda)) {
                mydata->data_to_send.val[0] = diff->avg_lambda;
                mydata->data_to_send.data_type = DATA_AVG_LAMBDA;
            }
        }
        mydata->time_last_coll_avg_avg_lambda_it = pogoticks;
    }

#if defined(ENABLE_COLOR_SIGNS_AND_AVGLAMBDA) || defined(ENABLE_COLOR_SIGNS_AND_AVGLAMBDA_ALL_DIFF)
    set_color_from_lambda(diff->avg_lambda, 0);
#elif defined(ENABLE_COLOR_FROM_AVGLAMBDA)
    set_color_from_lambda(mydata->diff1.avg_lambda, 0);
#endif

    printf0("    avg_avg_lambda: avg_lambda=%d.%d\n", _float_to_2_int(diff->avg_lambda));
}
#endif


#ifdef ENABLE_HANDSHAKES
void behav_handshake(void) {
    mydata->enable_message_sending = true;
    mydata->hmsg_to_send.data_type = DATA_HANDSHAKE;
    set_motion(STOP);

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
#ifdef ENABLE_DEBUG_MSG
    printf0("Exchanging Handshakes. nb_peers=%d nb_known_neighbors=%d \n", nb_peers, mydata->nb_known_neighbors);
#endif
}
#endif



void end_iteration(void) {
#if !defined(ENABLE_AVG_AVG_LAMBDA)
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
        diff->sum_lambda = 0.;
        diff->avg_lambda = 0;
        diff->current_avg_it = 0;
//            printf0("SUM(S(0)):%f  x:%f  lambda:%f\n", diff->sum_s0, diff->s[0], diff->lambda);
    }
#endif

#ifdef ENABLE_COLOR_FROM_AVGLAMBDA
    set_color_from_lambda(mydata->diff1.avg_lambda, 0);
#elif defined(ENABLE_COLOR_SIGNS_AND_AVGLAMBDA) || defined(ENABLE_COLOR_SIGNS_AND_AVGLAMBDA_ALL_DIFF)
    set_color_from_lambda(mydata->curr_diff->avg_lambda, 0);
#else
    pogobot_led_setColors(3, 3, 3, 0);
#endif
    mydata->current_it++;

    printf0("END ITERATION: avg_lambda:%d.%d\n", _float_to_2_int(mydata->curr_diff->avg_lambda)); // XXX HACK
    mydata->time_last_it = pogoticks;
    mydata->µs_start_it = pogoticks - µs_initial_random_walk;

#ifdef ENABLE_INITIAL_WAIT_FOR_NEIGHBOR
    set_behavior(INIT_BEHAVIOR);
#endif
}


void iteration(void) {
#define iteration_reached_behavior(b) (µs_elapsed_since_start_it < (index += b))
    uint64_t index = 0;
    uint64_t const ticks_after_init_phase = pogoticks - µs_initial_random_walk;

    // Determine current behavior
    uint64_t const µs_elapsed_since_start_it = ticks_after_init_phase - mydata->µs_start_it;

    //printf0("Debug iteration: pogoticks=%u  elapsed_since_start_it=%u\n", (uint16_t) (pogoticks / 1000000), (uint16_t) (µs_elapsed_since_start_it / 1000000));

#ifdef ENABLE_INITIAL_WAIT_FOR_NEIGHBOR
    if(mydata->current_behavior == INIT_BEHAVIOR) {
        if((pogoticks / µs_random_walk_choice) % 2 == 0) {
            pogobot_led_setColors(10, 0, 0, 0);
        } else {
            pogobot_led_setColors(0, 0, 0, 0);
        }
        printf0("Searching for neighbors... (found %d/%d)\n", mydata->nb_neighbors, wait_for_min_nb_neighbors);
        mydata->data_to_send.data_type = DATA_HEARTBEAT;
        mydata->neighbors_age_max = kiloticks_to_µs * 31 * 60;
        mydata->enable_message_sending = true;
        if(mydata->nb_neighbors >= wait_for_min_nb_neighbors) {
            printf0("Found %d/%d neighbor !\n", mydata->nb_neighbors, wait_for_min_nb_neighbors);
            clear_all_neighbors();
            set_behavior(MISC_BEHAVIOR);
            init_ticks();
        }
        return;
    }

#elif defined(ENABLE_PHOTO_START)
    if(mydata->current_behavior == INIT_BEHAVIOR) {
        mydata->enable_message_sending = false;
        int16_t const data_b  = pogobot_photosensors_read(0);
        int16_t const data_fl = pogobot_photosensors_read(1);
        int16_t const data_fr = pogobot_photosensors_read(2);

        // Stopping if the difference between the last value and the current value is more than the threshold
        int16_t const diff_b  = data_b  - mydata->last_data_b;  // Positive if data > last_data, i.e more light than before
        int16_t const diff_fl = data_fl - mydata->last_data_fl;
        int16_t const diff_fr = data_fr - mydata->last_data_fr;
        mydata->last_data_b  = data_b;
        mydata->last_data_fl = data_fl;
        mydata->last_data_fr = data_fr;

#ifdef ENABLE_DEBUG_MSG
        printf0("Current light level: %d %d %d \n", diff_b, diff_fl, diff_fr);
#endif

        if(diff_b >= LIGHT_THRESHOLD || diff_fl >= LIGHT_THRESHOLD || diff_fr >= LIGHT_THRESHOLD) {
            printf0("Detected light is above threshold. Starting experiment!\n");
            clear_all_neighbors();
            set_behavior(MISC_BEHAVIOR);
            init_ticks();
        }
        return;
    }
#endif

#ifdef SHOW_AVGLAMBDA_IN_LED1
    set_color_from_lambda(mydata->curr_diff->avg_lambda, 1);
#endif

    if(pogoticks < µs_initial_random_walk) {
        if(mydata->current_behavior != INIT_RANDOM_WALK) {
            mydata->behavior_start_µs = pogoticks;
#ifdef ENABLE_INITIAL_WAIT_FOR_NEIGHBOR
            mydata->enable_message_sending = true;
            mydata->data_to_send.data_type = DATA_HEARTBEAT;
            mydata->neighbors_age_max = kiloticks_to_µs * 31 * 60;
#else
            mydata->enable_message_sending = false;
#endif
            start_dispersion();
            printf0("Initial random walk\n");
            pogobot_led_setColors(0, 0, 0, 0);
        }
        set_behavior(INIT_RANDOM_WALK);
        mydata->time_last_it = pogoticks;

    } else if(iteration_reached_behavior(µs_start_it_waiting_time)) {
        if(mydata->current_behavior != WAITING_TIME) {
            mydata->behavior_start_µs = pogoticks;
            set_motion(STOP);
#ifdef ENABLE_INITIAL_WAIT_FOR_NEIGHBOR
            mydata->enable_message_sending = true;
            mydata->data_to_send.data_type = DATA_HEARTBEAT;
            mydata->neighbors_age_max = kiloticks_to_µs * 31 * 60;
#else
            mydata->enable_message_sending = false;
#endif
            printf0("BEGIN ITERATION it=%d\n", mydata->current_it);
            printf0("  it=%d WAITING_TIME\n", mydata->current_it);
            pogobot_led_setColors(3, 3, 3, 0);
        }
        set_behavior(WAITING_TIME);

    } else if(iteration_reached_behavior(µs_randow_walk)) {
        if(mydata->current_behavior != RANDOM_WALK) {
            mydata->behavior_start_µs = pogoticks;
#ifdef ENABLE_INITIAL_WAIT_FOR_NEIGHBOR
            mydata->enable_message_sending = true;
            mydata->data_to_send.data_type = DATA_HEARTBEAT;
            mydata->neighbors_age_max = kiloticks_to_µs * 31 * 60;
#else
            mydata->enable_message_sending = false;
#endif
            start_dispersion();
            printf0("  it=%d RANDOM_WALK\n", mydata->current_it);
        }
        set_behavior(RANDOM_WALK);

#ifdef ENABLE_HANDSHAKES
    } else if(iteration_reached_behavior(µs_handshake)) {
        if(mydata->current_behavior != HANDSHAKE) {
            set_motion(STOP);
            printf0("Exchanging Handshakes\n");
            clear_all_neighbors();
            clear_known_neighbors();
        }
        set_behavior(HANDSHAKE);
#endif

#ifdef ENABLE_PRE_DIFFUSION
    } else if(iteration_reached_behavior(µs_diffusion)) {
        if(mydata->current_behavior != PRE_DIFFUSION) {
            fp_t s[NUMBER_DIFF];
            for(uint8_t i = 0; i < NUMBER_DIFF; i++) {
                s[i] = ((uint32_t)(rand()+pogoid) % 2 == 0) ? -initial_s_max_val : initial_s_max_val;
            }
            printf0("  it=%d PRE_DIFFUSION\n", mydata->current_it);
            init_diffusion(&mydata->diff1, s, PRE_DIFFUSION_TYPE);
        }
        set_behavior(PRE_DIFFUSION);
#endif

    } else if(iteration_reached_behavior(µs_diffusion)) {
        if(mydata->current_behavior != DIFFUSION) {
            fp_t s[NUMBER_DIFF];
            for(uint8_t i = 0; i < NUMBER_DIFF; i++) {
            //fp_t s = ((uint32_t)(rand_soft()+kilo_uid) % 2 == 0) ? -initial_s_max_val : initial_s_max_val;
                s[i] = (pogoid % 2 == 0) ? -initial_s_max_val : initial_s_max_val;
#ifdef ENABLE_INIT_REMOVE_SUM_S
                s[i] -= mydata->diff1.sum_s0[i];
#elif defined(ENABLE_PRE_DIFFUSION)
                s[i] = mydata->diff1.s0[i] - mydata->diff1.s[i];
#endif
            }
            printf0("  it=%d DIFFUSION\n", mydata->current_it);
            init_diffusion(&mydata->diff1, s, NORMAL_DIFFUSION_TYPE);
        }
        set_behavior(DIFFUSION);

#ifdef ENABLE_INIT_REMOVE_SUM_S
    } else if(mydata->current_it > 0 && iteration_reached_behavior(µs_collective_avg_lambda)) {
#else
    } else if(iteration_reached_behavior(µs_collective_avg_lambda)) {
#endif
        if(mydata->current_behavior != AVG_LAMBDA) {
            end_diffusion();
            printf0("  it=%d AVG_LAMBDA\n", mydata->current_it);
            init_coll_avg_lambda();
        }
        set_behavior(AVG_LAMBDA);

#ifdef ENABLE_AVG_AVG_LAMBDA
#ifdef ENABLE_INIT_REMOVE_SUM_S
    } else if(mydata->current_it > 0 && iteration_reached_behavior(µs_collective_avg_avg_lambda)) {
#else
    } else if(iteration_reached_behavior(µs_collective_avg_avg_lambda)) {
#endif
        if(mydata->current_behavior != AVG_AVG_LAMBDA) {
            end_coll_avg_lambda();
            printf0("  it=%d AVG_AVG_LAMBDA\n", mydata->current_it);
            init_coll_avg_avg_lambda();
        }
        set_behavior(AVG_AVG_LAMBDA);
#endif

    } else {
        // End of an iteration !
        end_iteration();
    }

    if(mydata->current_behavior == RANDOM_WALK || mydata->current_behavior == INIT_RANDOM_WALK) {
        behav_random_walk();
        //behav_dispersion();
#ifdef ENABLE_HANDSHAKES
    } else if(mydata->current_behavior == HANDSHAKE) {
        behav_handshake();
#endif
#ifdef ENABLE_PRE_DIFFUSION
    } else if(mydata->current_behavior == PRE_DIFFUSION) {
        behav_diffusion();
#endif
    } else if(mydata->current_behavior == DIFFUSION) {
        behav_diffusion();
    } else if(mydata->current_behavior == AVG_LAMBDA) {
        behav_coll_avg_lambda();
#ifdef ENABLE_AVG_AVG_LAMBDA
    } else if(mydata->current_behavior == AVG_AVG_LAMBDA) {
        behav_coll_avg_avg_lambda();
#endif
    }
}

bool send_message(void) {
    if(!mydata->enable_message_sending) {
        // Not allowed to send messages!
        return false;
    }
    // Random delay
    //msleep(rand() % max_delay_send_message);

    // Send message
    switch(mydata->current_behavior) {
#ifdef ENABLE_HANDSHAKES
        case HANDSHAKE:
            pogobot_infrared_sendLongMessage_omniGen( (uint8_t *)(&mydata->hmsg_to_send), sizeof(mydata->hmsg_to_send) );
            break;
#endif

        default:
            //pogobot_infrared_sendMessageAllDirection( 0x1234, (uint8_t *)(&mydata->data_to_send), sizeof(mydata->data_to_send) );
            pogobot_infrared_sendLongMessage_omniGen( (uint8_t *)(&mydata->data_to_send), sizeof(mydata->data_to_send) );
            break;
    }
    return true;
}


void process_message(message_t* mr) {
    // Check if payload has the correct size
    size_t const m_size = mr->header.payload_length;
    if(m_size < sizeof(message_data_t))
        return;

    // Check if the message if not from the focal robot
    uint16_t const sender_id = mr->header._sender_id;
    if(sender_id == pogoid)
        return;

    // Get message data
    message_data_t const* data = (message_data_t*)(&( mr->payload ));

#ifdef ENABLE_HANDSHAKES
#ifdef ENABLE_INITIAL_WAIT_FOR_NEIGHBOR
    if(data->data_type != DATA_HANDSHAKE && data->data_type != DATA_HEARTBEAT) {
#else
    if(data->data_type != DATA_HANDSHAKE) {
#endif
        // Check if neighbor is known
        if(!is_neighbor_known(sender_id)) {
            // Unknown neighbor. Ignore message
            return;
        }
    }
#endif

    // Further process message according to current behavior
    switch(mydata->current_behavior) {
#ifdef ENABLE_INITIAL_WAIT_FOR_NEIGHBOR
        case INIT_BEHAVIOR:
            // XXX
            //if(data->data_type != DATA_HEARTBEAT) // Check if message if of the correct type
            //    return;
            break;
#endif

#ifdef ENABLE_HANDSHAKES
        case HANDSHAKE:
            if(data->data_type != DATA_HANDSHAKE) // Check if message if of the correct type
                return;
            break;
#endif

        case PRE_DIFFUSION:
            if(data->data_type != DATA_PRE_S) // Check if message if of the correct type
                return;
            break;

        case DIFFUSION:
            if(data->data_type != DATA_S) // Check if message if of the correct type
                return;
            break;

        case AVG_LAMBDA:
            if(data->data_type != DATA_LAMBDA) // Check if message if of the correct type
                return;
            break;

#ifdef ENABLE_AVG_AVG_LAMBDA
        case AVG_AVG_LAMBDA:
            if(data->data_type != DATA_AVG_LAMBDA) // Check if message if of the correct type
                return;
            break;
#endif

        default:
            break;
    }

    // search the neighbor list by ID
    uint16_t i;
    for(i = 0; i < mydata->nb_neighbors; i++) {
        if(mydata->neighbors[i].id == sender_id)
            break;
    }

    // If the neighbor is new, add it to the list
    if(i == mydata->nb_neighbors) {
        if(mydata->nb_neighbors < MAXN-1)   // If we have too many neighbors, overwrite the last entry
            mydata->nb_neighbors++;
    }

    // Update corresponding neighbor entry
    mydata->neighbors[i].id = sender_id;
    mydata->neighbors[i].timestamp = pogoticks;
    mydata->neighbors[i].data_type = data->data_type;

    // Update internal values depending on current behavior
    switch(mydata->current_behavior) {
#ifdef ENABLE_HANDSHAKES
        case HANDSHAKE: {
            handshake_message_data_t const* hmsg = (handshake_message_data_t*) (&( mr->payload ));
            uint8_t am_i_a_peer = false;
            uint8_t nb_peers = hmsg->nb_peers;
            if(nb_peers > MAXN)
                nb_peers = MAXN;
            for(uint8_t j = 0; j < nb_peers; j++) {
                am_i_a_peer |= hmsg->peers[j] == pogoid; // Bit-wise OR. Still works because the condition results in either 0 or 1
            }
            if(am_i_a_peer) {
                // Check if the neighbor is not already known
                if(!is_neighbor_known(sender_id) && mydata->nb_known_neighbors < MAXN-1) {
                    // If neighbor is unknown, add it to the list of known neighbors
                    mydata->known_neighbors_uid[mydata->nb_known_neighbors] = sender_id;
                    mydata->nb_known_neighbors++;          // sloppy but better than overflow
                }
            }
            break;
        }

#endif

        default:
            for(uint8_t j = 0; j < NUMBER_DIFF; j++) {
                mydata->neighbors[i].val[j] = data->val[j];
            }
            break;
    }


#ifdef ENABLE_DEBUG_MSG
#if defined(ENABLE_SINGLE_DIFF)
    printf0("    recv msg from neighbor:%d/%d id=%u val0=%d.%d type=%d\n", i, mydata->nb_neighbors, mydata->neighbors[i].id, _float_to_2_int(mydata->neighbors[i].val[0]), (int)mydata->neighbors[i].data_type);
#elif defined(ENABLE_DOUBLE_DIFF)
    printf0("    recv msg from neighbor:%d/%d id=%u val0=%d.%d val1=%d.%d type=%d\n", i, mydata->nb_neighbors, mydata->neighbors[i].id, _float_to_2_int(mydata->neighbors[i].val[0]), _float_to_2_int(mydata->neighbors[i].val[1]), (int)mydata->neighbors[i].data_type);
#elif defined(ENABLE_TRIPLE_DIFF)
    printf0("    recv msg from neighbor:%d/%d id=%u val0=%d.%d val1=%d.%d val1=%d.%d type=%d\n", i, mydata->nb_neighbors, mydata->neighbors[i].id, _float_to_2_int(mydata->neighbors[i].val[0]), _float_to_2_int(mydata->neighbors[i].val[1]), _float_to_2_int(mydata->neighbors[i].val[2]), (int)mydata->neighbors[i].data_type);
#endif
#endif
}



void loop(void) {
    // Update ticks according to timer
    update_ticks();

    // Purge neighbors if needed
    purge_old_neighbors();

    // Main behavior of the robots
    iteration();
}


#ifdef SIMULATOR
// Function called once by the simulator to specify user-defined data fields to add to the exported data files
void create_data_schema() {
    data_add_column_int8("current_behavior");
    data_add_column_bool("diffusion_valid1");
    data_add_column_double("t");
    data_add_column_int8("nb_neighbors");

    data_add_column_double("s");
    data_add_column_double("lambda");
    data_add_column_double("avg_lambda");
}

// Function called periodically by the simulator each time data is saved (cf config parameter "save_data_period" in seconds)
void export_data() {
    data_set_value_int8("current_behavior", mydata->current_behavior);
    data_set_value_bool("diffusion_valid1", mydata->diff1.diffusion_valid);
    data_set_value_double("t", mydata->curr_diff->t);
    data_set_value_int8("nb_neighbors", mydata->nb_neighbors);

    data_set_value_double("s", mydata->curr_diff->s[0]);
    data_set_value_double("lambda", mydata->diff1.lambda);
    data_set_value_double("avg_lambda", mydata->diff1.avg_lambda);
}
#endif


int main(void) {
    pogobot_init();
    pogobot_start(setup, loop);
    SET_CALLBACK(callback_create_data_schema, create_data_schema);
    SET_CALLBACK(callback_export_data, export_data);
    return 0;
}

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
