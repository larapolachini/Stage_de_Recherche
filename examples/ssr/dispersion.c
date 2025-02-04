/* TODO
 */

#include "pogosim/pogosim.h"
#include <math.h>
#include "dispersion.h"
#include "util.h"



double const prob_moving = 0.75;
uint64_t const base_tumble_time = 31 * kiloticks_to_µs;
double const offset = -5;
double const scaling = 64;
double const d_optim = 48;
uint64_t const lower_tumble_time = 0 * 31 * kiloticks_to_µs;
uint64_t const upper_tumble_time = 2 * 31 * kiloticks_to_µs;

uint8_t const dist_with_no_neighbors = 255; // big enough

double uniform(void);
double rand_normal(double mu, double sigma);


double uniform(void) {
    return ((double)rand()+1.0)/((double)RAND_MAX+2.0);
}

double rand_normal(double mu, double sigma) {
    double z = sqrt(-2.0*log(uniform())) * sin(2.0*M_PI*uniform());
    return mu + sigma*z;
}

void setup_dispersion(void) {
//    for(;;) {
//        mydata->tumble_time = fabs(base_tumble_time + rand_normal(0, 1) * 32); // 2 sec // not too big
//        if (mydata->tumble_time < upper_tumble_time && mydata->tumble_time > lower_tumble_time) break;
//    }
    mydata->run_time = 64; // 255;
    mydata->direction = rand() % 2;
    mydata->prob = ((double)rand() / 255.0f) * ((double)rand() / 255.0f);
}

void start_dispersion(void) {
    mydata->run_time = 64; // 255;
    mydata->direction = rand() % 2;
    mydata->prob = ((double)rand() / 255.0f) * ((double)rand() / 255.0f);
    mydata->last_pogoticks_dispersion = pogoticks;
    mydata->flag_dispersion = 1;
}

// TODO
void behav_dispersion(void) {
//    /*
//    run and tumble algorithm
//    */
////    get_d_min();
////    get_d_max();
//    
//    mydata->cycle_dispersion = pogoticks - mydata->last_pogoticks_dispersion;
//
//    if (mydata->flag_dispersion == 0) {
////        for(;;) {
////            mydata->tumble_time = fabs(base_tumble_time + rand_normal(0, 1) * 32); // 2 sec // not too big
////            if (mydata->tumble_time < upper_tumble_time && mydata->tumble_time > lower_tumble_time) break;
////        }
//        mydata->tumble_time = fabs(base_tumble_time + rand_normal(0, 1) * 32); // 2 sec // not too big
//        if(pogoticks - mydata->behavior_start_µs <= kiloticks_to_µs * 1550)
//            return;
//        mydata->behavior_start_µs = pogoticks;
//
//        double const is_positive_frustration = 1.0f - mydata->d_min / d_optim;
//        if (is_positive_frustration > 0) // d_min < d_optim
//            mydata->frustration = is_positive_frustration;
//        else if (mydata->d_min < dist_with_no_neighbors) // if it has some neighbors, it doesn't have to move // d_min >= d_optim
//            mydata->frustration = 0;
//        else // if it has no neighbors, it needs to explore // d_min == dist_with_no_neighbors
//            mydata->frustration = is_positive_frustration * -1.0f;
//
//        double const is_positive_run_time = (double)offset + (double)mydata->frustration * (double)scaling;
//        if (is_positive_run_time > 0)
//            mydata->run_time = is_positive_run_time;
//        else
//            mydata->run_time = 0;
//
//        mydata->direction = rand() % 2;
//        mydata->prob = ((double)rand() / 255.0f) * ((double)rand() / 255.0f);
//        mydata->flag_dispersion = 1;
//
//    } else if (mydata->prob < prob_moving) { // move
//        if (mydata->cycle_dispersion < mydata->tumble_time) {
//            // tumble state
//            pogobot_led_setColors(3,0,0, 4); // red
//            if(mydata->direction)
//                set_motion(RIGHT);
//            else
//                set_motion(LEFT);
//        } else if (mydata->cycle_dispersion < mydata->tumble_time + mydata->run_time) {
//            // run state
//            set_motion(FORWARD);
//            pogobot_led_setColors(0,3,0, 4); // green
//        } else {
//            mydata->last_pogoticks_dispersion = pogoticks;
//            mydata->flag_dispersion = 0;
//        }
//
//    } else { // stop
//        if (mydata->cycle_dispersion < mydata->tumble_time + mydata->run_time) {
//            pogobot_motor_set(0, 0);
//            pogobot_led_setColors(3,3,3, 4); // white
//        } else {
//            mydata->last_pogoticks_dispersion = pogoticks;
//            mydata->flag_dispersion = 0;
//        }
//    }
//
////     set_color_from_nb_neighbours();
}



// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
