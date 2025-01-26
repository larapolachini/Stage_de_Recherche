/* TODO
 */

#include "util.h"
#include "pogosim.h"

time_reference_t timer_it;
uint16_t pogoid;
uint64_t pogoticks;


void set_motion(motion_t new_motion) {
#ifdef DISABLE_MOTION
    pogobot_motor_set(motorL, motorStop);
    pogobot_motor_set(motorR, motorStop);
    return;

#else
    switch(new_motion) {
        case STOP:
            pogobot_motor_set(motorL, motorStop);
            pogobot_motor_set(motorR, motorStop);
            break;
        case FORWARD:
            pogobot_motor_set(motorL, motorFull);
            pogobot_motor_set(motorR, motorFull);
            break;
        case LEFT:
            pogobot_motor_set(motorL, motorFull);
            pogobot_motor_set(motorR, motorStop);
            break;
        case RIGHT:
            pogobot_motor_set(motorL, motorStop);
            pogobot_motor_set(motorR, motorFull);
            break;
    }
#endif
}


void update_ticks(void) {
    pogoticks += pogobot_stopwatch_get_elapsed_microseconds(&timer_it);
    pogobot_stopwatch_reset(&timer_it);
}

void init_ticks(void) {
    pogobot_stopwatch_reset(&timer_it);
    pogoticks = 0;
}

void init_pogoid(void) {
    pogoid = pogobot_helper_getid();
}

void init_rand(void) {
    srand(pogobot_helper_getRandSeed());
}


// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
