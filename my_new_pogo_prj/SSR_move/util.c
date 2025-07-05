/* TODO
 */

#include "util.h"
#include "pogobase.h"

time_reference_t timer_it;
uint32_t pogoticks;


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
            pogobot_motor_set(motorL, motorQuarter);
            pogobot_motor_set(motorR, motorQuarter);
            break;
        case LEFT:
            pogobot_motor_set(motorL, motorQuarter);
            pogobot_motor_set(motorR, motorQuarter);
            break;
        case RIGHT:
            pogobot_motor_set(motorL, motorQuarter);
            pogobot_motor_set(motorR, motorQuarter);
            break;
    }
#endif
}


void init_rand(void) {
    srand(pogobot_helper_getRandSeed());
}


// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
