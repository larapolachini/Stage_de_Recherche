
//#include "pogobot.h"
#include "pogosim.h"

typedef struct {
    uint8_t data_foo[8];
    time_reference_t timer_it;
} USERDATA;

//extern USERDATA *mydata;
REGISTER_USERDATA(USERDATA)



void user_init(void) {
    printf("setup ok\n");

    // Init timer
    pogobot_stopwatch_reset(&mydata->timer_it);

    // TODO Set main loop frequency, message sending frequency, message processing frequency (+ specify function???)
}


void user_step(void) {
    pogobot_led_setColor(0,0,255);
    pogobot_motor_set(motorL, motorFull);
    pogobot_motor_set(motorR, motorStop);
    msleep(50);

    printf(" HELLO WORLD !!!   Robot ID: %d   Current time: %lu   pogo_ticks: %d\n", pogobot_helper_getid(), pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it), pogo_ticks);

    pogobot_led_setColor(255,0,0);
    pogobot_motor_set(motorL, motorStop);
    pogobot_motor_set(motorR, motorFull);
    msleep(50);

    mydata->data_foo[0] = 42;
}


int main(void) {
    pogobot_init();
    printf("init ok\n");

    pogo_start(user_init, user_step);
    return 0;
}

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
