
#include "pogobase.h"

typedef struct {
    uint8_t data_foo[8];
    time_reference_t timer_it;
} USERDATA;

//extern USERDATA *mydata;
REGISTER_USERDATA(USERDATA)



void user_init(void) {
#ifndef SIMULATOR
    printf("setup ok\n");
#endif

    // Init timer
    pogobot_stopwatch_reset(&mydata->timer_it);

    // Set main loop frequency, message sending frequency, message processing frequency
    main_loop_hz = 60;
    max_nb_processed_msg_per_tick = 3;
    // Specify functions to send/transmit messages
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;

    // Set led index to show error codes
    error_codes_led_idx = 3; // Default value, negative values to disable
}


void user_step(void) {
    if (pogobot_ticks % 1000 == 0 && pogobot_helper_getid() == 0) {
        //printf(" HELLO WORLD !!!   Robot ID: %d   Current time: %lu   pogobot_ticks: %d\n", pogobot_helper_getid(), pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it), pogobot_ticks);
        printf(" HELLO WORLD !!!   Robot ID: %d   Current time: %llu   pogobot_ticks: %lu\n",
                pogobot_helper_getid(), (long long unsigned int) current_time_milliseconds(), (long unsigned int) pogobot_ticks);
    }

    if ((uint32_t)(current_time_milliseconds() / 10000) % 2 == 0) {
        pogobot_led_setColor(0,0,255);
        pogobot_motor_set(motorL, motorFull);
        pogobot_motor_set(motorR, motorStop);
    } else {
        pogobot_led_setColor(255,0,0);
        pogobot_motor_set(motorL, motorStop);
        pogobot_motor_set(motorR, motorFull);
    }

    mydata->data_foo[0] = 42;
}


int main(void) {
    pogobot_init();
#ifndef SIMULATOR
    printf("init ok\n");
#endif

    pogobot_start(user_init, user_step);
    return 0;
}

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
