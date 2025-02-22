
// Main include for pogobots, both for real robots and for simulations
#include "pogobase.h"

// "Global" variables should be inserted within the USERDATA struct.
// /!\  In simulation, don't declare non-const global variables outside this struct, elsewise they will be shared among all agents.
typedef struct {
    uint8_t data_foo[8];
    time_reference_t timer_it;
} USERDATA;

// Don't forget to call this macro in the main .c file of your project
REGISTER_USERDATA(USERDATA)


// Init function. Called once at the beginning of the program (cf 'pogobot_start' call in main())
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


// Step function. Called continuously at each step of the pogobot main loop
void user_step(void) {
    if (pogobot_ticks % 1000 == 0 && pogobot_helper_getid() == 0) {     // Only print messages for robot 0
        printf(" HELLO WORLD !!!   Robot ID: %d   Current time: %llums  Timer: %lluµs   pogobot_ticks: %lu\n",
                pogobot_helper_getid(),
                (long long unsigned int) current_time_milliseconds(),
                (long long unsigned int) pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it),
                (long unsigned int) pogobot_ticks       // Increased by one at each execution of user_step
                );
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


// Entrypoint of the program
int main(void) {
    pogobot_init();     // Initialization routine for the robots
#ifndef SIMULATOR
    printf("init ok\n");
#endif

    // Specify the user_init and user_step functions
    pogobot_start(user_init, user_step);
    return 0;
}

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
