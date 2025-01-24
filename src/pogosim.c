
#include "pogosim.h"
#include "colormaps.h"

uint8_t main_loop_hz = 60;
uint8_t send_msg_hz = 30;
uint8_t process_msg_hz = 60;
void (*msg_rx_fn)(message_t *) = NULL;
void (*msg_tx_fn)(void) = NULL;
int8_t error_codes_led_idx = 3; // Default value, negative values to disable
uint32_t pogobot_ticks = 0;
uint64_t _current_time_milliseconds = 0LL;
time_reference_t _global_timer;
time_reference_t timer_main_loop;


#ifndef SIMULATOR // Compiling for real robots
void pogobot_start(void (*user_init)(void), void (*user_step)(void)) {
    pogobot_ticks = 0;
    pogobot_stopwatch_reset(&_global_timer);
    pogobot_stopwatch_reset(&timer_main_loop);

    // Launch user-specified init function
    user_init();
    // Main loop
    for (;;) {
        pogo_main_loop_step(user_step);
    }
}
#endif

uint64_t current_time_milliseconds(void) {
    _current_time_milliseconds += pogobot_stopwatch_get_elapsed_microseconds(&_global_timer) / 1000;
    pogobot_stopwatch_reset(&_global_timer);
    return _current_time_milliseconds;
}

void display_led_error_code(error_code_t const c) {
    // Check if led error codes are enabled
    if (error_codes_led_idx < 0)
        return;

    // Find color from error code using the qualitative color map
    uint8_t r, g, b;
    qualitative_colormap(c, &r, &g, &b);
    pogobot_led_setColors(r, g, b, error_codes_led_idx);
}

void pogo_main_loop_step(void (*user_step)(void)) {
    // Update millisecond clock
    pogobot_stopwatch_reset(&timer_main_loop);

    // Purge old messages if needed
    // TODO

    // Check if we received a system message (e.g. rc_stop)
    // TODO

    // Messages I/O (send & receive)
    // TODO frequency !!!
    if (msg_tx_fn)
        msg_tx_fn();
//    if (msg_rx_fn)
//        msg_rx_fn(); // XXX TODO

    // Call user-specified step function
    user_step();

    if (main_loop_hz > 0) {
        // Detect and handle time overflows
        uint64_t const elapsed_µs = pogobot_stopwatch_get_elapsed_microseconds(&timer_main_loop);
        uint64_t const step_max_duration = 1000000/main_loop_hz;
        if (elapsed_µs > step_max_duration) {
            printf("[TIME] Error code: %u: TIME OVERFLOW ERROR! Step took %llu µs, should be less than %llu µs.", ERROR_TIME_OVERFLOW, elapsed_µs, step_max_duration);
            display_led_error_code(ERROR_TIME_OVERFLOW);
        }

        // Sleep to align temporally to the next main loop step
        if (elapsed_µs < step_max_duration) {
            uint32_t const ms_to_sleep = (step_max_duration - elapsed_µs) / 1000;
            msleep(ms_to_sleep); // Wait for next step
        }
    }

    // Update pogobot_ticks
    pogobot_ticks++;
}

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
