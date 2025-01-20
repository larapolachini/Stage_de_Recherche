
#include "pogosim.h"

#ifdef SIMULATOR // Compiling for the simulator
void pogo_start(void (*user_init)(void), void (*user_step)(void)) {
    current_robot->pogo_ticks = 0;
    pogobot_stopwatch_reset(&_global_timer);
    pogobot_stopwatch_reset(&timer_main_loop);
    current_robot->user_init = user_init;
    current_robot->user_step = user_step;
}

#endif

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
