
#include "pogosim.h"
#include "robot.h"

#ifdef SIMULATOR // Compiling for the simulator
//void _pogobot_start(void (*user_init)(void), void (*user_step)(void)) {
void _pogobot_start(void (*user_init)(void), void (*user_step)(void), const char *object_category) {
    if (std::string(object_category) != current_robot->category)
        return;
    current_robot->pogobot_ticks = 0;
    pogobot_stopwatch_reset(&_global_timer);
    pogobot_stopwatch_reset(&timer_main_loop);
    current_robot->user_init = user_init;
    current_robot->user_step = user_step;
}

#endif

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
