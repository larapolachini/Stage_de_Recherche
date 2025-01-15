

#include "pogosim.h"

void pogo_start(void (*setup)(void), void (*loop)(void)) {
    // TODO FROM KILOMBO !!!
    current_robot->user_setup = setup;
    current_robot->user_loop = loop;
}

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
