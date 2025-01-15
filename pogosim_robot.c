
#include "pogosim.h"

#ifndef SIMULATOR // Compiling for real robots
void pogo_start(void (*user_init)(void), void (*user_step)(void)) {
    // Launch user-specified init function
    user_init();

    // Main loop
    for (;;) {
        // Update clock_ms
        // TODO

        // Purge old messages if needed
        // TODO

        // Check if we received a system message (e.g. rc_stop)
        // TODO

        // Messages I/O (send & receive)
        // TODO

        // Call user-specified step function
        user_step();

        // Detect and handle time overflows
        // TODO

        // Sleep to align temporally to the next main loop step
        // TODO

        // Update pogo_ticks
        // TODO

    }
}

#endif

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
