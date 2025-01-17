#ifndef SIMULATOR_H
#define SIMULATOR_H

//#include "spogobot.h"
#include "pogosim.h"
#include "configuration.h"

extern "C" int robot_main(void);

void create_robots(Configuration& config);
void main_loop(Configuration& config);
void set_current_robot(Robot& robot);

#endif // SIMULATOR_H

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
