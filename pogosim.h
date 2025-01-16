#ifndef POGOSIM_H
#define POGOSIM_H

#ifdef REAL_ROBOT
#include "pogobot.h"
#else
#include "spogobot.h"
#endif


// Macros to declare the mydata pointer
#ifdef SIMULATOR // Compiling for the simulator
#include <stddef.h>
#define REGISTER_USERDATA(UDT) 		\
	size_t UserdataSize = sizeof(UDT); \
	UDT *mydata;

#else // Compiling for real robots
#define REGISTER_USERDATA(UDT) 		\
	UDT myuserdata;                 \
	UDT *mydata = &myuserdata; 

void user_init(void);
void user_step(void);

#endif // SIMULATOR


#ifdef __cplusplus
extern "C" {
#endif

extern uint64_t pogo_ticks;

// TODO
void pogo_start(void (*user_init)(void), void (*user_step)(void));

#ifdef __cplusplus
}
#endif


#endif // POGOSIM_H

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
