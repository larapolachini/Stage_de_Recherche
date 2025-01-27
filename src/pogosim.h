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

extern uint32_t pogobot_ticks;

extern uint8_t main_loop_hz;
extern uint8_t send_msg_hz;
extern uint8_t process_msg_hz;
extern void (*msg_rx_fn)(message_t *);
extern void (*msg_tx_fn)(void);
extern int8_t error_codes_led_idx;
extern time_reference_t _global_timer;
extern time_reference_t timer_main_loop;
extern uint32_t _current_time_milliseconds;

typedef enum {
    ERROR_TIME_OVERFLOW,
    error_code_t_last_entry
} error_code_t ;

void pogobot_start(void (*user_init)(void), void (*user_step)(void));
void pogo_main_loop_step(void (*user_step)(void));
uint32_t current_time_milliseconds(void);
void display_led_error_code(error_code_t const c);

#ifdef __cplusplus
}
#endif


#endif // POGOSIM_H

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
