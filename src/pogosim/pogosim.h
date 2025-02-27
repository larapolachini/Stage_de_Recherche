#ifndef POGOSIM_H
#define POGOSIM_H

#include <stdbool.h>

#ifdef REAL_ROBOT
#include "pogobot.h"
#else
#include "spogobot.h"
#endif


// Macros to declare the mydata pointer
#ifdef SIMULATOR // Compiling for the simulator
#include <stddef.h>

#define DECLARE_USERDATA(UDT)       \
    extern UDT *mydata;

#define REGISTER_USERDATA(UDT) 		\
	size_t UserdataSize = sizeof(UDT); \
	UDT *mydata;

#define SET_CALLBACK(CALLBACK_FN, FN) \
    CALLBACK_FN = FN;

extern void (*callback_create_data_schema)(void);
extern void (*callback_export_data)(void);
extern void (*callback_global_setup)(void);

#else // Compiling for real robots

// On real robots, declare an extern variable for a single shared instance.
#define DECLARE_USERDATA(UDT)       \
    extern UDT myuserdata;         \
    static inline UDT * restrict get_mydata(void) { return &myuserdata; }  // The accessor returns a restrict-qualified pointer.
    // Here we use the keyword "restrict" to ensure that the compiler automatically transform mydata->foo statements into myuserdata.foo after optimization.
    //  This increases performance by removing pointer access operations.

/* Now, use mydata as an alias for the inline function result */
#define mydata (get_mydata())

#define REGISTER_USERDATA(UDT) 		\
	UDT myuserdata;

#define SET_CALLBACK(CALLBACK_FN, FN)

void user_init(void);
void user_step(void);

#endif // SIMULATOR


#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t pogobot_ticks;

extern uint8_t main_loop_hz;
extern uint8_t max_nb_processed_msg_per_tick;
extern void (*msg_rx_fn)(message_t *);
extern bool (*msg_tx_fn)(void);
extern int8_t error_codes_led_idx;
extern time_reference_t _global_timer;
extern time_reference_t timer_main_loop;
extern uint32_t _current_time_milliseconds;
extern uint32_t _error_code_initial_time;

extern uint8_t percent_msgs_sent_per_ticks;
extern uint32_t nb_msgs_sent;
extern uint32_t nb_msgs_recv;

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
