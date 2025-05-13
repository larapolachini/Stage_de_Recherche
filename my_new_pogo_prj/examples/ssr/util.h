
#ifndef UTIL_H_
#define UTIL_H_

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399375105820974944
#endif
#include "limmsswarm.h"
#include "pogobase.h"

//#define kiloticks_to_µs (1e6 / 31)
#define kiloticks_to_µs (1e6 / 62)
//#define kiloticks_to_µs (1e6 / 1)
#define _float_to_2_int(val) (int)val, (int)((ABS(val) - (int)ABS(val))*1000)


#define _EXP2(a, b) a ## b
#define _EXP(a, b) _EXP2(a ## e,b)
#define _CONSTPOW(C, x) ((int)_EXP(C, x))
#define _print_float_2(N, P) printf("%d.%d", (int)N, (int)((N - (int)N)*(_CONSTPOW(10, P)/10)))
#define _print_float_1(N) _print_float_2(N, 3)    // Default precision : 3 significative figures
#define _print_float_X(x,A,B,FUNC,...) FUNC
#define print_float(...) _print_float_X(,##__VA_ARGS__,_print_float_2(__VA_ARGS__),_print_float_1(__VA_ARGS__))


// declare motion variable type
typedef enum {
    STOP,
    FORWARD,
    LEFT,
    RIGHT
} motion_t;

extern uint16_t pogoid;
extern uint64_t pogoticks;

void set_motion(motion_t new_motion);
void init_ticks(void);
void update_ticks(void);
void init_pogoid(void);
void init_rand(void);

#endif

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
