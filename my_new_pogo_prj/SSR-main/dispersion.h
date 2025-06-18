
#ifndef DISPERSION_H_
#define DISPERSION_H_

#include "ssr.h"

extern double const prob_moving;
extern uint32_t const base_tumble_time;
extern double const offset;
extern double const scaling;
extern double const d_optim;
extern uint32_t const lower_tumble_time;
extern uint32_t const upper_tumble_time;

void setup_dispersion(void);
void start_dispersion(void);
void behav_dispersion(void);

#endif

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
