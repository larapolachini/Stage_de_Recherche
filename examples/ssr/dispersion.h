
#ifndef DISPERSION_H_
#define DISPERSION_H_

#include "limmsswarm.h"

extern double const prob_moving;
extern uint64_t const base_tumble_time;
extern double const offset;
extern double const scaling;
extern double const d_optim;
extern uint64_t const lower_tumble_time;
extern uint64_t const upper_tumble_time;

void setup_dispersion(void);
void start_dispersion(void);
void behav_dispersion(void);

#endif

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
