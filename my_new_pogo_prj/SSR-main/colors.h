
#ifndef COLORS_H_
#define COLORS_H_

#include "ssr.h"

extern fp_t class_centroids[10];

typedef struct rgb8_t {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb8_t;

#define RGB(r_,g_,b_) {.r = r_, .g = g_, .b = b_}

inline void set_color(rgb8_t const* c, uint8_t nb_led) {
    pogobot_led_setColors(c->r, c->g, c->b, nb_led);
}

inline void set_colors(rgb8_t const* c, uint8_t nb_led) {
    pogobot_led_setColors(c->r, c->g, c->b, nb_led);
}

void set_color_from_lambda(fp_t lambda, uint8_t nb_led);
void set_color_from_s(fp_t x);
void set_color_from_signs(fp_t x);
void set_color_from_nb_neighbours(void);

#endif

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
