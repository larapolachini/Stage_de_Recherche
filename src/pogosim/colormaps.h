#ifndef COLORMAPS_H
#define COLORMAPS_H

#include <stdint.h>

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} color_t;

void qualitative_colormap(uint8_t const value, uint8_t *r, uint8_t *g, uint8_t *b);
void rainbow_colormap(uint8_t const value, uint8_t *r, uint8_t *g, uint8_t *b);

#endif // COLORMAPS_H

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
