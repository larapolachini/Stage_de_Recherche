
#include <stddef.h>
#include "colormaps.h"

void qualitative_colormap(uint8_t const value, uint8_t *r, uint8_t *g, uint8_t *b) {
    // Define a fixed array of colors (qualitative colormap)
    uint8_t const colormap[][3] = {
        {255,   0,   0}, // Red
        {  0, 255,   0}, // Green
        {  0,   0, 255}, // Blue
        {255, 255,   0}, // Yellow
        {255,   0, 255}, // Magenta
        {  0, 255, 255}, // Cyan
        {128,   0, 128}, // Purple
        {128, 128,   0}, // Olive
        {  0, 128, 128}, // Teal
        {128, 128, 128}  // Gray
    };

    size_t const num_colors = sizeof(colormap) / sizeof(colormap[0]);

    // Map value to a color index (using modulo to wrap around)
    uint8_t const index = value % num_colors;

    // Assign R, G, B from the colormap
    *r = colormap[index][0];
    *g = colormap[index][1];
    *b = colormap[index][2];
}


void rainbow_colormap(uint8_t const value, uint8_t *r, uint8_t *g, uint8_t *b) {
    // Normalize value to a range [0, 6]
    float const normalized = (float)value / 255.0f * 6.0f;
    int const region = (int)normalized; // Integer part: determines the color segment
    float const fraction = normalized - region; // Fractional part: interpolation factor

    switch (region) {
        case 0: // Red to Yellow
            *r = 255;
            *g = (uint8_t)(255 * fraction);
            *b = 0;
            break;
        case 1: // Yellow to Green
            *r = (uint8_t)(255 * (1.0f - fraction));
            *g = 255;
            *b = 0;
            break;
        case 2: // Green to Cyan
            *r = 0;
            *g = 255;
            *b = (uint8_t)(255 * fraction);
            break;
        case 3: // Cyan to Blue
            *r = 0;
            *g = (uint8_t)(255 * (1.0f - fraction));
            *b = 255;
            break;
        case 4: // Blue to Magenta
            *r = (uint8_t)(255 * fraction);
            *g = 0;
            *b = 255;
            break;
        case 5: // Magenta to Red
            *r = 255;
            *g = 0;
            *b = (uint8_t)(255 * (1.0f - fraction));
            break;
        default: // Shouldn't reach here
            *r = 0;
            *g = 0;
            *b = 0;
            break;
    }
}

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
