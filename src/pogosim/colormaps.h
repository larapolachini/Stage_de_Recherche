#ifndef COLORMAPS_H
#define COLORMAPS_H

#include <stdint.h>

/**
 * @brief Structure representing a color with red, green, and blue components.
 */
typedef struct {
    uint8_t r; /**< Red component (0-255) */
    uint8_t g; /**< Green component (0-255) */
    uint8_t b; /**< Blue component (0-255) */
} color_t;


/**
 * @brief Maps a given value to a qualitative colormap.
 *
 * This function assigns a fixed color from a predefined qualitative colormap based on the
 * provided value. The colormap is defined as an array of 10 distinct colors. The input value is
 * mapped to a color index using modulo arithmetic, ensuring that it wraps around if the value
 * exceeds the number of available colors.
 *
 * @param value The input value used to select a color from the colormap.
 * @param r Pointer to a uint8_t variable where the red component of the selected color will be stored.
 * @param g Pointer to a uint8_t variable where the green component of the selected color will be stored.
 * @param b Pointer to a uint8_t variable where the blue component of the selected color will be stored.
 */
void qualitative_colormap(uint8_t const value, uint8_t *r, uint8_t *g, uint8_t *b);

/**
 * @brief Maps a given value to a rainbow colormap with smooth interpolation.
 *
 * This function computes a rainbow color based on the input value. The value, expected in the range
 * [0, 255], is normalized to a range [0, 6] to determine the color segment and the interpolation factor.
 * Depending on the segment, the function interpolates between two adjacent colors in the rainbow spectrum.
 *
 * The segments are as follows:
 * - Region 0: Red to Yellow
 * - Region 1: Yellow to Green
 * - Region 2: Green to Cyan
 * - Region 3: Cyan to Blue
 * - Region 4: Blue to Magenta
 * - Region 5: Magenta to Red
 *
 * @param value The input value (0-255) that determines the color.
 * @param r Pointer to a uint8_t variable where the red component of the computed color will be stored.
 * @param g Pointer to a uint8_t variable where the green component of the computed color will be stored.
 * @param b Pointer to a uint8_t variable where the blue component of the computed color will be stored.
 */
void rainbow_colormap(uint8_t const value, uint8_t *r, uint8_t *g, uint8_t *b);

#endif // COLORMAPS_H

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
