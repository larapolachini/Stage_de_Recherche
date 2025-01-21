#ifndef RENDER_H
#define RENDER_H

#include <SDL2/SDL.h>

void SDL_RenderDrawCircle(SDL_Renderer* renderer, int x, int y, int radius);

float const VISUALIZATION_SCALE = 100.0f; // 1 Box2D unit = 100 pixels

#endif // RENDER_H

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
