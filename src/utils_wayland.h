#ifndef __UTILS_WAYLAND_H_INCLUDED__
#define __UTILS_WAYLAND_H_INCLUDED__

#include "state.h"

void move_pointer(
    struct state *state, uint32_t x, uint32_t y, enum click click
);

void drag_pointer(
    struct state *state, uint32_t from_x, uint32_t from_y, uint32_t to_x,
    uint32_t to_y
);

#endif
