#include "utils_wayland.h"

#include "eis.h"
#include "state.h"
#include "wlr-virtual-pointer-unstable-v1-client-protocol.h"

#include <unistd.h>
#include <wayland-client.h>

// Timings for the synthesised drag gesture (in microseconds). Many apps ignore
// a drag that happens instantly, so we settle on the press, hold the button,
// move in small steps and pause before releasing.
#define DRAG_SETTLE_US 60000  // after moving to the press point
#define DRAG_PRESS_US  120000 // after pressing the button
#define DRAG_STEP_US   12000  // between intermediate motion steps
#define DRAG_RELEASE_US 80000 // after reaching the target, before releasing
#define DRAG_STEPS     24     // number of intermediate motion steps

static void _apply_transform(
    uint32_t *x, uint32_t *y, uint32_t *width, uint32_t *height,
    enum wl_output_transform transform
) {
    uint32_t temp;

    switch (transform) {
    case WL_OUTPUT_TRANSFORM_NORMAL:
        break;

    case WL_OUTPUT_TRANSFORM_90:
        temp = *x;
        *x   = *y;
        *y   = *width - temp;

        temp    = *width;
        *width  = *height;
        *height = temp;
        break;

    case WL_OUTPUT_TRANSFORM_180:
        *x = *width - *x;
        *y = *height - *y;
        break;

    case WL_OUTPUT_TRANSFORM_270:
        temp = *x;
        *x   = *height - *y;
        *y   = temp;

        temp    = *width;
        *width  = *height;
        *height = temp;
        break;

    case WL_OUTPUT_TRANSFORM_FLIPPED:
        *x = *width - *x;
        break;

    case WL_OUTPUT_TRANSFORM_FLIPPED_90:
        *x = *width - *x;
        _apply_transform(x, y, width, height, WL_OUTPUT_TRANSFORM_90);
        break;

    case WL_OUTPUT_TRANSFORM_FLIPPED_180:
        *x = *width - *x;
        _apply_transform(x, y, width, height, WL_OUTPUT_TRANSFORM_180);
        break;

    case WL_OUTPUT_TRANSFORM_FLIPPED_270:
        *x = *width - *x;
        _apply_transform(x, y, width, height, WL_OUTPUT_TRANSFORM_270);
        break;
    }
}

struct pointer {
    struct state                   *state;
    struct zwlr_virtual_pointer_v1 *wlr;
};

// Picks the wlr virtual pointer when the compositor has one and KWin's EIS
// otherwise. Returns false when there is nothing to drive, which is the case in
// `--only-print` mode.
static bool pointer_begin(struct state *state, struct pointer *pointer) {
    pointer->state = state;
    pointer->wlr   = NULL;

    if (state->wl_virtual_pointer_mgr) {
        wl_display_roundtrip(state->wl_display);
        pointer->wlr =
            zwlr_virtual_pointer_manager_v1_create_virtual_pointer_with_output(
                state->wl_virtual_pointer_mgr,
                ((struct seat *)state->seats.next)->wl_seat,
                state->current_output->wl_output
            );
        return true;
    }

#if KWIN_ENABLED
    if (state->ei_device) {
        eis_begin(state);
        return true;
    }
#endif

    return false;
}

static void pointer_motion(struct pointer *pointer, uint32_t x, uint32_t y) {
    struct state *state = pointer->state;

#if KWIN_ENABLED
    if (pointer->wlr == NULL) {
        eis_motion(state, x, y);
        return;
    }
#endif

    uint32_t width  = state->current_output->width;
    uint32_t height = state->current_output->height;

    _apply_transform(&x, &y, &width, &height, state->current_output->transform);

    zwlr_virtual_pointer_v1_motion_absolute(pointer->wlr, 0, x, y, width, height);
    zwlr_virtual_pointer_v1_frame(pointer->wlr);
    wl_display_roundtrip(state->wl_display);
}

static void pointer_button(struct pointer *pointer, int button, bool pressed) {
#if KWIN_ENABLED
    if (pointer->wlr == NULL) {
        eis_button(pointer->state, button, pressed);
        return;
    }
#endif

    zwlr_virtual_pointer_v1_button(
        pointer->wlr, 0, button,
        pressed ? WL_POINTER_BUTTON_STATE_PRESSED
                : WL_POINTER_BUTTON_STATE_RELEASED
    );
    zwlr_virtual_pointer_v1_frame(pointer->wlr);
    wl_display_roundtrip(pointer->state->wl_display);
}

static void pointer_end(struct pointer *pointer) {
#if KWIN_ENABLED
    if (pointer->wlr == NULL) {
        eis_end(pointer->state);
        return;
    }
#endif

    zwlr_virtual_pointer_v1_destroy(pointer->wlr);
}

void move_pointer(
    struct state *state, uint32_t x, uint32_t y, enum click click
) {
    struct pointer pointer;
    if (!pointer_begin(state, &pointer)) {
        return;
    }

    pointer_motion(&pointer, x, y);

    if (state->click != CLICK_NONE) {
        int btn = 271 + click;
        pointer_button(&pointer, btn, true);
        pointer_button(&pointer, btn, false);
    }

    pointer_end(&pointer);
}

void drag_pointer(
    struct state *state, uint32_t from_x, uint32_t from_y, uint32_t to_x,
    uint32_t to_y
) {
    struct pointer pointer;
    if (!pointer_begin(state, &pointer)) {
        return;
    }

    const int btn = 271 + CLICK_LEFT_BTN;

    // Move to the press point and let it settle.
    pointer_motion(&pointer, from_x, from_y);
    usleep(DRAG_SETTLE_US);

    // Press and hold.
    pointer_button(&pointer, btn, true);
    usleep(DRAG_PRESS_US);

    // Move to the target in small steps so apps register a real drag.
    for (int i = 1; i <= DRAG_STEPS; i++) {
        uint32_t x = from_x + (int64_t)(to_x - from_x) * i / DRAG_STEPS;
        uint32_t y = from_y + (int64_t)(to_y - from_y) * i / DRAG_STEPS;
        pointer_motion(&pointer, x, y);
        usleep(DRAG_STEP_US);
    }

    // Pause on the target, then release.
    usleep(DRAG_RELEASE_US);
    pointer_button(&pointer, btn, false);

    pointer_end(&pointer);
}
