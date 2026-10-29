#ifndef __EIS_H_INCLUDED__
#define __EIS_H_INCLUDED__

#if KWIN_ENABLED

#include <stdbool.h>
#include <stdint.h>

struct state;

bool eis_connect(struct state *state);
void eis_disconnect(struct state *state);

void eis_begin(struct state *state);
void eis_motion(struct state *state, uint32_t x, uint32_t y);
void eis_button(struct state *state, uint32_t button, bool pressed);
void eis_end(struct state *state);

#endif

#endif
