#if KWIN_ENABLED

#include "eis.h"

#include "log.h"
#include "state.h"

#include <fcntl.h>
#include <libei.h>
#include <poll.h>
#include <string.h>
#include <systemd/sd-bus.h>
#include <unistd.h>

// `connectToEIS` takes the xdg-desktop-portal device types, 2 is the pointer.
#define EIS_PORTAL_POINTER 2
#define EIS_TIMEOUT_MS     2000

static bool handle_event(
    struct state *state, struct ei_event *event, struct ei_ping *ping
) {
    switch (ei_event_get_type(event)) {
    case EI_EVENT_SEAT_ADDED:
        ei_seat_bind_capabilities(
            ei_event_get_seat(event), EI_DEVICE_CAP_POINTER_ABSOLUTE,
            EI_DEVICE_CAP_BUTTON, NULL
        );
        return false;

    case EI_EVENT_DEVICE_ADDED: {
        struct ei_device *device = ei_event_get_device(event);
        if (state->ei_device == NULL &&
            ei_device_has_capability(device, EI_DEVICE_CAP_POINTER_ABSOLUTE) &&
            ei_device_has_capability(device, EI_DEVICE_CAP_BUTTON)) {
            state->ei_device = ei_device_ref(device);
        }
        return false;
    }

    case EI_EVENT_DEVICE_RESUMED:
        return ping == NULL && ei_event_get_device(event) == state->ei_device;

    case EI_EVENT_PONG:
        return ping != NULL && ei_event_pong_get_ping(event) == ping;

    default:
        return false;
    }
}

// Processes server events until the pointer device is resumed or, with a
// `ping`, until the server answered it, which means everything sent before was
// handled.
static bool pump(struct state *state, struct ei_ping *ping) {
    struct pollfd pollfd = {.fd = ei_get_fd(state->ei), .events = POLLIN};

    while (true) {
        if (poll(&pollfd, 1, EIS_TIMEOUT_MS) <= 0) {
            LOG_ERR("Timed out waiting for the EIS server.");
            return false;
        }

        ei_dispatch(state->ei);

        bool             done         = false;
        bool             disconnected = false;
        struct ei_event *event;
        while ((event = ei_get_event(state->ei)) != NULL) {
            if (ei_event_get_type(event) == EI_EVENT_DISCONNECT) {
                disconnected = true;
            } else if (handle_event(state, event, ping)) {
                done = true;
            }
            ei_event_unref(event);
        }

        if (disconnected) {
            LOG_ERR("The EIS server disconnected.");
            return false;
        }
        if (done) {
            return true;
        }
    }
}

static int request_eis_fd(struct state *state) {
    sd_bus_error    error = SD_BUS_ERROR_NULL;
    sd_bus_message *reply = NULL;
    int             fd    = -1;

    int r = sd_bus_call_method(
        state->sd_bus, "org.kde.KWin", "/org/kde/KWin/EIS/RemoteDesktop",
        "org.kde.KWin.EIS.RemoteDesktop", "connectToEIS", &error, &reply, "i",
        EIS_PORTAL_POINTER
    );
    if (r < 0) {
        LOG_ERR(
            "Failed to connect to KWin's EIS: %s.",
            error.message ? error.message : strerror(-r)
        );
        sd_bus_error_free(&error);
        return -1;
    }

    int server_fd;
    int cookie;
    r = sd_bus_message_read(reply, "hi", &server_fd, &cookie);
    if (r < 0) {
        LOG_ERR("Unexpected reply from KWin's EIS: %s.", strerror(-r));
    } else {
        fd = fcntl(server_fd, F_DUPFD_CLOEXEC, 0);
    }

    sd_bus_message_unref(reply);
    return fd;
}

bool eis_connect(struct state *state) {
    int r = sd_bus_open_user(&state->sd_bus);
    if (r < 0) {
        LOG_ERR("Failed to connect to the session bus: %s.", strerror(-r));
        return false;
    }

    int fd = request_eis_fd(state);
    if (fd < 0) {
        return false;
    }

    state->ei = ei_new_sender(NULL);
    ei_configure_name(state->ei, "wl-kbptr");

    r = ei_setup_backend_fd(state->ei, fd);
    if (r < 0) {
        LOG_ERR("Failed to set up the EIS connection: %s.", strerror(-r));
        return false;
    }

    return pump(state, NULL);
}

void eis_disconnect(struct state *state) {
    if (state->ei_device != NULL) {
        ei_device_unref(state->ei_device);
        state->ei_device = NULL;
    }
    if (state->ei != NULL) {
        state->ei = ei_unref(state->ei);
    }
    if (state->sd_bus != NULL) {
        state->sd_bus = sd_bus_flush_close_unref(state->sd_bus);
    }
}

void eis_begin(struct state *state) {
    ei_device_start_emulating(state->ei_device, ++state->ei_sequence);
}

void eis_motion(struct state *state, uint32_t x, uint32_t y) {
    ei_device_pointer_motion_absolute(
        state->ei_device, state->current_output->x + x,
        state->current_output->y + y
    );
    ei_device_frame(state->ei_device, ei_now(state->ei));
}

void eis_button(struct state *state, uint32_t button, bool pressed) {
    ei_device_button_button(state->ei_device, button, pressed);
    ei_device_frame(state->ei_device, ei_now(state->ei));
}

void eis_end(struct state *state) {
    ei_device_stop_emulating(state->ei_device);

    struct ei_ping *ping = ei_new_ping(state->ei);
    ei_ping(ping);
    pump(state, ping);
    ei_ping_unref(ping);
}

#endif
