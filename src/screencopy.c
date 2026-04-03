#if OPENCV_ENABLED

#include "screencopy.h"

#include "ext-image-capture-source-v1-client-protocol.h"
#include "ext-image-copy-capture-v1-client-protocol.h"
#include "log.h"
#include "state.h"
#include "surface_buffer.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

enum capture_phase {
    CAP_INIT,
    CAP_CONSTRAINTS,
    CAP_REQUESTED,
    CAP_SUCCESS,
    CAP_FAILED,
};

struct capture_state {
    struct wl_shm *wl_shm;
    struct ext_image_copy_capture_session_v1 *session;
    struct ext_image_copy_capture_frame_v1 *frame;
    struct scrcpy_buffer *buffer;

    int32_t buf_width;
    int32_t buf_height;
    uint32_t shm_format;
    bool format_received;
    bool constraints_done;

    enum capture_phase phase;
};

static struct scrcpy_buffer *create_scrcpy_buffer(
    struct wl_shm *shm, enum wl_shm_format format, int32_t width,
    int32_t height, int32_t stride
) {
    size_t size = (size_t)stride * height;

    int fd = allocate_shm_file(size);
    if (fd == -1) {
        LOG_ERR("Could not allocate SHM file.");
        return NULL;
    }

    void *data = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (data == MAP_FAILED) {
        LOG_ERR("Could not mmap shared buffer for screencopy.");
        close(fd);
        return NULL;
    }

    struct wl_shm_pool *wl_shm_pool = wl_shm_create_pool(shm, fd, size);
    struct wl_buffer   *wl_buffer   = wl_shm_pool_create_buffer(
        wl_shm_pool, 0, width, height, stride, format
    );
    wl_shm_pool_destroy(wl_shm_pool);
    close(fd);

    struct scrcpy_buffer *buffer = malloc(sizeof(*buffer));
    buffer->wl_buffer = wl_buffer;
    buffer->format    = format;
    buffer->data      = data;
    buffer->width     = width;
    buffer->height    = height;
    buffer->stride    = stride;

    return buffer;
}

void destroy_scrcpy_buffer(struct scrcpy_buffer *buf) {
    if (buf != NULL) {
        munmap(buf->data, (size_t)buf->stride * buf->height);
        wl_buffer_destroy(buf->wl_buffer);
        free(buf);
    }
}

static void session_handle_buffer_size(
    void *data, struct ext_image_copy_capture_session_v1 *session,
    uint32_t width, uint32_t height
) {
    struct capture_state *cs = data;
    cs->buf_width            = width;
    cs->buf_height           = height;
}

static void session_handle_shm_format(
    void *data, struct ext_image_copy_capture_session_v1 *session,
    uint32_t format
) {
    struct capture_state *cs = data;
    if (!cs->format_received ||
        format == WL_SHM_FORMAT_ARGB8888 ||
        format == WL_SHM_FORMAT_XRGB8888) {
        cs->shm_format      = format;
        cs->format_received = true;
    }
}

static void session_handle_done(
    void *data, struct ext_image_copy_capture_session_v1 *session
) {
    struct capture_state *cs = data;
    cs->constraints_done     = true;
    cs->phase                = CAP_CONSTRAINTS;
}

static void noop() {}

static const struct ext_image_copy_capture_session_v1_listener
    session_listener = {
        .buffer_size  = session_handle_buffer_size,
        .shm_format   = session_handle_shm_format,
        .dmabuf_device = noop,
        .dmabuf_format = noop,
        .done         = session_handle_done,
        .stopped      = noop,
};

static void frame_handle_ready(
    void *data, struct ext_image_copy_capture_frame_v1 *frame
) {
    struct capture_state *cs = data;
    cs->phase                = CAP_SUCCESS;
}

static void frame_handle_failed(
    void *data, struct ext_image_copy_capture_frame_v1 *frame, uint32_t reason
) {
    struct capture_state *cs = data;
    cs->phase                = CAP_FAILED;
    LOG_ERR("Frame capture failed (reason: %u).", reason);
}

static const struct ext_image_copy_capture_frame_v1_listener frame_listener = {
    .transform         = noop,
    .damage            = noop,
    .presentation_time = noop,
    .ready             = frame_handle_ready,
    .failed            = frame_handle_failed,
};

static struct scrcpy_buffer *crop_buffer(
    struct scrcpy_buffer *full, struct wl_shm *shm,
    int32_t x, int32_t y, int32_t w, int32_t h
) {
    int32_t bpp    = 4;
    int32_t stride = w * bpp;

    struct scrcpy_buffer *cropped =
        create_scrcpy_buffer(shm, full->format, w, h, stride);
    if (cropped == NULL) {
        return NULL;
    }

    for (int32_t row = 0; row < h; row++) {
        void *dst = (char *)cropped->data + (size_t)row * stride;
        void *src = (char *)full->data +
                    (size_t)(y + row) * full->stride + (size_t)x * bpp;
        memcpy(dst, src, (size_t)w * bpp);
    }

    return cropped;
}

struct scrcpy_buffer *
query_screenshot(struct state *state, struct rect region) {
    if (state->ext_capture_manager == NULL) {
        LOG_ERR("Could not load `ext_image_copy_capture_manager_v1`.");
        exit(1);
    }

    if (state->ext_output_source_manager == NULL) {
        LOG_ERR(
            "Could not load `ext_output_image_capture_source_manager_v1`."
        );
        exit(1);
    }

    LOG_DEBUG(
        "Capture region: %dx%d+%d+%d", region.w, region.h, region.x, region.y
    );

    struct capture_state cs = {
        .wl_shm           = state->wl_shm,
        .session          = NULL,
        .frame            = NULL,
        .buffer           = NULL,
        .buf_width        = 0,
        .buf_height       = 0,
        .shm_format       = WL_SHM_FORMAT_ARGB8888,
        .format_received  = false,
        .constraints_done = false,
        .phase            = CAP_INIT,
    };

    struct ext_image_capture_source_v1 *source =
        ext_output_image_capture_source_manager_v1_create_source(
            state->ext_output_source_manager,
            state->current_output->wl_output
        );

    cs.session = ext_image_copy_capture_manager_v1_create_session(
        state->ext_capture_manager, source, 0
    );

    ext_image_capture_source_v1_destroy(source);

    ext_image_copy_capture_session_v1_add_listener(
        cs.session, &session_listener, &cs
    );

    while (!cs.constraints_done) {
        wl_display_roundtrip(state->wl_display);
    }

    if (cs.buf_width <= 0 || cs.buf_height <= 0) {
        LOG_ERR("Invalid buffer dimensions from session.");
        ext_image_copy_capture_session_v1_destroy(cs.session);
        return NULL;
    }

    LOG_DEBUG(
        "Capture buffer: %dx%d (format: 0x%08x)", cs.buf_width, cs.buf_height,
        cs.shm_format
    );

    int32_t stride = cs.buf_width * 4;
    cs.buffer = create_scrcpy_buffer(
        state->wl_shm, cs.shm_format, cs.buf_width, cs.buf_height, stride
    );
    if (cs.buffer == NULL) {
        ext_image_copy_capture_session_v1_destroy(cs.session);
        return NULL;
    }

    cs.frame = ext_image_copy_capture_session_v1_create_frame(cs.session);
    ext_image_copy_capture_frame_v1_attach_buffer(cs.frame, cs.buffer->wl_buffer);
    ext_image_copy_capture_frame_v1_damage_buffer(
        cs.frame, 0, 0, cs.buf_width, cs.buf_height
    );
    ext_image_copy_capture_frame_v1_capture(cs.frame);

    ext_image_copy_capture_frame_v1_add_listener(
        cs.frame, &frame_listener, &cs
    );

    cs.phase = CAP_REQUESTED;
    while (cs.phase == CAP_REQUESTED) {
        wl_display_roundtrip(state->wl_display);
    }

    ext_image_copy_capture_frame_v1_destroy(cs.frame);
    ext_image_copy_capture_session_v1_destroy(cs.session);

    if (cs.phase != CAP_SUCCESS) {
        destroy_scrcpy_buffer(cs.buffer);
        return NULL;
    }

    double scale_x =
        (double)cs.buf_width / state->current_output->width;
    double scale_y =
        (double)cs.buf_height / state->current_output->height;

    int32_t crop_x = (int32_t)(region.x * scale_x);
    int32_t crop_y = (int32_t)(region.y * scale_y);
    int32_t crop_w = (int32_t)(region.w * scale_x);
    int32_t crop_h = (int32_t)(region.h * scale_y);

    if (crop_x + crop_w > cs.buf_width) {
        crop_w = cs.buf_width - crop_x;
    }
    if (crop_y + crop_h > cs.buf_height) {
        crop_h = cs.buf_height - crop_y;
    }

    LOG_DEBUG(
        "Cropping: %dx%d+%d+%d (scale: %.2fx%.2f)", crop_w, crop_h, crop_x,
        crop_y, scale_x, scale_y
    );

    struct scrcpy_buffer *cropped =
        crop_buffer(cs.buffer, state->wl_shm, crop_x, crop_y, crop_w, crop_h);

    destroy_scrcpy_buffer(cs.buffer);

    return cropped;
}

#endif
