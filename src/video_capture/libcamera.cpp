/**
 * @file   video_capture/libcamera.cpp
 */
/*
 * Copyright (c) 2026 CESNET, zajmove sdruzeni pravnickych osob
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, is permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of CESNET nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESSED OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "config.h"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cinttypes>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string_view>
#include <vector>

#include <sys/mman.h>
#include <unistd.h>

#include <libcamera/control_ids.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/libcamera.h>
#include <libcamera/formats.h>

#include "debug.h"
#include "lib_common.h"
#include "utils/string_view_utils.hpp"
#include "video.h"
#include "video_capture.h"
#include "video_capture_params.h"
#include "video_codec.h"
#include "video_frame.h"

#define MOD_NAME "[libcamera] "

namespace {

const char *validation_status_to_string(libcamera::CameraConfiguration::Status status)
{
        switch (status) {
        case libcamera::CameraConfiguration::Valid:
                return "valid";
        case libcamera::CameraConfiguration::Adjusted:
                return "adjusted";
        case libcamera::CameraConfiguration::Invalid:
                return "invalid";
        }
        return "unknown";
}

struct libcamera_options {
        enum class Action {
                Run,
                Help,
                Fullhelp,
                List,
                Caps,
        } action = Action::Run;
        size_t camera_index = 0;
        libcamera::Size size = {};
        unsigned int fps = 0;
        bool size_set = false;
        bool fps_set = false;
        bool format_yuv420 = false;
};

void log_stream_config(const char *label,
                const libcamera::StreamConfiguration &stream_config)
{
        log_msg(LOG_LEVEL_INFO,
                        MOD_NAME "%s: pixelFormat=%s size=%s stride=%u "
                        "frameSize=%u bufferCount=%u\n",
                        label, stream_config.pixelFormat.toString().c_str(),
                        stream_config.size.toString().c_str(),
                        stream_config.stride, stream_config.frameSize,
                        stream_config.bufferCount);
}

void log_frame_buffer(const libcamera::FrameBuffer &buffer)
{
        const libcamera::FrameMetadata &metadata = buffer.metadata();

        log_msg(LOG_LEVEL_INFO,
                        MOD_NAME "completed buffer metadata: sequence=%u "
                        "timestamp=%" PRIu64 " status=%u\n",
                        metadata.sequence, metadata.timestamp,
                        static_cast<unsigned int>(metadata.status));

        auto planes = buffer.planes();
        auto metadata_planes = metadata.planes();
        log_msg(LOG_LEVEL_INFO, MOD_NAME "buffer planes: %zu\n",
                        planes.size());
        for (size_t i = 0; i < planes.size(); ++i) {
                const libcamera::FrameBuffer::Plane &plane = planes[i];
                const char *bytesused = "n/a";
                char bytesused_buf[32] = {};
                if (i < metadata_planes.size()) {
                        snprintf(bytesused_buf, sizeof bytesused_buf, "%u",
                                        metadata_planes[i].bytesused);
                        bytesused = bytesused_buf;
                }
                log_msg(LOG_LEVEL_INFO,
                                MOD_NAME "buffer plane %zu: fd=%d offset=%u "
                                "length=%u bytesused=%s\n",
                                i, plane.fd.get(), plane.offset, plane.length,
                                bytesused);
        }
}

void log_frame_buffer_layout(const libcamera::FrameBuffer &buffer)
{
        auto planes = buffer.planes();
        log_msg(LOG_LEVEL_INFO, MOD_NAME "buffer planes: %zu\n", planes.size());
        for (size_t i = 0; i < planes.size(); ++i) {
                const libcamera::FrameBuffer::Plane &plane = planes[i];
                log_msg(LOG_LEVEL_INFO,
                                MOD_NAME "buffer plane %zu: fd=%d offset=%u "
                                "length=%u\n",
                                i, plane.fd.get(), plane.offset, plane.length);
        }
}

struct mapped_plane {
        void *base = MAP_FAILED;
        size_t map_len = 0;
        const unsigned char *data = nullptr;
        size_t len = 0;
};

struct mapped_buffer {
        std::vector<mapped_plane> planes;
};

struct vidcap_libcamera_state {
        std::unique_ptr<libcamera::CameraManager> camera_manager;
        std::shared_ptr<libcamera::Camera> camera;
        std::unique_ptr<libcamera::FrameBufferAllocator> allocator;
        std::vector<std::unique_ptr<libcamera::Request>> requests;
        std::map<libcamera::FrameBuffer *, mapped_buffer> mapped_buffers;
        libcamera::Stream *stream = nullptr;
        libcamera::StreamConfiguration stream_config = {};
        struct video_desc desc = {};
        struct video_frame *frame = nullptr;
        std::mutex lock;
        std::condition_variable completed_cv;
        std::deque<libcamera::Request *> completed_requests;
        bool callback_connected = false;
        bool camera_started = false;
        bool camera_acquired = false;
        bool stopping = false;
        unsigned int copied_frames = 0;

        void request_completed(libcamera::Request *request)
        {
                std::lock_guard<std::mutex> guard(lock);
                if (!stopping &&
                                request->status() ==
                                        libcamera::Request::RequestComplete) {
                        completed_requests.push_back(request);
                }
                completed_cv.notify_all();
        }
};

void unmap_buffers(vidcap_libcamera_state *s)
{
        for (auto &[buffer, mapped] : s->mapped_buffers) {
                (void) buffer;
                for (mapped_plane &plane : mapped.planes) {
                        if (plane.base != MAP_FAILED) {
                                munmap(plane.base, plane.map_len);
                                plane.base = MAP_FAILED;
                        }
                }
        }
        s->mapped_buffers.clear();
}

void vidcap_libcamera_cleanup(vidcap_libcamera_state *s)
{
        if (s == nullptr) {
                return;
        }

        {
                std::lock_guard<std::mutex> guard(s->lock);
                s->stopping = true;
                s->completed_requests.clear();
        }
        s->completed_cv.notify_all();

        if (s->camera && s->camera_started) {
                int ret = s->camera->stop();
                if (ret != 0) {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "camera stop failed: %d\n",
                                        ret);
                }
                s->camera_started = false;
        }
        if (s->camera && s->callback_connected) {
                s->camera->requestCompleted.disconnect(s);
                s->callback_connected = false;
        }

        s->requests.clear();
        unmap_buffers(s);
        s->allocator.reset();

        if (s->camera && s->camera_acquired) {
                s->camera->release();
                s->camera_acquired = false;
        }
        s->camera.reset();

        if (s->camera_manager) {
                s->camera_manager->stop();
                s->camera_manager.reset();
        }

        vf_free(s->frame);
        s->frame = nullptr;
        delete s;
}

bool map_frame_buffer(vidcap_libcamera_state *s,
                libcamera::FrameBuffer *buffer)
{
        const long page_size = sysconf(_SC_PAGESIZE);
        if (page_size <= 0) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "failed to get system page size\n");
                return false;
        }

        mapped_buffer mapped;
        for (const libcamera::FrameBuffer::Plane &plane : buffer->planes()) {
                const off_t page_mask = static_cast<off_t>(page_size - 1);
                const off_t map_offset = plane.offset & ~page_mask;
                const size_t delta = plane.offset - map_offset;
                const size_t map_len = delta + plane.length;
                void *base = mmap(nullptr, map_len, PROT_READ, MAP_SHARED,
                                plane.fd.get(), map_offset);
                if (base == MAP_FAILED) {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "mmap failed: %s\n",
                                        strerror(errno));
                        return false;
                }

                mapped.planes.push_back({
                                base,
                                map_len,
                                static_cast<const unsigned char *>(base) + delta,
                                plane.length,
                });
        }

        s->mapped_buffers.emplace(buffer, std::move(mapped));
        return true;
}

bool setup_mmaps_and_requests(vidcap_libcamera_state *s)
{
        int alloc_ret = s->allocator->allocate(s->stream);
        if (alloc_ret < 0) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "failed to allocate frame buffers: %d\n",
                                alloc_ret);
                return false;
        }

        const auto &buffers = s->allocator->buffers(s->stream);
        log_msg(LOG_LEVEL_INFO, MOD_NAME "allocated %zu frame buffers\n",
                        buffers.size());
        if (buffers.empty()) {
                log_msg(LOG_LEVEL_ERROR, MOD_NAME "no frame buffers allocated\n");
                return false;
        }

        s->requests.reserve(buffers.size());
        for (const std::unique_ptr<libcamera::FrameBuffer> &buffer : buffers) {
                log_frame_buffer_layout(*buffer);
                if (!map_frame_buffer(s, buffer.get())) {
                        return false;
                }

                std::unique_ptr<libcamera::Request> request =
                        s->camera->createRequest();
                if (!request) {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "failed to create request\n");
                        return false;
                }
                int add_ret = request->addBuffer(s->stream, buffer.get());
                if (add_ret != 0) {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "failed to add buffer to "
                                        "request: %d\n",
                                        add_ret);
                        return false;
                }
                s->requests.push_back(std::move(request));
        }

        return true;
}

bool start_camera(vidcap_libcamera_state *s, const libcamera_options &opts)
{
        libcamera::ControlList controls(s->camera->controls());
        libcamera::ControlList *start_controls = nullptr;
        if (opts.fps_set) {
                const int64_t frame_duration_us =
                        1000000 / static_cast<int64_t>(opts.fps);
                if (s->camera->controls().count(
                                libcamera::controls::FrameDurationLimits.id()) >
                                0) {
                        controls.set(libcamera::controls::FrameDurationLimits,
                                        { frame_duration_us,
                                                frame_duration_us });
                        start_controls = &controls;
                        log_msg(LOG_LEVEL_INFO,
                                        MOD_NAME "applying fps=%u via "
                                        "FrameDurationLimits=%" PRId64 " us\n",
                                        opts.fps, frame_duration_us);
                } else {
                        log_msg(LOG_LEVEL_INFO,
                                        MOD_NAME "requested fps=%u; "
                                        "FrameDurationLimits control is not "
                                        "available\n",
                                        opts.fps);
                }
        }

        s->camera->requestCompleted.connect(s,
                        &vidcap_libcamera_state::request_completed);
        s->callback_connected = true;

        int start_ret = s->camera->start(start_controls);
        if (start_ret != 0) {
                log_msg(LOG_LEVEL_ERROR, MOD_NAME "camera start failed: %d\n",
                                start_ret);
                return false;
        }
        s->camera_started = true;

        for (const std::unique_ptr<libcamera::Request> &request : s->requests) {
                int queue_ret = s->camera->queueRequest(request.get());
                if (queue_ret != 0) {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "failed to queue request: %d\n",
                                        queue_ret);
                        return false;
                }
        }

        return true;
}

void vidcap_libcamera_probe(struct device_info **available_cards, int *count,
                void (**deleter)(void *))
{
        *deleter = free;
        *available_cards = nullptr;
        *count = 0;

        libcamera::CameraManager camera_manager;
        if (camera_manager.start() != 0) {
                log_msg(LOG_LEVEL_WARNING, MOD_NAME "failed to start CameraManager\n");
                return;
        }

        {
                auto cameras = camera_manager.cameras();
                if (!cameras.empty()) {
                        auto *devices = static_cast<device_info *>(
                                        calloc(cameras.size(),
                                                sizeof(device_info)));
                        if (devices != nullptr) {
                                for (size_t i = 0; i < cameras.size(); ++i) {
                                        snprintf(devices[i].dev,
                                                        sizeof devices[i].dev,
                                                        ":camera=%zu", i);
                                        snprintf(devices[i].name,
                                                        sizeof devices[i].name,
                                                        "%s",
                                                        cameras[i]->id().c_str());
                                }

                                *available_cards = devices;
                                *count = static_cast<int>(cameras.size());
                        }
                }
        }

        camera_manager.stop();
}

void print_usage()
{
        printf("libcamera capture\n");
        printf("Usage:\n");
        printf("\t-t libcamera[:d=<index>|camera=<index>][:size=WxH][:fps=N][:format=YUV420][:list|caps|help|fullhelp]\n");
        printf("\n");
}

void show_fullhelp()
{
        print_usage();
        printf("Without overrides, libcamera's default VideoRecording configuration is used.\n");
        printf("d=<index>, camera=<index> select a libcamera device; both names are aliases.\n");
        printf("size=WxH selects the output stream size.\n");
        printf("fps=N requests frame rate through FrameDurationLimits when streaming starts.\n");
        printf("format=YUV420 selects the output pixel format. Other formats are not supported yet.\n");
        printf("list prints only available cameras.\n");
        printf("caps prints detailed libcamera StreamFormats and may be long.\n");
        printf("help prints a compact device and output-size overview.\n");
        printf("fullhelp prints this detailed parameter description.\n");
        printf("mode=N is intentionally not part of the public libcamera API because libcamera/RPi sensor modes are a different layer than VideoRecording output.\n");
        printf("The currently supported output format for future frame handoff is YUV420.\n");
        printf("Status: capture is not implemented yet.\n");
}

void show_help_header()
{
        print_usage();
        printf("Examples:\n");
        printf("\t-t libcamera\n");
        printf("\t-t libcamera:d=0:size=1280x720:fps=50:format=YUV420\n");
        printf("\t-t libcamera:list\n");
        printf("\t-t libcamera:caps\n");
        printf("\n");
        printf("Default uses libcamera's VideoRecording configuration.\n");
        printf("Supported output format for future frame handoff: YUV420.\n");
        printf("Use -t libcamera:fullhelp for parameter details and -t libcamera:caps for full capabilities.\n");
        printf("\n");
}

int parse_fmt(std::string_view fmt, libcamera_options *opts)
{
        while (!fmt.empty()) {
                auto tok = tokenize(fmt, ':', '"');

                if (tok == "help") {
                        opts->action = libcamera_options::Action::Help;
                        continue;
                } else if (tok == "fullhelp") {
                        opts->action = libcamera_options::Action::Fullhelp;
                        continue;
                }

                auto key = tokenize(tok, '=', '"');
                auto val = tokenize(tok, '=', '"');

                if (key == "list" && val.empty()) {
                        opts->action = libcamera_options::Action::List;
                } else if (key == "caps" && val.empty()) {
                        opts->action = libcamera_options::Action::Caps;
                } else if (key == "camera" || key == "d") {
                        if (!parse_num(val, opts->camera_index)) {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "failed to parse camera index\n");
                                return VIDCAP_INIT_FAIL;
                        }
                } else if (key == "size") {
                        auto width = tokenize(val, 'x', '"');
                        auto height = tokenize(val, 'x', '"');
                        unsigned int parsed_width = 0;
                        unsigned int parsed_height = 0;
                        if (!parse_num(width, parsed_width) ||
                                        !parse_num(height, parsed_height) ||
                                        !val.empty() || parsed_width == 0 ||
                                        parsed_height == 0) {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "failed to parse size\n");
                                return VIDCAP_INIT_FAIL;
                        }
                        opts->size = { parsed_width, parsed_height };
                        opts->size_set = true;
                } else if (key == "mode") {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "mode is not supported by "
                                        "libcamera module; use "
                                        "size/fps/format\n");
                        return VIDCAP_INIT_FAIL;
                } else if (key == "fps") {
                        if (!parse_num(val, opts->fps) || opts->fps == 0) {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "failed to parse fps\n");
                                return VIDCAP_INIT_FAIL;
                        }
                        opts->fps_set = true;
                } else if (key == "format") {
                        if (val == "YUV420") {
                                opts->format_yuv420 = true;
                        } else {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "unsupported format\n");
                                return VIDCAP_INIT_FAIL;
                        }
                } else if (!key.empty()) {
                        log_msg(LOG_LEVEL_ERROR, MOD_NAME "unknown parameter: %.*s\n",
                                        static_cast<int>(key.size()), key.data());
                        return VIDCAP_INIT_FAIL;
                }
        }

        return VIDCAP_INIT_OK;
}

void log_stream_formats(const libcamera::StreamConfiguration &stream_config)
{
        const libcamera::StreamFormats &formats = stream_config.formats();
        for (const libcamera::PixelFormat &pixfmt : formats.pixelformats()) {
                const std::vector<libcamera::Size> sizes = formats.sizes(pixfmt);
                if (sizes.empty()) {
                        const libcamera::SizeRange range = formats.range(pixfmt);
                        log_msg(LOG_LEVEL_INFO,
                                        MOD_NAME "format %s size range %s-%s\n",
                                        pixfmt.toString().c_str(),
                                        range.min.toString().c_str(),
                                        range.max.toString().c_str());
                } else {
                        for (const libcamera::Size &size : sizes) {
                                log_msg(LOG_LEVEL_INFO,
                                                MOD_NAME "format %s size %s\n",
                                                pixfmt.toString().c_str(),
                                                size.toString().c_str());
                        }
                }
        }
}

bool size_in_list(const std::vector<libcamera::Size> &sizes,
                const libcamera::Size &needle)
{
        return std::find(sizes.begin(), sizes.end(), needle) != sizes.end();
}

std::vector<libcamera::Size> get_common_yuv420_sizes(
                const libcamera::StreamConfiguration &stream_config)
{
        const std::vector<libcamera::Size> sizes =
                stream_config.formats().sizes(libcamera::formats::YUV420);
        std::vector<libcamera::Size> common_available_sizes;

        if (sizes.empty()) {
                return common_available_sizes;
        }

        const std::vector<libcamera::Size> common_sizes = {
                { 640, 480 },
                { 1280, 720 },
                { 1920, 1080 },
                { 3840, 2160 },
        };
        for (const libcamera::Size &size : common_sizes) {
                if (size_in_list(sizes, size)) {
                        common_available_sizes.push_back(size);
                }
        }

        return common_available_sizes;
}

void print_common_yuv420_sizes(
                const libcamera::StreamConfiguration &stream_config)
{
        const std::vector<libcamera::Size> sizes =
                stream_config.formats().sizes(libcamera::formats::YUV420);
        const std::vector<libcamera::Size> common_sizes =
                get_common_yuv420_sizes(stream_config);

        if (sizes.empty()) {
                const libcamera::SizeRange range =
                        stream_config.formats().range(libcamera::formats::YUV420);
                printf("  YUV420 size range: %s-%s\n",
                                range.min.toString().c_str(),
                                range.max.toString().c_str());
        } else if (!common_sizes.empty()) {
                printf("  Common YUV420 sizes:\n");
                for (const libcamera::Size &size : common_sizes) {
                        printf("    %s\n", size.toString().c_str());
                }
        } else {
                printf("  Use -t libcamera:caps for full YUV420 size list\n");
        }
        printf("  FPS: request with fps=N; exact support depends on camera/pipeline\n");
}

bool print_camera_help(const std::shared_ptr<libcamera::Camera> &camera,
                size_t index)
{
        if (camera->acquire() != 0) {
                printf("Device %zu) %s\n  unable to acquire camera\n",
                                index, camera->id().c_str());
                return false;
        }

        bool success = false;
        {
                std::unique_ptr<libcamera::CameraConfiguration> config =
                        camera->generateConfiguration({
                                libcamera::StreamRole::VideoRecording });
                printf("Device %zu) %s\n", index, camera->id().c_str());
                if (!config || config->empty()) {
                        printf("  unable to generate VideoRecording configuration\n");
                } else {
                        libcamera::StreamConfiguration &stream_config =
                                config->at(0);
                        printf("  Default: %s %s\n",
                                        stream_config.pixelFormat.toString().c_str(),
                                        stream_config.size.toString().c_str());
                        print_common_yuv420_sizes(stream_config);
                        success = true;
                }
        }

        camera->release();
        return success;
}

bool inspect_camera_caps(const std::shared_ptr<libcamera::Camera> &camera)
{
        if (camera->acquire() != 0) {
                log_msg(LOG_LEVEL_ERROR, MOD_NAME "failed to acquire camera\n");
                return false;
        }

        bool success = false;
        {
                std::unique_ptr<libcamera::CameraConfiguration> config =
                        camera->generateConfiguration({
                                libcamera::StreamRole::VideoRecording });
                if (!config || config->empty()) {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "failed to generate "
                                        "VideoRecording configuration\n");
                } else {
                        libcamera::StreamConfiguration &stream_config =
                                config->at(0);
                        log_stream_config("generated stream", stream_config);
                        log_stream_formats(stream_config);
                        success = true;
                }
        }

        camera->release();
        return success;
}

bool configure_camera(vidcap_libcamera_state *s, const libcamera_options &opts)
{
        if (s->camera->acquire() != 0) {
                log_msg(LOG_LEVEL_ERROR, MOD_NAME "failed to acquire camera\n");
                return false;
        }
        s->camera_acquired = true;

        std::unique_ptr<libcamera::CameraConfiguration> config =
                s->camera->generateConfiguration({
                        libcamera::StreamRole::VideoRecording });
        if (!config || config->empty()) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "failed to generate VideoRecording "
                                "configuration\n");
                return false;
        }

        libcamera::StreamConfiguration &stream_config = config->at(0);
        log_stream_config("generated stream", stream_config);

        const libcamera::PixelFormat requested_pixfmt =
                libcamera::formats::YUV420;
        const libcamera::Size requested_size = opts.size;
        libcamera::Size effective_requested_size = requested_size;
        bool check_requested_format = opts.format_yuv420;
        bool check_requested_size = opts.size_set;

        if (opts.size_set) {
                stream_config.size = requested_size;
        }
        if (opts.format_yuv420) {
                stream_config.pixelFormat = requested_pixfmt;
        }
        if (stream_config.bufferCount < 4) {
                stream_config.bufferCount = 4;
        }

        if (opts.size_set || opts.format_yuv420 || opts.fps_set) {
                log_msg(LOG_LEVEL_INFO,
                                MOD_NAME "requested overrides: size=%s "
                                "format=%s fps=%s bufferCount=%u\n",
                                check_requested_size ?
                                        effective_requested_size.toString().c_str() :
                                        "default",
                                check_requested_format ?
                                        requested_pixfmt.toString().c_str() :
                                        "default",
                                opts.fps_set ? "set" : "default",
                                stream_config.bufferCount);
        }
        if (opts.fps_set) {
                log_msg(LOG_LEVEL_INFO,
                                MOD_NAME "requested fps=%u; will apply during "
                                "camera start if supported\n",
                                opts.fps);
        }

        libcamera::CameraConfiguration::Status status = config->validate();
        log_msg(LOG_LEVEL_INFO, MOD_NAME "configuration validation: %s\n",
                        validation_status_to_string(status));
        log_stream_config("validated stream", stream_config);

        if (status == libcamera::CameraConfiguration::Invalid) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "requested configuration is invalid\n");
                return false;
        }
        if (stream_config.pixelFormat != requested_pixfmt) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "only YUV420 is supported for "
                                "UltraGrid handoff, got %s\n",
                                stream_config.pixelFormat.toString().c_str());
                return false;
        }
        if (check_requested_format && stream_config.pixelFormat != requested_pixfmt) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "requested YUV420 was adjusted to %s\n",
                                stream_config.pixelFormat.toString().c_str());
                return false;
        }
        if (check_requested_size && stream_config.size != effective_requested_size) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "requested size %s was adjusted to %s\n",
                                effective_requested_size.toString().c_str(),
                                stream_config.size.toString().c_str());
                return false;
        }

        int configure_ret = s->camera->configure(config.get());
        if (configure_ret != 0) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "camera configure failed: %d\n",
                                configure_ret);
                return false;
        }
        log_msg(LOG_LEVEL_INFO, MOD_NAME "camera configure succeeded\n");

        s->stream = stream_config.stream();
        if (s->stream == nullptr) {
                log_msg(LOG_LEVEL_ERROR, MOD_NAME "configured stream is null\n");
                return false;
        }
        s->stream_config = stream_config;

        const unsigned int width = stream_config.size.width;
        const unsigned int height = stream_config.size.height;
        if (stream_config.stride != width) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "unsupported YUV420 stride %u for "
                                "width %u\n",
                                stream_config.stride, width);
                return false;
        }

        s->desc = {
                width,
                height,
                I420,
                opts.fps_set ? static_cast<double>(opts.fps) : 0.0,
                PROGRESSIVE,
                1,
        };
        s->frame = vf_alloc_desc_data(s->desc);
        if (s->frame == nullptr) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "failed to allocate UltraGrid frame\n");
                return false;
        }

        s->allocator = std::make_unique<libcamera::FrameBufferAllocator>(
                        s->camera);
        if (!setup_mmaps_and_requests(s)) {
                return false;
        }
        if (!start_camera(s, opts)) {
                return false;
        }

        log_msg(LOG_LEVEL_INFO,
                        MOD_NAME "libcamera capture started: %ux%u I420\n",
                        width, height);
        return true;
}

int vidcap_libcamera_init(const struct vidcap_params *params, void **state)
{
        libcamera_options opts;
        const char *fmt = vidcap_params_get_fmt(params);

        if (fmt != nullptr) {
                int ret = parse_fmt(fmt, &opts);
                if (ret != VIDCAP_INIT_OK) {
                        return ret;
                }
        }

        *state = nullptr;

        if (opts.action == libcamera_options::Action::Fullhelp) {
                show_fullhelp();
                return VIDCAP_INIT_NOERR;
        }

        auto camera_manager = std::make_unique<libcamera::CameraManager>();
        if (camera_manager->start() != 0) {
                log_msg(LOG_LEVEL_ERROR, MOD_NAME "failed to start CameraManager\n");
                return VIDCAP_INIT_FAIL;
        }

        bool have_config = false;
        vidcap_libcamera_state *new_state = nullptr;
        {
                auto cameras = camera_manager->cameras();
                if (cameras.empty()) {
                        log_msg(LOG_LEVEL_ERROR, MOD_NAME "no cameras found\n");
                } else if (opts.action == libcamera_options::Action::Help) {
                        show_help_header();
                        for (size_t i = 0; i < cameras.size(); ++i) {
                                print_camera_help(cameras[i], i);
                        }
                        have_config = true;
                } else if (opts.action == libcamera_options::Action::List) {
                        for (size_t i = 0; i < cameras.size(); ++i) {
                                printf("Device %zu) %s\n", i,
                                                cameras[i]->id().c_str());
                        }
                        have_config = true;
                } else if (opts.action == libcamera_options::Action::Caps) {
                        for (size_t i = 0; i < cameras.size(); ++i) {
                                log_msg(LOG_LEVEL_INFO,
                                                MOD_NAME "camera %zu: %s\n",
                                                i, cameras[i]->id().c_str());
                                inspect_camera_caps(cameras[i]);
                        }
                        have_config = true;
                } else if (opts.camera_index >= cameras.size()) {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "camera index %zu out of range "
                                        "(found %zu cameras)\n",
                                        opts.camera_index, cameras.size());
                } else {
                        std::shared_ptr<libcamera::Camera> camera =
                                cameras[opts.camera_index];
                        log_msg(LOG_LEVEL_INFO, MOD_NAME "using camera %zu: %s\n",
                                        opts.camera_index, camera->id().c_str());
                        new_state = new vidcap_libcamera_state();
                        new_state->camera_manager = std::move(camera_manager);
                        new_state->camera = camera;
                        have_config = configure_camera(new_state, opts);
                }
        }

        if (opts.action == libcamera_options::Action::Help ||
                        opts.action == libcamera_options::Action::List ||
                        opts.action == libcamera_options::Action::Caps) {
                camera_manager->stop();
                return VIDCAP_INIT_NOERR;
        }
        if (!have_config) {
                if (new_state != nullptr) {
                        vidcap_libcamera_cleanup(new_state);
                } else if (camera_manager) {
                        camera_manager->stop();
                }
                return VIDCAP_INIT_FAIL;
        }

        *state = new_state;
        return VIDCAP_INIT_OK;
}

void vidcap_libcamera_done(void *state)
{
        vidcap_libcamera_cleanup(static_cast<vidcap_libcamera_state *>(state));
}

struct video_frame *vidcap_libcamera_grab(void *state,
                struct audio_frame **audio)
{
        auto *s = static_cast<vidcap_libcamera_state *>(state);
        *audio = nullptr;

        while (true) {
                libcamera::Request *request = nullptr;
                {
                        std::unique_lock<std::mutex> lock(s->lock);
                        s->completed_cv.wait(lock, [s] {
                                return s->stopping ||
                                        !s->completed_requests.empty();
                        });
                        if (s->stopping) {
                                return nullptr;
                        }
                        request = s->completed_requests.front();
                        s->completed_requests.pop_front();
                }

                libcamera::FrameBuffer *buffer = request->findBuffer(s->stream);
                if (buffer == nullptr) {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "completed request has no "
                                        "buffer for stream\n");
                } else {
                        const unsigned int width = s->stream_config.size.width;
                        const unsigned int height = s->stream_config.size.height;
                        const size_t y_size = static_cast<size_t>(width) * height;
                        const size_t chroma_size = y_size / 4;
                        const size_t frame_size = y_size + 2 * chroma_size;
                        const auto planes = buffer->planes();
                        const auto metadata_planes = buffer->metadata().planes();
                        auto mapped_it = s->mapped_buffers.find(buffer);

                        if (planes.size() != 3 ||
                                        metadata_planes.size() < 3 ||
                                        mapped_it == s->mapped_buffers.end() ||
                                        mapped_it->second.planes.size() != 3 ||
                                        s->frame->tiles[0].data_len < frame_size ||
                                        mapped_it->second.planes[0].len < y_size ||
                                        mapped_it->second.planes[1].len < chroma_size ||
                                        mapped_it->second.planes[2].len < chroma_size ||
                                        metadata_planes[0].bytesused < y_size ||
                                        metadata_planes[1].bytesused < chroma_size ||
                                        metadata_planes[2].bytesused < chroma_size) {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "unexpected YUV420 "
                                                "buffer layout, skipping frame\n");
                        } else {
                                char *dst = s->frame->tiles[0].data;
                                const mapped_buffer &mapped = mapped_it->second;
                                if (s->copied_frames == 0) {
                                        log_frame_buffer(*buffer);
                                }
                                memcpy(dst, mapped.planes[0].data, y_size);
                                memcpy(dst + y_size, mapped.planes[1].data,
                                                chroma_size);
                                memcpy(dst + y_size + chroma_size,
                                                mapped.planes[2].data,
                                                chroma_size);
                                s->frame->tiles[0].data_len = frame_size;
                                s->frame->timestamp =
                                        buffer->metadata().timestamp * 90 / 1000000;
                                if (s->copied_frames < 5) {
                                        log_msg(LOG_LEVEL_INFO,
                                                        MOD_NAME "grab copied "
                                                        "frame %u: %ux%u "
                                                        "Y=%zu U=%zu V=%zu "
                                                        "sequence=%u\n",
                                                        s->copied_frames + 1,
                                                        width, height, y_size,
                                                        chroma_size,
                                                        chroma_size,
                                                        buffer->metadata().sequence);
                                }
                                s->copied_frames += 1;

                                request->reuse(libcamera::Request::ReuseBuffers);
                                int queue_ret = s->camera->queueRequest(request);
                                if (queue_ret != 0) {
                                        log_msg(LOG_LEVEL_ERROR,
                                                        MOD_NAME "failed to "
                                                        "requeue request: %d\n",
                                                        queue_ret);
                                }
                                return s->frame;
                        }
                }

                request->reuse(libcamera::Request::ReuseBuffers);
                int queue_ret = s->camera->queueRequest(request);
                if (queue_ret != 0) {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "failed to requeue request: %d\n",
                                        queue_ret);
                        return nullptr;
                }
        }
}

const struct video_capture_info vidcap_libcamera_info = {
        vidcap_libcamera_probe,
        vidcap_libcamera_init,
        vidcap_libcamera_done,
        vidcap_libcamera_grab,
        MOD_NAME,
};

} // namespace

REGISTER_MODULE(libcamera, &vidcap_libcamera_info, LIBRARY_CLASS_VIDEO_CAPTURE,
                VIDEO_CAPTURE_ABI_VERSION);
