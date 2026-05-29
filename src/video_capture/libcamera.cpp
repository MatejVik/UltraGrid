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

#include <cassert>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string_view>
#include <vector>

#include <libcamera/libcamera.h>
#include <libcamera/formats.h>

#include "debug.h"
#include "lib_common.h"
#include "utils/string_view_utils.hpp"
#include "video_capture.h"
#include "video_capture_params.h"

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
        unsigned int mode = 0;
        unsigned int fps = 0;
        bool size_set = false;
        bool mode_set = false;
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
        printf("\t-t libcamera[:d=<index>|camera=<index>][:mode=N|size=WxH][:fps=N][:format=YUV420][:list|caps|help|fullhelp]\n");
        printf("\n");
}

void show_fullhelp()
{
        print_usage();
        printf("Without overrides, libcamera's default VideoRecording configuration is used.\n");
        printf("d=<index>, camera=<index> select a libcamera device; both names are aliases.\n");
        printf("size=WxH overrides the VideoRecording stream size.\n");
        printf("fps=N is parsed for future use; it will be applied later through FrameDurationLimits at camera start.\n");
        printf("format=YUV420 requests YUV420 output. Other formats are not supported yet.\n");
        printf("list prints only available cameras.\n");
        printf("caps prints detailed libcamera StreamFormats and may be long.\n");
        printf("help prints a compact device and mode overview.\n");
        printf("fullhelp prints this detailed parameter description.\n");
        printf("mode=N selects an internal mode printed by libcamera:help, not a raw sensor mode.\n");
        printf("mode=0 uses the default VideoRecording configuration.\n");
        printf("mode=1+ selects compact YUV420 modes derived from VideoRecording StreamFormats.\n");
        printf("The currently supported output format for future frame handoff is YUV420.\n");
        printf("Status: capture is not implemented yet.\n");
}

void show_help_header()
{
        print_usage();
        printf("Examples:\n");
        printf("\t-t libcamera\n");
        printf("\t-t libcamera:d=0:mode=2:fps=50\n");
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
                        if (opts->mode_set) {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "mode and size cannot "
                                                "be combined\n");
                                return VIDCAP_INIT_FAIL;
                        }
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
                        if (opts->size_set) {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "mode and size cannot "
                                                "be combined\n");
                                return VIDCAP_INIT_FAIL;
                        }
                        if (!parse_num(val, opts->mode)) {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "failed to parse mode\n");
                                return VIDCAP_INIT_FAIL;
                        }
                        opts->mode_set = true;
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

std::vector<libcamera::Size> get_compact_yuv420_modes(
                const libcamera::StreamConfiguration &stream_config)
{
        const std::vector<libcamera::Size> sizes =
                stream_config.formats().sizes(libcamera::formats::YUV420);
        std::vector<libcamera::Size> modes;

        if (sizes.empty()) {
                return modes;
        }

        const std::vector<libcamera::Size> common_sizes = {
                { 640, 480 },
                { 1280, 720 },
                { 1920, 1080 },
                { 3840, 2160 },
        };
        for (const libcamera::Size &size : common_sizes) {
                if (size_in_list(sizes, size)) {
                        modes.push_back(size);
                }
        }

        if (modes.empty()) {
                const size_t max_modes = std::min<size_t>(sizes.size(), 6);
                for (size_t i = 0; i < max_modes; ++i) {
                        modes.push_back(sizes[i]);
                }
        }

        return modes;
}

void print_compact_yuv420_modes(
                const libcamera::StreamConfiguration &stream_config)
{
        const std::vector<libcamera::Size> sizes =
                stream_config.formats().sizes(libcamera::formats::YUV420);
        const std::vector<libcamera::Size> modes =
                get_compact_yuv420_modes(stream_config);

        if (sizes.empty()) {
                const libcamera::SizeRange range =
                        stream_config.formats().range(libcamera::formats::YUV420);
                printf("  YUV420 size range: %s-%s\n",
                                range.min.toString().c_str(),
                                range.max.toString().c_str());
                return;
        }

        printf("  Common YUV420 modes:\n");
        for (size_t i = 0; i < modes.size(); ++i) {
                printf("    Mode %zu) YUV420 %s\n", i + 1,
                                modes[i].toString().c_str());
        }

        if (sizes.size() > modes.size()) {
                printf("    ... use -t libcamera:caps for full list\n");
        }
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
                        printf("  Mode 0) default %s %s\n",
                                        stream_config.pixelFormat.toString().c_str(),
                                        stream_config.size.toString().c_str());
                        print_compact_yuv420_modes(stream_config);
                        success = true;
                }
        }

        camera->release();
        return success;
}

bool inspect_camera_config(const std::shared_ptr<libcamera::Camera> &camera,
                const libcamera_options &opts, bool caps_only)
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
                        if (caps_only) {
                                log_stream_formats(stream_config);
                                success = true;
                        } else {
                                do {
                                        const libcamera::PixelFormat requested_pixfmt =
                                                libcamera::formats::YUV420;
                                        const libcamera::Size requested_size =
                                                opts.size;
                                        libcamera::Size effective_requested_size =
                                                requested_size;
                                        bool mode_ok = true;
                                        bool check_requested_format =
                                                opts.format_yuv420;
                                        bool check_requested_size = opts.size_set;

                                        if (opts.mode_set) {
                                                if (opts.mode == 0) {
                                                        log_msg(LOG_LEVEL_INFO,
                                                                        MOD_NAME "selected "
                                                                        "mode 0: default "
                                                                        "VideoRecording\n");
                                                } else {
                                                        const std::vector<libcamera::Size>
                                                                modes =
                                                                        get_compact_yuv420_modes(
                                                                                        stream_config);
                                                        if (opts.mode > modes.size()) {
                                                                log_msg(LOG_LEVEL_ERROR,
                                                                                MOD_NAME "mode %u "
                                                                                "out of range; "
                                                                                "use -t "
                                                                                "libcamera:help "
                                                                                "or -t "
                                                                                "libcamera:caps\n",
                                                                                opts.mode);
                                                                mode_ok = false;
                                                                break;
                                                        }
                                                        effective_requested_size =
                                                                modes[opts.mode - 1];
                                                        stream_config.size =
                                                                effective_requested_size;
                                                        stream_config.pixelFormat =
                                                                requested_pixfmt;
                                                        check_requested_format = true;
                                                        check_requested_size = true;
                                                        log_msg(LOG_LEVEL_INFO,
                                                                        MOD_NAME "selected "
                                                                        "mode %u: YUV420 "
                                                                        "%s\n",
                                                                        opts.mode,
                                                                        stream_config.size
                                                                                .toString().c_str());
                                                }
                                        }
                                        if (!mode_ok) {
                                                break;
                                        }
                                        if (opts.size_set) {
                                                stream_config.size = requested_size;
                                        }
                                        if (opts.format_yuv420) {
                                                stream_config.pixelFormat =
                                                        requested_pixfmt;
                                        }
                                        if (stream_config.bufferCount < 4) {
                                                stream_config.bufferCount = 4;
                                        }

                                        if (opts.mode_set || opts.size_set ||
                                                        opts.format_yuv420 ||
                                                        opts.fps_set) {
                                                log_msg(LOG_LEVEL_INFO,
                                                                MOD_NAME "requested "
                                                                "overrides: mode=%s "
                                                                "size=%s format=%s "
                                                                "fps=%s bufferCount=%u\n",
                                                                opts.mode_set ?
                                                                        "set" : "default",
                                                                check_requested_size ?
                                                                        effective_requested_size
                                                                                .toString().c_str() :
                                                                        "default",
                                                                check_requested_format ?
                                                                        requested_pixfmt
                                                                                .toString().c_str() :
                                                                        "default",
                                                                opts.fps_set ? "set" :
                                                                        "default",
                                                                stream_config.bufferCount);
                                        }
                                if (opts.fps_set) {
                                        log_msg(LOG_LEVEL_INFO,
                                                        MOD_NAME "requested fps=%u; "
                                                        "fps not applied yet\n",
                                                        opts.fps);
                                }

                                libcamera::CameraConfiguration::Status status =
                                        config->validate();
                                log_msg(LOG_LEVEL_INFO,
                                                MOD_NAME "configuration "
                                                "validation: %s\n",
                                                validation_status_to_string(status));
                                log_stream_config("validated stream",
                                                stream_config);

                                if (status ==
                                                libcamera::CameraConfiguration::Invalid) {
                                        log_msg(LOG_LEVEL_ERROR,
                                                        MOD_NAME "requested "
                                                        "configuration is invalid\n");
                                } else if (check_requested_format &&
                                                stream_config.pixelFormat !=
                                                        requested_pixfmt) {
                                        log_msg(LOG_LEVEL_ERROR,
                                                        MOD_NAME "requested "
                                                        "YUV420 was adjusted to %s\n",
                                                        stream_config.pixelFormat
                                                                .toString().c_str());
                                } else if (check_requested_size &&
                                                stream_config.size !=
                                                        effective_requested_size) {
                                        log_msg(LOG_LEVEL_ERROR,
                                                        MOD_NAME "requested size "
                                                        "%s was adjusted to %s\n",
                                                        effective_requested_size
                                                                .toString().c_str(),
                                                        stream_config.size
                                                                .toString().c_str());
                                } else {
                                        int configure_ret =
                                                camera->configure(config.get());
                                        if (configure_ret == 0) {
                                                log_msg(LOG_LEVEL_INFO,
                                                                MOD_NAME "camera "
                                                                "configure "
                                                                "succeeded\n");
                                                success = true;
                                        } else {
                                                log_msg(LOG_LEVEL_ERROR,
                                                                MOD_NAME "camera "
                                                                "configure "
                                                                "failed: %d\n",
                                                                configure_ret);
                                        }
                                }
                                } while (false);
                        }
                }
        }

        camera->release();
        return success;
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

        libcamera::CameraManager camera_manager;
        if (camera_manager.start() != 0) {
                log_msg(LOG_LEVEL_ERROR, MOD_NAME "failed to start CameraManager\n");
                return VIDCAP_INIT_FAIL;
        }

        bool have_config = false;
        {
                auto cameras = camera_manager.cameras();
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
                                inspect_camera_config(cameras[i], opts, true);
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
                        have_config = inspect_camera_config(camera, opts, false);
                }
        }

        camera_manager.stop();

        if (opts.action == libcamera_options::Action::Help ||
                        opts.action == libcamera_options::Action::List ||
                        opts.action == libcamera_options::Action::Caps) {
                return VIDCAP_INIT_NOERR;
        }
        if (!have_config) {
                return VIDCAP_INIT_FAIL;
        }

        log_msg(LOG_LEVEL_ERROR,
                        MOD_NAME "libcamera frame capture not implemented yet\n");
        return VIDCAP_INIT_FAIL;
}

void vidcap_libcamera_done(void *state)
{
        assert(state == nullptr);
}

struct video_frame *vidcap_libcamera_grab(void *state,
                struct audio_frame **audio)
{
        assert(state == nullptr);
        *audio = nullptr;
        return nullptr;
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
