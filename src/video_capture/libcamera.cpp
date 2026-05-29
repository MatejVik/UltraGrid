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
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string_view>

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
        size_t camera_index = 0;
        libcamera::Size size = { 1280, 720 };
        unsigned int fps = 50;
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

void show_help()
{
        printf("libcamera capture\n");
        printf("Usage:\n");
        printf("\t-t libcamera[:camera=<index>][:size=WxH][:fps=N][:help]\n");
        printf("\n");
        printf("Defaults: size=1280x720, fps=50\n");
        printf("Status: capture is not implemented yet.\n");
}

int parse_fmt(std::string_view fmt, libcamera_options *opts)
{
        while (!fmt.empty()) {
                auto tok = tokenize(fmt, ':', '"');

                if (tok == "help") {
                        show_help();
                        return VIDCAP_INIT_NOERR;
                }

                auto key = tokenize(tok, '=', '"');
                auto val = tokenize(tok, '=', '"');

                if (key == "camera") {
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
                } else if (key == "fps") {
                        if (!parse_num(val, opts->fps) || opts->fps == 0) {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "failed to parse fps\n");
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

                        if (camera->acquire() != 0) {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "failed to acquire camera\n");
                        } else {
                                {
                                        std::unique_ptr<libcamera::CameraConfiguration>
                                                config =
                                                        camera->generateConfiguration({
                                                                libcamera::StreamRole::VideoRecording });
                                        if (!config || config->empty()) {
                                                log_msg(LOG_LEVEL_ERROR,
                                                                MOD_NAME "failed "
                                                                "to generate "
                                                                "VideoRecording "
                                                                "configuration\n");
                                        } else {
                                                libcamera::StreamConfiguration
                                                        &stream_config = config->at(0);
                                                log_stream_config(
                                                                "generated stream",
                                                                stream_config);

                                                const libcamera::PixelFormat
                                                        requested_pixfmt =
                                                                libcamera::formats::YUV420;
                                                const libcamera::Size requested_size =
                                                        opts.size;

                                                stream_config.size = requested_size;
                                                stream_config.pixelFormat =
                                                        requested_pixfmt;
                                                if (stream_config.bufferCount < 4) {
                                                        stream_config.bufferCount = 4;
                                                }

                                                log_msg(LOG_LEVEL_INFO,
                                                                MOD_NAME "requested "
                                                                "stream: pixelFormat=%s "
                                                                "size=%s fps=%u "
                                                                "bufferCount=%u\n",
                                                                requested_pixfmt
                                                                        .toString().c_str(),
                                                                requested_size
                                                                        .toString().c_str(),
                                                                opts.fps,
                                                                stream_config.bufferCount);
                                                log_msg(LOG_LEVEL_INFO,
                                                                MOD_NAME "fps not "
                                                                "applied yet\n");

                                                libcamera::CameraConfiguration::Status
                                                        status = config->validate();
                                                log_msg(LOG_LEVEL_INFO,
                                                                MOD_NAME "configuration "
                                                                "validation: %s\n",
                                                                validation_status_to_string(
                                                                        status));
                                                log_stream_config(
                                                                "validated stream",
                                                                stream_config);

                                                if (stream_config.pixelFormat !=
                                                                requested_pixfmt ||
                                                                stream_config.size !=
                                                                        requested_size) {
                                                        log_msg(LOG_LEVEL_ERROR,
                                                                        MOD_NAME "requested "
                                                                        "YUV420 %s was "
                                                                        "adjusted to "
                                                                        "%s %s\n",
                                                                        requested_size
                                                                                .toString().c_str(),
                                                                        stream_config
                                                                                .pixelFormat
                                                                                .toString().c_str(),
                                                                        stream_config.size
                                                                                .toString().c_str());
                                                } else if (status ==
                                                                libcamera::CameraConfiguration::Invalid) {
                                                        log_msg(LOG_LEVEL_ERROR,
                                                                        MOD_NAME "requested "
                                                                        "configuration is "
                                                                        "invalid\n");
                                                } else {
                                                        int configure_ret =
                                                                camera->configure(
                                                                                config.get());
                                                        if (configure_ret == 0) {
                                                                log_msg(LOG_LEVEL_INFO,
                                                                                MOD_NAME
                                                                                "camera configure "
                                                                                "succeeded\n");
                                                                have_config = true;
                                                        } else {
                                                                log_msg(LOG_LEVEL_ERROR,
                                                                                MOD_NAME
                                                                                "camera configure "
                                                                                "failed: %d\n",
                                                                                configure_ret);
                                                        }
                                                }
                                        }
                                }
                                camera->release();
                        }
                }
        }

        camera_manager.stop();

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
