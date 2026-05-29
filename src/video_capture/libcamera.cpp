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

#include <libcamera/libcamera.h>

#include "debug.h"
#include "lib_common.h"
#include "video_capture.h"
#include "video_capture_params.h"

#define MOD_NAME "[libcamera] "

namespace {

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

        const auto &cameras = camera_manager.cameras();
        if (cameras.empty()) {
                camera_manager.stop();
                return;
        }

        auto *devices = static_cast<device_info *>(
                calloc(cameras.size(), sizeof(device_info)));
        if (devices == nullptr) {
                camera_manager.stop();
                return;
        }

        for (size_t i = 0; i < cameras.size(); ++i) {
                snprintf(devices[i].dev, sizeof devices[i].dev, ":camera=%zu", i);
                snprintf(devices[i].name, sizeof devices[i].name, "%s",
                                cameras[i]->id().c_str());
        }

        *available_cards = devices;
        *count = static_cast<int>(cameras.size());
        camera_manager.stop();
}

void show_help()
{
        printf("libcamera capture\n");
        printf("Usage:\n");
        printf("\t-t libcamera[:camera=<index>][:help]\n");
        printf("\n");
        printf("Status: capture is not implemented yet.\n");
}

int vidcap_libcamera_init(const struct vidcap_params *params, void **state)
{
        const char *fmt = vidcap_params_get_fmt(params);

        if (fmt != nullptr && strcmp(fmt, "help") == 0) {
                show_help();
                return VIDCAP_INIT_NOERR;
        }

        *state = nullptr;
        log_msg(LOG_LEVEL_ERROR, MOD_NAME "libcamera capture not implemented yet\n");
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
