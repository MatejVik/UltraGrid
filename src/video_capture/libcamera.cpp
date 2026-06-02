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
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <cinttypes>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <strings.h>
#include <deque>
#include <limits.h>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <libcamera/control_ids.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/libcamera.h>
#include <libcamera/formats.h>
#include <libcamera/property_ids.h>

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
                enum class HdrMode {
                        Off,
                        Sensor,
                };

                enum class AfSpeed {
                        Normal,
                        Fast,
                };

                enum class AfRange {
                        Normal,
                        Macro,
                        Full,
                };

                enum class NightMode {
                        Off,
                        Ir,
                        Lowlight,
                };

        enum class Action {
                Run,
                Help,
                Fullhelp,
                List,
                Caps,
                Test,
        } action = Action::Run;
        size_t camera_index = 0;
        libcamera::Size size = {};
        unsigned int sensor_width = 0;
        unsigned int sensor_height = 0;
        double fps = 0.0;
        bool size_set = false;
        bool sensor_set = false;
        bool fps_set = false;
        bool format_set = false;
                bool hdr_set = false;
                std::string format_name = "YUV420";
                std::string hdr_name = "unset";
                HdrMode hdr_mode = HdrMode::Off;
                bool focus_set = false;
                bool focus_afc = false;
                bool focus_manual = false;
                bool focus_m_set = false;
                bool focus_m_inf = false;
                bool af_speed_set = false;
                bool af_range_set = false;
                bool af_area_set = false;
                bool night_set = false;
                bool ae_set = false;
                bool ae_enable = false;
                bool metering_set = false;
                bool ae_diag_set = false;
                unsigned int ae_diag_interval = 30;
                bool ev_set = false;
                float ev = 0.0f;
                bool brightness_set = false;
                float brightness = 0.0f;
                bool saturation_set = false;
                float saturation = 1.0f;
                bool contrast_set = false;
                float contrast = 1.0f;
                double focus_m = 0.0;
                AfSpeed af_speed = AfSpeed::Normal;
                AfRange af_range = AfRange::Normal;
                NightMode night_mode = NightMode::Off;
                std::string af_area_name;
                std::string night_name;
                int32_t metering_mode = libcamera::controls::MeteringCentreWeighted;
                std::string metering_name;
        libcamera::PixelFormat pixel_format = libcamera::formats::YUV420;
        codec_t codec = I420;
        bool test_verbose = false;
};

struct sensor_mode_cap {
        libcamera::Size sensor_size;
        unsigned int max_fps;
        libcamera_options::HdrMode hdr_mode;
        const char *hdr_name;
        const char *note;
};

const sensor_mode_cap imx708_sensor_caps[] = {
        { { 4608, 2592 }, 14, libcamera_options::HdrMode::Off, "off",
                "full resolution" },
        { { 2304, 1296 }, 56, libcamera_options::HdrMode::Off, "off",
                "full FOV-ish 16:9" },
        { { 1536, 864 }, 120, libcamera_options::HdrMode::Off, "off",
                "cropped high-speed 16:9" },
        { { 2304, 1296 }, 30, libcamera_options::HdrMode::Sensor, "on",
                "sensor HDR, discovered by rpicam --list-cameras --hdr" },
};

struct libcamera_format_mapping {
        const char *name;
        libcamera::PixelFormat pixel_format;
        codec_t codec;
};

struct af_area_preset {
        const char *name;
        double x;
        double y;
        double width;
        double height;
};

const libcamera_format_mapping supported_formats[] = {
        { "YUV420", libcamera::formats::YUV420, I420 },
        { "I420", libcamera::formats::YUV420, I420 },
        { "UYVY", libcamera::formats::UYVY, UYVY },
        { "YUYV", libcamera::formats::YUYV, YUYV },
};

const af_area_preset af_area_presets[] = {
        { "full", 0.0, 0.0, 1.0, 1.0 },
        { "mid", 0.25, 0.25, 0.5, 0.5 },
        { "center", 1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0 },
};

const libcamera_format_mapping *find_supported_format(std::string_view name)
{
        for (const libcamera_format_mapping &mapping : supported_formats) {
                if (name.size() == strlen(mapping.name) &&
                                strncasecmp(name.data(), mapping.name,
                                        name.size()) == 0) {
                        return &mapping;
                }
        }
        return nullptr;
}

const af_area_preset *find_af_area_preset(std::string_view name)
{
        for (const af_area_preset &preset : af_area_presets) {
                if (name.size() == strlen(preset.name) &&
                                strncasecmp(name.data(), preset.name,
                                        name.size()) == 0) {
                        return &preset;
                }
        }
        return nullptr;
}

bool is_format_available(
                const libcamera::StreamConfiguration &stream_config,
                const libcamera::PixelFormat &pixel_format)
{
        const std::vector<libcamera::PixelFormat> formats =
                stream_config.formats().pixelformats();
        return std::find(formats.begin(), formats.end(), pixel_format) !=
                formats.end();
}

bool is_packed_422(codec_t codec)
{
        return codec == UYVY || codec == YUYV;
}

std::string to_lower(std::string_view value)
{
        std::string lowered(value);
        std::transform(lowered.begin(), lowered.end(), lowered.begin(),
                        [](unsigned char c) { return std::tolower(c); });
        return lowered;
}

std::string camera_model_string(
                const std::shared_ptr<libcamera::Camera> &camera)
{
        std::string model;
        const std::optional<std::string_view> prop_model =
                camera->properties().get(libcamera::properties::Model);
        if (prop_model && !prop_model->empty()) {
                model.assign(prop_model->begin(), prop_model->end());
        }
        if (model.empty()) {
                model = camera->id();
        }
        return model;
}

bool is_known_imx708_camera(const std::string &model_or_id)
{
        const std::string model = to_lower(model_or_id);
        return model.find("imx708") != std::string::npos ||
                model.find("imx708_wide") != std::string::npos ||
                model.find("imx708_wide_noir") != std::string::npos;
}

const char *hdr_mode_to_string(libcamera_options::HdrMode hdr_mode)
{
                switch (hdr_mode) {
                case libcamera_options::HdrMode::Off:
                        return "off";
                case libcamera_options::HdrMode::Sensor:
                        return "on";
                }
        return "unknown";
}

const sensor_mode_cap *find_known_sensor_cap(
                const std::shared_ptr<libcamera::Camera> &camera,
                const libcamera::Size &sensor_size,
                libcamera_options::HdrMode hdr_mode)
{
        if (!is_known_imx708_camera(camera_model_string(camera))) {
                return nullptr;
        }
        for (const sensor_mode_cap &cap : imx708_sensor_caps) {
                if (cap.sensor_size == sensor_size &&
                                cap.hdr_mode == hdr_mode) {
                        return &cap;
                }
        }
        return nullptr;
}

bool parse_size(std::string_view val, libcamera::Size *size)
{
        auto width = tokenize(val, 'x', '"');
        auto height = tokenize(val, 'x', '"');
        unsigned int parsed_width = 0;
        unsigned int parsed_height = 0;
        if (!parse_num(width, parsed_width) ||
                        !parse_num(height, parsed_height) || !val.empty() ||
                        parsed_width == 0 || parsed_height == 0) {
                return false;
        }
        *size = { parsed_width, parsed_height };
        return true;
}

bool parse_fps(std::string_view val, double *fps)
{
        std::string fps_str(val);
        char *end = nullptr;
        errno = 0;
        const double parsed_fps = strtod(fps_str.c_str(), &end);
        if (errno != 0 || end == fps_str.c_str() || *end != '\0' ||
                        !std::isfinite(parsed_fps) || parsed_fps <= 0.0) {
                return false;
        }
        *fps = parsed_fps;
        return true;
}

std::string read_symlink_path(const std::string &path)
{
        char buf[PATH_MAX] = {};
        const ssize_t ret = readlink(path.c_str(), buf, sizeof buf - 1);
        if (ret < 0) {
                return {};
        }
        buf[ret] = '\0';
        return buf;
}

std::string read_text_file(const std::string &path)
{
        FILE *file = fopen(path.c_str(), "r");
        if (file == nullptr) {
                return {};
        }
        char buf[256] = {};
        std::string result;
        while (fgets(buf, sizeof buf, file) != nullptr) {
                result += buf;
        }
        fclose(file);
        while (!result.empty() &&
                        (result.back() == '\n' || result.back() == '\r')) {
                result.pop_back();
        }
        return result;
}

bool xioctl(int fd, unsigned long request, void *arg)
{
        int ret = 0;
        do {
                ret = ioctl(fd, request, arg);
        } while (ret < 0 && errno == EINTR);
        return ret == 0;
}

struct v4l2_subdev_match {
        std::string dev_path;
        std::string sys_path;
        std::string name;
        bool camera_id_match = false;
};

bool of_node_matches_camera_id(const std::string &of_node,
                const std::string &camera_id)
{
        static const std::string dt_prefix = "/sys/firmware/devicetree/base";
        if (camera_id.empty() || of_node.empty()) {
                return false;
        }
        if (of_node == camera_id) {
                return true;
        }
        if (of_node.size() > dt_prefix.size() &&
                        of_node.compare(0, dt_prefix.size(), dt_prefix) == 0) {
                return of_node.substr(dt_prefix.size()) == camera_id;
        }
        return of_node.find(camera_id) != std::string::npos;
}

std::optional<v4l2_subdev_match> find_imx708_v4l_subdev(
                const std::shared_ptr<libcamera::Camera> &camera)
{
        const std::string camera_id = camera->id();
        DIR *dir = opendir("/sys/class/video4linux");
        if (dir == nullptr) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "failed to open /sys/class/video4linux: %s\n",
                                strerror(errno));
                return std::nullopt;
        }

        std::vector<v4l2_subdev_match> matches;
        while (dirent *entry = readdir(dir)) {
                const std::string name = entry->d_name;
                if (name.find("v4l-subdev") != 0) {
                        continue;
                }

                const std::string sys_path =
                        std::string("/sys/class/video4linux/") + name;
                const std::string module =
                        read_symlink_path(sys_path + "/device/driver/module");
                const std::string of_node =
                        read_symlink_path(sys_path + "/device/of_node");
                const std::string subdev_name =
                        read_text_file(sys_path + "/name");
                const std::string haystack = to_lower(module + " " +
                        of_node + " " + subdev_name);
                if (haystack.find("imx708") == std::string::npos) {
                        continue;
                }

                v4l2_subdev_match match = {};
                match.dev_path = std::string("/dev/") + name;
                match.sys_path = sys_path;
                match.name = subdev_name;
                match.camera_id_match =
                        of_node_matches_camera_id(of_node, camera_id);
                matches.push_back(match);
        }
        closedir(dir);

        std::vector<v4l2_subdev_match> exact_matches;
        for (const v4l2_subdev_match &match : matches) {
                if (match.camera_id_match) {
                        exact_matches.push_back(match);
                }
        }
        if (exact_matches.size() == 1) {
                return exact_matches.front();
        }
        if (exact_matches.size() > 1) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "multiple IMX708 v4l-subdevs match "
                                "camera id %s; refusing ambiguous HDR setup\n",
                                camera_id.c_str());
                return std::nullopt;
        }
        if (matches.size() == 1) {
                log_msg(LOG_LEVEL_WARNING,
                                MOD_NAME "using single IMX708 v4l-subdev "
                                "%s without camera-id match fallback\n",
                                matches.front().dev_path.c_str());
                return matches.front();
        }
        if (matches.empty()) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "no matching IMX708 v4l-subdev found "
                                "for camera %s\n",
                                camera_id.c_str());
        } else {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "multiple IMX708 v4l-subdevs found "
                                "without camera-id match; refusing ambiguous "
                                "HDR setup\n");
        }
        return std::nullopt;
}

bool set_imx708_wdr_control(
                const std::shared_ptr<libcamera::Camera> &camera,
                bool enable, bool *changed)
{
        *changed = false;
        const std::optional<v4l2_subdev_match> match =
                find_imx708_v4l_subdev(camera);
        if (!match) {
                return false;
        }

        log_msg(LOG_LEVEL_INFO,
                        MOD_NAME "selected v4l-subdev for WDR control: "
                        "%s (%s)\n",
                        match->dev_path.c_str(), match->name.c_str());

        const int fd = open(match->dev_path.c_str(), O_RDWR | O_CLOEXEC);
        if (fd < 0) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "failed to open %s for WDR control: "
                                "%s\n",
                                match->dev_path.c_str(), strerror(errno));
                return false;
        }

        struct v4l2_control control = {};
        control.id = V4L2_CID_WIDE_DYNAMIC_RANGE;
        if (!xioctl(fd, VIDIOC_G_CTRL, &control)) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "VIDIOC_G_CTRL "
                                "V4L2_CID_WIDE_DYNAMIC_RANGE failed on %s: "
                                "%s\n",
                                match->dev_path.c_str(), strerror(errno));
                close(fd);
                return false;
        }

        const int requested_value = enable ? 1 : 0;
        if (control.value == requested_value) {
                log_msg(LOG_LEVEL_INFO,
                                MOD_NAME "WDR/HDR already %s on %s\n",
                                enable ? "enabled" : "disabled",
                                match->dev_path.c_str());
                close(fd);
                return true;
        }

        control.value = requested_value;
        if (!xioctl(fd, VIDIOC_S_CTRL, &control)) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "VIDIOC_S_CTRL "
                                "V4L2_CID_WIDE_DYNAMIC_RANGE=%d failed on %s: "
                                "%s\n",
                                requested_value, match->dev_path.c_str(),
                                strerror(errno));
                close(fd);
                return false;
        }
        close(fd);

        *changed = true;
        log_msg(LOG_LEVEL_INFO,
                        MOD_NAME "WDR/HDR changed to %s on %s\n",
                        enable ? "enabled" : "disabled",
                        match->dev_path.c_str());
        return true;
}

bool resolve_and_apply_hdr_mode(libcamera_options *opts,
                const std::shared_ptr<libcamera::Camera> &camera,
                bool *wdr_changed)
{
        *wdr_changed = false;
        const std::string model = camera_model_string(camera);
        const bool is_imx708 = is_known_imx708_camera(model);
        const bool enable_sensor_hdr = opts->hdr_set &&
                opts->hdr_mode == libcamera_options::HdrMode::Sensor;

        log_msg(LOG_LEVEL_INFO,
                        MOD_NAME "selected camera model/id: %s / %s\n",
                        model.c_str(), camera->id().c_str());
        if (opts->hdr_set) {
                log_msg(LOG_LEVEL_INFO,
                                MOD_NAME "requested HDR mode: %s\n",
                                opts->hdr_name.c_str());
                log_msg(LOG_LEVEL_INFO,
                                MOD_NAME "resolved HDR mode: %s\n",
                                hdr_mode_to_string(opts->hdr_mode));
        } else {
                log_msg(LOG_LEVEL_INFO,
                                MOD_NAME "HDR not requested; ensuring SDR/WDR off\n");
        }

        if (enable_sensor_hdr && !is_imx708) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "hdr is currently supported only for "
                                "known IMX708 cameras; got %s\n",
                                model.c_str());
                return false;
        }

        if (!is_imx708) {
                return true;
        }

        if (enable_sensor_hdr) {
                const libcamera::Size hdr_sensor_size = { 2304, 1296 };
                if (opts->sensor_set) {
                        const libcamera::Size requested_sensor_size(
                                        opts->sensor_width, opts->sensor_height);
                        if (requested_sensor_size != hdr_sensor_size) {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "IMX708 sensor HDR "
                                                "supports only sensor=2304x1296 "
                                                "max_fps=30; omit sensor= or "
                                                "use sensor=2304x1296.\n");
                                return false;
                        }
                } else {
                        opts->sensor_width = hdr_sensor_size.width;
                        opts->sensor_height = hdr_sensor_size.height;
                        opts->sensor_set = true;
                        log_msg(LOG_LEVEL_INFO,
                                        MOD_NAME "HDR requested; using internal "
                                        "sensor output 2304x1296 max_fps=30\n");
                }
        }

        return set_imx708_wdr_control(camera, enable_sensor_hdr, wdr_changed);
}

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
        libcamera_options opts = {};
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
        unsigned int ae_diag_completed_frames = 0;
        std::chrono::steady_clock::time_point last_ae_ceiling_log = {};
        std::chrono::steady_clock::time_point last_focus_metadata_log = {};
        std::optional<int32_t> last_metadata_af_mode;
        std::optional<float> last_metadata_lens_position;
        std::optional<int32_t> last_metadata_af_state;
        std::optional<int32_t> last_metadata_focus_fom;

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

void unmap_mapped_buffer(mapped_buffer *mapped)
{
        for (mapped_plane &plane : mapped->planes) {
                if (plane.base != MAP_FAILED) {
                        munmap(plane.base, plane.map_len);
                        plane.base = MAP_FAILED;
                }
        }
        mapped->planes.clear();
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
                        unmap_mapped_buffer(&mapped);
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

const char *af_speed_to_string(libcamera_options::AfSpeed speed)
{
        switch (speed) {
        case libcamera_options::AfSpeed::Normal:
                return "normal";
        case libcamera_options::AfSpeed::Fast:
                return "fast";
        }
        return "unknown";
}

const char *af_range_to_string(libcamera_options::AfRange range)
{
        switch (range) {
        case libcamera_options::AfRange::Normal:
                return "normal";
        case libcamera_options::AfRange::Macro:
                return "macro";
        case libcamera_options::AfRange::Full:
                return "full";
        }
        return "unknown";
}

const char *night_mode_to_string(libcamera_options::NightMode mode)
{
        switch (mode) {
        case libcamera_options::NightMode::Off:
                return "off";
        case libcamera_options::NightMode::Ir:
                return "ir";
        case libcamera_options::NightMode::Lowlight:
                return "lowlight";
        }
        return "unknown";
}

const char *metering_mode_to_string(int32_t mode)
{
        switch (mode) {
        case libcamera::controls::MeteringCentreWeighted:
                return "centre";
        case libcamera::controls::MeteringSpot:
                return "spot";
        case libcamera::controls::MeteringMatrix:
                return "matrix";
        }
        return "unknown";
}

const char *af_mode_to_string(int32_t mode)
{
        switch (mode) {
        case libcamera::controls::AfModeManual:
                return "manual";
        case libcamera::controls::AfModeAuto:
                return "auto";
        case libcamera::controls::AfModeContinuous:
                return "continuous";
        }
        return "unknown";
}

const char *af_state_to_string(int32_t state)
{
        switch (state) {
        case libcamera::controls::AfStateIdle:
                return "idle";
        case libcamera::controls::AfStateScanning:
                return "scanning";
        case libcamera::controls::AfStateFocused:
                return "focused";
        case libcamera::controls::AfStateFailed:
                return "failed";
        }
        return "unknown";
}

const char *ae_state_to_string(int32_t state)
{
        switch (state) {
        case libcamera::controls::AeStateIdle:
                return "idle";
        case libcamera::controls::AeStateSearching:
                return "searching";
        case libcamera::controls::AeStateConverged:
                return "converged";
        }
        return "unknown";
}

bool parse_on_off(std::string_view val, bool *enabled)
{
        if (val.size() == strlen("on") &&
                        strncasecmp(val.data(), "on", val.size()) == 0) {
                *enabled = true;
                return true;
        }
        if (val.size() == strlen("off") &&
                        strncasecmp(val.data(), "off", val.size()) == 0) {
                *enabled = false;
                return true;
        }
        return false;
}

bool parse_night_mode(std::string_view val, libcamera_options *opts)
{
        if (val.size() == strlen("off") &&
                        strncasecmp(val.data(), "off", val.size()) == 0) {
                opts->night_set = true;
                opts->night_mode = libcamera_options::NightMode::Off;
                opts->night_name = "off";
                return true;
        }
        if (val.size() == strlen("ir") &&
                        strncasecmp(val.data(), "ir", val.size()) == 0) {
                opts->night_set = true;
                opts->night_mode = libcamera_options::NightMode::Ir;
                opts->night_name = "ir";
                return true;
        }
        if (val.size() == strlen("lowlight") &&
                        strncasecmp(val.data(), "lowlight",
                                val.size()) == 0) {
                opts->night_set = true;
                opts->night_mode = libcamera_options::NightMode::Lowlight;
                opts->night_name = "lowlight";
                return true;
        }
        return false;
}

bool parse_metering_mode(std::string_view val, libcamera_options *opts)
{
        if (val.size() == strlen("centre") &&
                        strncasecmp(val.data(), "centre", val.size()) == 0) {
                opts->metering_set = true;
                opts->metering_mode =
                        libcamera::controls::MeteringCentreWeighted;
                opts->metering_name = "centre";
                return true;
        }
        if (val.size() == strlen("spot") &&
                        strncasecmp(val.data(), "spot", val.size()) == 0) {
                opts->metering_set = true;
                opts->metering_mode = libcamera::controls::MeteringSpot;
                opts->metering_name = "spot";
                return true;
        }
        if (val.size() == strlen("matrix") &&
                        strncasecmp(val.data(), "matrix", val.size()) == 0) {
                opts->metering_set = true;
                opts->metering_mode = libcamera::controls::MeteringMatrix;
                opts->metering_name = "matrix";
                return true;
        }
        return false;
}

bool parse_focus_m(std::string_view val, libcamera_options *opts)
{
        if (val == "inf") {
                opts->focus_m_set = true;
                opts->focus_m_inf = true;
                opts->focus_m = std::numeric_limits<double>::infinity();
                return true;
        }

        std::string focus_m_str(val);
        char *end = nullptr;
        errno = 0;
        const double parsed_focus_m = strtod(focus_m_str.c_str(), &end);
        if (errno != 0 || end == focus_m_str.c_str() || *end != '\0' ||
                        !std::isfinite(parsed_focus_m) ||
                        parsed_focus_m <= 0.0) {
                return false;
        }

        opts->focus_m_set = true;
        opts->focus_m_inf = false;
        opts->focus_m = parsed_focus_m;
        return true;
}

bool parse_ae_diag_interval(std::string_view val, libcamera_options *opts)
{
        opts->ae_diag_set = true;
        if (val.empty()) {
                opts->ae_diag_interval = 30;
                return true;
        }

        std::string interval_str(val);
        char *end = nullptr;
        errno = 0;
        const unsigned long interval =
                strtoul(interval_str.c_str(), &end, 10);
        if (errno != 0 || end == interval_str.c_str() || *end != '\0' ||
                        interval == 0 ||
                        interval > std::numeric_limits<unsigned int>::max()) {
                return false;
        }

        opts->ae_diag_interval = static_cast<unsigned int>(interval);
        return true;
}

bool parse_float_option(std::string_view val, const char *name, float *target,
                bool *set)
{
        std::string value_str(val);
        char *end = nullptr;
        errno = 0;
        const float value = strtof(value_str.c_str(), &end);
        if (errno != 0 || end == value_str.c_str() || *end != '\0' ||
                        !std::isfinite(value)) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "invalid %s value: expected finite "
                                "float\n",
                                name);
                return false;
        }
        *target = value;
        *set = true;
        return true;
}

void set_af_range_control(libcamera::ControlList *controls,
                libcamera_options::AfRange range)
{
        switch (range) {
        case libcamera_options::AfRange::Normal:
                controls->set(libcamera::controls::AfRange,
                                libcamera::controls::AfRangeNormal);
                break;
        case libcamera_options::AfRange::Macro:
                controls->set(libcamera::controls::AfRange,
                                libcamera::controls::AfRangeMacro);
                break;
        case libcamera_options::AfRange::Full:
                controls->set(libcamera::controls::AfRange,
                                libcamera::controls::AfRangeFull);
                break;
        }
}

libcamera::Rectangle map_af_area_to_scaler_crop(
                const af_area_preset &preset,
                const libcamera::Rectangle &scaler_crop)
{
        const int min_x = scaler_crop.x;
        const int min_y = scaler_crop.y;
        const int max_x = scaler_crop.x + static_cast<int>(scaler_crop.width);
        const int max_y = scaler_crop.y + static_cast<int>(scaler_crop.height);
        int x = scaler_crop.x +
                static_cast<int>(std::llround(preset.x * scaler_crop.width));
        int y = scaler_crop.y +
                static_cast<int>(std::llround(preset.y * scaler_crop.height));
        unsigned int width = static_cast<unsigned int>(
                        std::max<int64_t>(1,
                                std::llround(preset.width *
                                        scaler_crop.width)));
        unsigned int height = static_cast<unsigned int>(
                        std::max<int64_t>(1,
                                std::llround(preset.height *
                                        scaler_crop.height)));
        x = std::clamp(x, min_x, max_x - 1);
        y = std::clamp(y, min_y, max_y - 1);
        width = std::min<unsigned int>(width,
                        static_cast<unsigned int>(max_x - x));
        height = std::min<unsigned int>(height,
                        static_cast<unsigned int>(max_y - y));

        return { x, y, width, height };
}

bool apply_focus_controls(vidcap_libcamera_state *s,
                const libcamera_options &opts,
                libcamera::ControlList *controls,
                bool log_requested)
{
        if (!opts.focus_set) {
                return true;
        }

        if (opts.focus_manual) {
                if (s->camera->controls().count(
                                libcamera::controls::AfMode.id()) == 0 ||
                                s->camera->controls().count(
                                        libcamera::controls::LensPosition.id()) == 0) {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "manual focus requested but "
                                        "this camera does not expose AfMode/"
                                        "LensPosition controls\n");
                        return false;
                }
                const float lens_position = opts.focus_m_inf ? 0.0f :
                        static_cast<float>(1.0 / opts.focus_m);
                controls->set(libcamera::controls::AfMode,
                                libcamera::controls::AfModeManual);
                controls->set(libcamera::controls::LensPosition,
                                lens_position);
                if (!log_requested) {
                        return true;
                } else if (opts.focus_m_inf) {
                        log_msg(LOG_LEVEL_INFO,
                                        MOD_NAME "focus requested: mode=manual "
                                        "distance=inf lens_position=%.3fD\n",
                                        lens_position);
                } else {
                        log_msg(LOG_LEVEL_INFO,
                                        MOD_NAME "focus requested: mode=manual "
                                        "distance=%.3fm lens_position=%.3fD\n",
                                        opts.focus_m, lens_position);
                }
                return true;
        }

        if (s->camera->controls().count(libcamera::controls::AfMode.id()) == 0) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "focus=afc requested but AfMode "
                                "control is not available\n");
                return false;
        }
        controls->set(libcamera::controls::AfMode,
                        libcamera::controls::AfModeContinuous);

        if (opts.af_range_set) {
                if (s->camera->controls().count(
                                libcamera::controls::AfRange.id()) == 0) {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "af_range requested but "
                                        "AfRange control is not available\n");
                        return false;
                }
                set_af_range_control(controls, opts.af_range);
        }

        if (opts.af_speed_set) {
                if (s->camera->controls().count(
                                libcamera::controls::AfSpeed.id()) == 0) {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "af_speed requested but "
                                        "AfSpeed control is not available\n");
                        return false;
                }
                controls->set(libcamera::controls::AfSpeed,
                                opts.af_speed ==
                                        libcamera_options::AfSpeed::Fast ?
                                        libcamera::controls::AfSpeedFast :
                                        libcamera::controls::AfSpeedNormal);
        }

        if (!opts.af_area_set) {
                if (log_requested) {
                        log_msg(LOG_LEVEL_INFO,
                                        MOD_NAME "focus requested: mode=continuous "
                                        "speed=%s range=%s area=default\n",
                                        opts.af_speed_set ?
                                                af_speed_to_string(opts.af_speed) :
                                                "default",
                                        opts.af_range_set ?
                                                af_range_to_string(opts.af_range) :
                                                "default");
                }
                return true;
        }

        if (s->camera->controls().count(
                        libcamera::controls::AfMetering.id()) == 0 ||
                        s->camera->controls().count(
                                libcamera::controls::AfWindows.id()) == 0) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "af_area requested but AfMetering/"
                                "AfWindows controls are not available\n");
                return false;
        }

        const af_area_preset *preset =
                find_af_area_preset(opts.af_area_name);
        if (preset == nullptr) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "invalid af_area value: expected "
                                "full, mid, or center\n");
                return false;
        }

        const std::optional<libcamera::Rectangle> scaler_crop =
                s->camera->properties().get(
                                libcamera::properties::ScalerCropMaximum);
        if (!scaler_crop || scaler_crop->isNull()) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "af_area requested but "
                                "ScalerCropMaximum is not available; cannot "
                                "map autofocus window safely\n");
                return false;
        }

        const libcamera::Rectangle pixel_area =
                map_af_area_to_scaler_crop(*preset, *scaler_crop);
        controls->set(libcamera::controls::AfMetering,
                        libcamera::controls::AfMeteringWindows);
        controls->set(libcamera::controls::AfWindows, { pixel_area });

        if (log_requested) {
                log_msg(LOG_LEVEL_INFO,
                                MOD_NAME "focus requested: mode=continuous "
                                "speed=%s range=%s area=%s "
                                "norm=%.4g,%.4g,%.4g,%.4g pixel_area=%s\n",
                                opts.af_speed_set ?
                                        af_speed_to_string(opts.af_speed) :
                                        "default",
                                opts.af_range_set ?
                                        af_range_to_string(opts.af_range) :
                                        "default",
                                preset->name, preset->x, preset->y,
                                preset->width, preset->height,
                                pixel_area.toString().c_str());
        }
        return true;
}

bool get_float_control_range(vidcap_libcamera_state *s,
                const libcamera::Control<float> &control,
                float *min,
                float *max,
                float *def)
{
        const auto info = s->camera->controls().find(control.id());
        if (info == s->camera->controls().end()) {
                return false;
        }

        if (!info->second.min().isNone()) {
                *min = info->second.min().get<float>();
        } else {
                *min = -std::numeric_limits<float>::infinity();
        }
        if (!info->second.max().isNone()) {
                *max = info->second.max().get<float>();
        } else {
                *max = std::numeric_limits<float>::infinity();
        }
        if (!info->second.def().isNone()) {
                *def = info->second.def().get<float>();
        } else {
                *def = std::numeric_limits<float>::quiet_NaN();
        }
        return true;
}

bool set_float_control_checked(vidcap_libcamera_state *s,
                libcamera::ControlList *controls,
                const libcamera::Control<float> &control,
                const char *public_name,
                const char *control_name,
                float value,
                bool explicit_request,
                const char *source)
{
        float min = 0.0f;
        float max = 0.0f;
        float def = 0.0f;
        if (!get_float_control_range(s, control, &min, &max, &def)) {
                if (explicit_request) {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "%s requested but %s control "
                                        "is not available\n",
                                        public_name, control_name);
                        return false;
                }
                log_msg(LOG_LEVEL_WARNING,
                                MOD_NAME "%s preset wants %s=%.3f but %s "
                                "control is not available; skipping\n",
                                source, public_name, value, control_name);
                return true;
        }

        if (value < min || value > max) {
                if (explicit_request) {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "%s=%.3f outside supported "
                                        "range %.3f..%.3f\n",
                                        public_name, value, min, max);
                        return false;
                }
                log_msg(LOG_LEVEL_WARNING,
                                MOD_NAME "%s preset wants %s=%.3f outside "
                                "supported range %.3f..%.3f; skipping\n",
                                source, public_name, value, min, max);
                return true;
        }

        controls->set(control, value);
        log_msg(LOG_LEVEL_INFO,
                        MOD_NAME "%s requested: %.3f applied: %.3f "
                        "source=%s range=%.3f..%.3f\n",
                        public_name, value, value,
                        explicit_request ? "user" : source, min, max);
        return true;
}

bool set_ir_contrast_preset(vidcap_libcamera_state *s,
                libcamera::ControlList *controls)
{
        float min = 0.0f;
        float max = 0.0f;
        float def = 0.0f;
        if (!get_float_control_range(s, libcamera::controls::Contrast,
                        &min, &max, &def)) {
                log_msg(LOG_LEVEL_WARNING,
                                MOD_NAME "night=ir preset wants contrast=1.200 "
                                "but Contrast control is not available; "
                                "skipping\n");
                return true;
        }

        float contrast = 1.2f;
        const char *source = "night=ir";
        if (contrast < min || contrast > max) {
                if (std::isfinite(def) && def >= min && def <= max) {
                        log_msg(LOG_LEVEL_WARNING,
                                        MOD_NAME "night=ir preset contrast=1.200 "
                                        "outside supported range %.3f..%.3f; "
                                        "using control default %.3f\n",
                                        min, max, def);
                        contrast = def;
                        source = "night=ir-default";
                } else {
                        log_msg(LOG_LEVEL_WARNING,
                                        MOD_NAME "night=ir preset contrast=1.200 "
                                        "outside supported range %.3f..%.3f "
                                        "and no usable default is available; "
                                        "skipping\n",
                                        min, max);
                        return true;
                }
        }

        controls->set(libcamera::controls::Contrast, contrast);
        log_msg(LOG_LEVEL_INFO,
                        MOD_NAME "contrast requested: %.3f applied: %.3f "
                        "source=%s range=%.3f..%.3f\n",
                        contrast, contrast, source, min, max);
        return true;
}

bool apply_exposure_controls(vidcap_libcamera_state *s,
                const libcamera_options &opts,
                libcamera::ControlList *controls)
{
        bool ae_enable_requested = opts.ae_set;
        bool ae_enable = opts.ae_enable;

        if (opts.night_set &&
                        (opts.night_mode ==
                                libcamera_options::NightMode::Ir ||
                         opts.night_mode ==
                                libcamera_options::NightMode::Lowlight) &&
                        !opts.ae_set) {
                ae_enable_requested = true;
                ae_enable = true;
        }

        if (ae_enable_requested) {
                if (s->camera->controls().count(
                                libcamera::controls::AeEnable.id()) == 0) {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "ae requested but AeEnable "
                                        "control is not available\n");
                        return false;
                }
                controls->set(libcamera::controls::AeEnable, ae_enable);
                log_msg(LOG_LEVEL_INFO,
                                MOD_NAME "AE requested: %s applied: %s\n",
                                ae_enable ? "on" : "off",
                                ae_enable ? "on" : "off");
        }

        if (opts.metering_set) {
                if (s->camera->controls().count(
                                libcamera::controls::AeMeteringMode.id()) ==
                                0) {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "metering requested but "
                                        "AeMeteringMode control is not "
                                        "available\n");
                        return false;
                }
                controls->set(libcamera::controls::AeMeteringMode,
                                opts.metering_mode);
                log_msg(LOG_LEVEL_INFO,
                                MOD_NAME "metering requested: %s applied: %s\n",
                                opts.metering_name.c_str(),
                                metering_mode_to_string(opts.metering_mode));
        }

        if (opts.night_set) {
                if (s->camera->controls().count(
                                libcamera::controls::AeExposureMode.id()) ==
                                0) {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "night requested but "
                                        "AeExposureMode control is not "
                                        "available\n");
                        return false;
                }

                int32_t exposure_mode = libcamera::controls::ExposureNormal;
                if (opts.night_mode == libcamera_options::NightMode::Ir ||
                                opts.night_mode ==
                                        libcamera_options::NightMode::Lowlight) {
                        exposure_mode = libcamera::controls::ExposureLong;
                }

                controls->set(libcamera::controls::AeExposureMode,
                                exposure_mode);
                log_msg(LOG_LEVEL_INFO,
                                MOD_NAME "night requested: %s applied: %s "
                                "exposure=%s\n",
                                opts.night_name.c_str(),
                                night_mode_to_string(opts.night_mode),
                                exposure_mode ==
                                        libcamera::controls::ExposureLong ?
                                        "long" : "normal");
                if (opts.night_mode == libcamera_options::NightMode::Ir) {
                        log_msg(LOG_LEVEL_INFO,
                                        MOD_NAME "night=ir is an image preset "
                                        "for NoIR/IR illumination; it does "
                                        "not switch an IR-cut filter\n");
                }
        }

        if (opts.night_set &&
                        opts.night_mode ==
                                libcamera_options::NightMode::Lowlight &&
                        !opts.ev_set) {
                log_msg(LOG_LEVEL_INFO,
                                MOD_NAME "night=lowlight preset controls: "
                                "preset ev=1.000\n");
                if (!set_float_control_checked(s, controls,
                                libcamera::controls::ExposureValue,
                                "ev", "ExposureValue", 1.0f, false,
                                "night=lowlight")) {
                        return false;
                }
        }

        if (opts.night_set &&
                        opts.night_mode == libcamera_options::NightMode::Ir) {
                log_msg(LOG_LEVEL_INFO,
                                MOD_NAME "night=ir preset controls: preset "
                                "ev=%.1f saturation=%.1f contrast=%.1f\n",
                                opts.ev_set ? opts.ev : 0.7f,
                                opts.saturation_set ? opts.saturation : 0.0f,
                                opts.contrast_set ? opts.contrast : 1.2f);
                if (!opts.ev_set &&
                                !set_float_control_checked(s, controls,
                                        libcamera::controls::ExposureValue,
                                        "ev", "ExposureValue", 0.7f, false,
                                        "night=ir")) {
                        return false;
                }
                if (!opts.saturation_set &&
                                !set_float_control_checked(s, controls,
                                        libcamera::controls::Saturation,
                                        "saturation", "Saturation", 0.0f,
                                        false, "night=ir")) {
                        return false;
                }
                if (!opts.contrast_set &&
                                !set_ir_contrast_preset(s, controls)) {
                        return false;
                }
        }

        if (opts.ev_set &&
                        !set_float_control_checked(s, controls,
                                libcamera::controls::ExposureValue,
                                "ev", "ExposureValue", opts.ev, true,
                                "user")) {
                return false;
        }
        if (opts.brightness_set &&
                        !set_float_control_checked(s, controls,
                                libcamera::controls::Brightness,
                                "brightness", "Brightness", opts.brightness,
                                true, "user")) {
                return false;
        }
        if (opts.saturation_set &&
                        !set_float_control_checked(s, controls,
                                libcamera::controls::Saturation,
                                "saturation", "Saturation", opts.saturation,
                                true, "user")) {
                return false;
        }
        if (opts.contrast_set &&
                        !set_float_control_checked(s, controls,
                                libcamera::controls::Contrast,
                                "contrast", "Contrast", opts.contrast, true,
                                "user")) {
                return false;
        }

        return true;
}

bool start_camera(vidcap_libcamera_state *s, const libcamera_options &opts)
{
        libcamera::ControlList controls(s->camera->controls());
        libcamera::ControlList *start_controls = nullptr;
        if (opts.fps_set) {
                const int64_t frame_duration_us =
                        static_cast<int64_t>(std::llround(1000000.0 / opts.fps));
                if (s->camera->controls().count(
                                libcamera::controls::FrameDurationLimits.id()) >
                                0) {
                        controls.set(libcamera::controls::FrameDurationLimits,
                                        { frame_duration_us,
                                                frame_duration_us });
                        start_controls = &controls;
                        log_msg(LOG_LEVEL_INFO,
                                        MOD_NAME "applying fps=%.3f via "
                                        "FrameDurationLimits=%" PRId64 " us\n",
                                        opts.fps, frame_duration_us);
                } else {
                        log_msg(LOG_LEVEL_INFO,
                                        MOD_NAME "requested fps=%.3f; "
                                        "FrameDurationLimits control is not "
                                        "available\n",
                                        opts.fps);
                }
        }

        if (opts.ae_set || opts.metering_set || opts.night_set ||
                        opts.ev_set || opts.brightness_set ||
                        opts.saturation_set || opts.contrast_set) {
                if (!apply_exposure_controls(s, opts, &controls)) {
                        return false;
                }
                start_controls = &controls;
        }

        if (opts.focus_set) {
                if (!apply_focus_controls(s, opts, &controls, true)) {
                        return false;
                }
                start_controls = &controls;
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
                if (opts.focus_set &&
                                !apply_focus_controls(s, opts,
                                        &request->controls(), false)) {
                        return false;
                }
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

bool request_focus_controls(vidcap_libcamera_state *s,
                const libcamera_options &opts,
                libcamera::Request *request)
{
        if (!opts.focus_set) {
                return true;
        }
        return apply_focus_controls(s, opts, &request->controls(), false);
}

void log_focus_metadata_if_needed(vidcap_libcamera_state *s,
                const libcamera::Request *request)
{
        const libcamera::ControlList &metadata = request->metadata();
        const std::optional<int32_t> af_mode =
                metadata.get(libcamera::controls::AfMode);
        const std::optional<float> lens_position =
                metadata.get(libcamera::controls::LensPosition);
        const std::optional<int32_t> af_state =
                metadata.get(libcamera::controls::AfState);
        const std::optional<int32_t> focus_fom =
                metadata.get(libcamera::controls::FocusFoM);

        if (!af_mode && !lens_position && !af_state && !focus_fom) {
                return;
        }

        const auto now = std::chrono::steady_clock::now();
        const bool key_state_changed =
                af_mode != s->last_metadata_af_mode ||
                lens_position != s->last_metadata_lens_position ||
                af_state != s->last_metadata_af_state;
        const bool elapsed =
                s->last_focus_metadata_log.time_since_epoch().count() == 0 ||
                now - s->last_focus_metadata_log >= std::chrono::seconds(1);
        if (!key_state_changed && !elapsed) {
                return;
        }

        char mode_buf[32] = "n/a";
        char lens_buf[32] = "n/a";
        char state_buf[32] = "n/a";
        char fom_buf[32] = "n/a";
        if (af_mode) {
                snprintf(mode_buf, sizeof mode_buf, "%s",
                                af_mode_to_string(*af_mode));
        }
        if (lens_position) {
                snprintf(lens_buf, sizeof lens_buf, "%.3fD", *lens_position);
        }
        if (af_state) {
                snprintf(state_buf, sizeof state_buf, "%s",
                                af_state_to_string(*af_state));
        }
        if (focus_fom) {
                snprintf(fom_buf, sizeof fom_buf, "%" PRId32, *focus_fom);
        }

        log_msg(LOG_LEVEL_INFO,
                        MOD_NAME "focus metadata: mode=%s "
                        "lens_position=%s af_state=%s focus_fom=%s\n",
                        mode_buf, lens_buf, state_buf, fom_buf);

        s->last_focus_metadata_log = now;
        s->last_metadata_af_mode = af_mode;
        s->last_metadata_lens_position = lens_position;
        s->last_metadata_af_state = af_state;
        s->last_metadata_focus_fom = focus_fom;
}

std::string format_int32_metadata(const std::optional<int32_t> &value)
{
        if (!value) {
                return "n/a";
        }
        char buf[32];
        snprintf(buf, sizeof buf, "%" PRId32, *value);
        return buf;
}

std::string format_int64_metadata(const std::optional<int64_t> &value)
{
        if (!value) {
                return "n/a";
        }
        char buf[32];
        snprintf(buf, sizeof buf, "%" PRId64, *value);
        return buf;
}

std::string format_float_metadata(const std::optional<float> &value,
                const char *fmt)
{
        if (!value) {
                return "n/a";
        }
        char buf[32];
        snprintf(buf, sizeof buf, fmt, *value);
        return buf;
}

std::string format_ae_state_metadata(const std::optional<int32_t> &value)
{
        if (!value) {
                return "n/a";
        }
        char buf[64];
        snprintf(buf, sizeof buf, "%" PRId32 "(%s)", *value,
                        ae_state_to_string(*value));
        return buf;
}

std::string format_af_state_metadata(const std::optional<int32_t> &value)
{
        if (!value) {
                return "n/a";
        }
        char buf[64];
        snprintf(buf, sizeof buf, "%" PRId32 "(%s)", *value,
                        af_state_to_string(*value));
        return buf;
}

std::string requested_focus_mode(const libcamera_options &opts)
{
        if (opts.focus_manual) {
                return "manual";
        }
        if (opts.focus_afc) {
                return "afc";
        }
        return "unset";
}

std::string requested_focus_m(const libcamera_options &opts)
{
        if (!opts.focus_manual || !opts.focus_m_set) {
                return "n/a";
        }
        if (opts.focus_m_inf) {
                return "inf";
        }
        char buf[32];
        snprintf(buf, sizeof buf, "%.6g", opts.focus_m);
        return buf;
}

void log_ae_diag_if_needed(vidcap_libcamera_state *s,
                const libcamera::Request *request,
                uint64_t sequence)
{
        const libcamera_options &opts = s->opts;
        if (!opts.ae_diag_set) {
                return;
        }

        const libcamera::ControlList &metadata = request->metadata();
        const std::optional<int32_t> exposure_time =
                metadata.get(libcamera::controls::ExposureTime);
        const std::optional<float> analogue_gain =
                metadata.get(libcamera::controls::AnalogueGain);
        const std::optional<float> digital_gain =
                metadata.get(libcamera::controls::DigitalGain);
        const std::optional<float> lux =
                metadata.get(libcamera::controls::Lux);
        const std::optional<int32_t> ae_state =
                metadata.get(libcamera::controls::AeState);
        const std::optional<int64_t> frame_duration =
                metadata.get(libcamera::controls::FrameDuration);
        const std::optional<float> lens_position =
                metadata.get(libcamera::controls::LensPosition);
        const std::optional<int32_t> af_state =
                metadata.get(libcamera::controls::AfState);
        const std::optional<int32_t> focus_fom =
                metadata.get(libcamera::controls::FocusFoM);

        if (exposure_time && analogue_gain && digital_gain &&
                        frame_duration &&
                        *exposure_time >= 0.95 * *frame_duration &&
                        *analogue_gain >= 15.5f &&
                        *digital_gain <= 1.05f) {
                const auto now = std::chrono::steady_clock::now();
                if (s->last_ae_ceiling_log.time_since_epoch().count() == 0 ||
                                now - s->last_ae_ceiling_log >=
                                        std::chrono::seconds(5)) {
                        log_msg(LOG_LEVEL_WARNING,
                                        "[libcamera ae_diag] exposure/gain "
                                        "ceiling reached: exposure_us=%" PRId32
                                        " frame_duration_us=%" PRId64
                                        " analogue_gain=%.3f "
                                        "digital_gain=%.3f; night preset "
                                        "cannot increase sensor exposure at "
                                        "current fps\n",
                                        *exposure_time, *frame_duration,
                                        *analogue_gain, *digital_gain);
                        s->last_ae_ceiling_log = now;
                }
        }

        const unsigned int frame_index = ++s->ae_diag_completed_frames;
        if (frame_index > 10 &&
                        frame_index % opts.ae_diag_interval != 0) {
                return;
        }

        const char *night = opts.night_set ? opts.night_name.c_str() : "unset";
        const char *ae = opts.ae_set ? (opts.ae_enable ? "on" : "off") :
                "unset";
        const char *metering = opts.metering_set ?
                opts.metering_name.c_str() : "unset";
        const std::string focus = requested_focus_mode(opts);
        const std::string focus_m = requested_focus_m(opts);

        log_msg(LOG_LEVEL_INFO,
                        "[libcamera ae_diag] seq=%" PRIu64
                        " night=%s ae=%s metering=%s focus=%s focus_m=%s "
                        "exposure_us=%s analogue_gain=%s digital_gain=%s "
                        "lux=%s ae_state=%s frame_duration_us=%s "
                        "lens_position=%s af_state=%s focus_fom=%s\n",
                        sequence, night, ae, metering, focus.c_str(),
                        focus_m.c_str(),
                        format_int32_metadata(exposure_time).c_str(),
                        format_float_metadata(analogue_gain, "%.3f").c_str(),
                        format_float_metadata(digital_gain, "%.3f").c_str(),
                        format_float_metadata(lux, "%.3f").c_str(),
                        format_ae_state_metadata(ae_state).c_str(),
                        format_int64_metadata(frame_duration).c_str(),
                        format_float_metadata(lens_position, "%.3f").c_str(),
                        format_af_state_metadata(af_state).c_str(),
                        format_int32_metadata(focus_fom).c_str());
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
                printf("\t-t libcamera[:d=<index>|camera=<index>][:hdr|hdr=on|hdr=off][:night=off|ir|lowlight][:metering=centre|spot|matrix][:ae=on|off][:ev=<float>][:brightness=<float>][:saturation=<float>][:contrast=<float>][:ae_diag[=N]][:sensor=WxH][:size=WxH][:fps=N][:format=YUV420|I420|UYVY|YUYV][:focus=manual:focus_m=<metres|inf>|focus=afc[:af_speed=normal|fast][:af_range=normal|macro|full][:af_area=full|mid|center]][:list|caps|test|help|fullhelp]\n");
        printf("\n");
}

void show_fullhelp()
{
        print_usage();
        printf("Without overrides, libcamera's default VideoRecording configuration is used.\n");
        printf("d=<index>, camera=<index> select a libcamera device; both names are aliases.\n");
        printf("size=WxH selects the output stream size.\n");
        printf("sensor=WxH optionally requests the libcamera sensor output size before ISP scaling.\n");
        printf("fps=N requests frame rate through FrameDurationLimits when streaming starts.\n");
        printf("Known IMX708 sensor fps limits are checked for sensor=WxH and hdr requests.\n");
        printf("hdr or hdr=on enables experimental IMX708 sensor HDR.\n");
        printf("hdr=off explicitly disables IMX708 WDR/HDR. Without hdr, WDR/HDR state is not touched.\n");
        printf("IMX708 HDR uses internal sensor mode 2304x1296 and max_fps=30.\n");
        printf("night=off|ir|lowlight selects AE/image presets only; it does not touch HDR/WDR state.\n");
        printf("night=lowlight applies ev=1.0 by default unless ev= is explicitly provided.\n");
        printf("night=ir applies ev=0.7, saturation=0.0 and, when supported, contrast=1.2 by default unless explicitly overridden.\n");
        printf("night=ir is an image preset for NoIR/IR illumination and does not switch a hardware IR-cut filter.\n");
        printf("metering=centre|spot|matrix selects AeMeteringMode.\n");
        printf("ae=on|off enables or disables automatic exposure.\n");
        printf("ev=<float> sets ExposureValue when supported.\n");
        printf("brightness=<float> sets Brightness when supported.\n");
        printf("saturation=<float> sets Saturation when supported.\n");
        printf("contrast=<float> sets Contrast when supported.\n");
        printf("ae_diag[=N] logs AE/AF metadata every N frames, plus first 10 frames; default N is 30.\n");
        printf("focus=manual:focus_m=<metres|inf> enables manual focus; focus_m=inf maps to infinity focus.\n");
        printf("focus=afc enables continuous autofocus.\n");
        printf("af_speed=normal or af_speed=fast selects autofocus speed when focus=afc is used.\n");
        printf("af_range=normal, af_range=macro or af_range=full selects autofocus range when focus=afc is used.\n");
        printf("af_area=full uses the whole image for autofocus metering when focus=afc is used.\n");
        printf("af_area=mid uses the central half of the image width and height.\n");
        printf("af_area=center uses the central third of the image width and height.\n");
        printf("Numeric autofocus windows are intentionally not part of the public API yet.\n");
        printf("format=YUV420 selects planar 4:2:0 and maps to UltraGrid I420. I420 is an alias.\n");
        printf("format=UYVY and format=YUYV select packed 8-bit 4:2:2 handoff if libcamera can negotiate them natively.\n");
        printf("format=YUV422 is recognized but rejected because this UltraGrid tree has no direct matching internal planar 8-bit 4:2:2 codec mapping.\n");
        printf("format=YUV444 is recognized but rejected because this UltraGrid tree has no direct matching internal planar 8-bit 4:4:4 codec mapping.\n");
        printf("list prints only available cameras.\n");
        printf("caps prints detailed libcamera StreamFormats and may be long.\n");
        printf("test runs a short measured FPS support test for typical values 24..120.\n");
        printf("test respects d/camera, sensor, size and format; fps=N limits test to that single FPS value.\n");
        printf("test is diagnostic only and does not use the UltraGrid frame handoff path.\n");
        printf("testverbose keeps libcamera diagnostic logs enabled while running the FPS test.\n");
        printf("help prints a compact device and output-size overview.\n");
        printf("fullhelp prints this detailed parameter description.\n");
        printf("mode=N is intentionally not part of the public libcamera API because libcamera/RPi sensor modes are a different layer than VideoRecording output.\n");
        printf("Supported output formats for native frame handoff: YUV420/I420, UYVY, YUYV.\n");
        printf("Note: size=1280x720 may select a cropped sensor mode unless sensor= is explicitly used.\n");
                printf("Examples:\n");
                printf("\tFull-FOV-ish 720p56: -t libcamera:d=0:sensor=2304x1296:size=1280x720:fps=56:format=YUV420\n");
                printf("\tCropped 720p60+:     -t libcamera:d=0:sensor=1536x864:size=1280x720:fps=60:format=YUV420\n");
                printf("\tIMX708 HDR 720p30:   -t libcamera:d=0:hdr:size=1280x720:fps=30:format=YUV420\n");
                printf("\tLow light preset:    -t libcamera:d=0:night=lowlight:size=1280x720:fps=30:format=YUV420\n");
                printf("\tIR exposure preset:  -t libcamera:d=0:night=ir:metering=centre:size=1280x720:fps=30:format=YUV420\n");
                printf("\tAE off:              -t libcamera:d=0:ae=off:size=1280x720:fps=30:format=YUV420\n");
                printf("\tAE diagnostics:      -t libcamera:d=0:size=1280x720:fps=30:format=YUV420:night=lowlight:metering=spot:ae_diag=30\n");
                printf("\tManual image ctrl:   -t libcamera:d=0:night=off:ev=1.0:brightness=0.1:contrast=1.2:saturation=0.5:size=1280x720:fps=30:format=YUV420\n");
                printf("\tManual focus 2 m:    -t libcamera:d=0:focus=manual:focus_m=2.0:size=1280x720:fps=30:format=YUV420\n");
                printf("\tManual focus inf:    -t libcamera:d=0:focus=manual:focus_m=inf:size=1280x720:fps=30:format=YUV420\n");
                printf("\tContinuous AF:       -t libcamera:d=0:focus=afc:af_speed=fast:af_range=full:af_area=mid:size=1280x720:fps=30:format=YUV420\n");
        printf("Status: progressive YUV capture handoff is implemented.\n");
}

void show_help_header()
{
        print_usage();
        printf("Examples:\n");
        printf("\t-t libcamera\n");
                printf("\t-t libcamera:d=0:size=1280x720:fps=50:format=YUV420\n");
                printf("\t-t libcamera:d=0:hdr:size=1280x720:fps=30:format=YUV420\n");
                printf("\t-t libcamera:d=0:night=lowlight:size=1280x720:fps=30:format=YUV420\n");
                printf("\t-t libcamera:d=0:night=ir:metering=centre:size=1280x720:fps=30:format=YUV420\n");
                printf("\t-t libcamera:d=0:ae=off:size=1280x720:fps=30:format=YUV420\n");
                printf("\t-t libcamera:d=0:size=1280x720:fps=30:format=YUV420:night=lowlight:metering=spot:ae_diag=30\n");
                printf("\t-t libcamera:d=0:night=off:ev=1.0:brightness=0.1:contrast=1.2:saturation=0.5:size=1280x720:fps=30:format=YUV420\n");
                printf("\t-t libcamera:d=0:focus=manual:focus_m=2.0:size=1280x720:fps=30:format=YUV420\n");
                printf("\t-t libcamera:d=0:focus=afc:af_speed=fast:af_range=full:af_area=mid:size=1280x720:fps=30:format=YUV420\n");
        printf("\t-t libcamera:d=0:sensor=2304x1296:size=1280x720:fps=56:format=YUV420\n");
        printf("\t-t libcamera:d=0:sensor=1536x864:size=1280x720:fps=60:format=YUV420\n");
        printf("\t-t libcamera:d=0:size=1280x720:fps=50:format=UYVY\n");
        printf("\t-t libcamera:list\n");
        printf("\t-t libcamera:caps\n");
        printf("\t-t libcamera:d=0:size=1280x720:format=YUV420:test\n");
        printf("\n");
        printf("Default uses libcamera's VideoRecording configuration.\n");
        printf("Supported output formats: YUV420/I420; UYVY/YUYV if negotiated natively.\n");
        printf("Note: size=1280x720 may select a cropped sensor mode unless sensor= is explicitly used.\n");
        printf("night=ir is an image preset for NoIR/IR illumination; it does not switch hardware IR-cut.\n");
        printf("ev=, brightness=, saturation= and contrast= apply libcamera image controls when supported.\n");
        printf("ae_diag[=N] logs AE/AF metadata every N frames, plus first 10 frames.\n");
        printf("YUV422/YUV444 are recognized but rejected: no direct internal planar 8-bit codec mapping in this UltraGrid tree.\n");
        printf("test: measured FPS support test.\n");
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
                } else if (key == "test" && val.empty()) {
                        opts->action = libcamera_options::Action::Test;
                } else if (key == "testverbose" && val.empty()) {
                        opts->action = libcamera_options::Action::Test;
                        opts->test_verbose = true;
                } else if (key == "camera" || key == "d") {
                        if (!parse_num(val, opts->camera_index)) {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "failed to parse camera index\n");
                                return VIDCAP_INIT_FAIL;
                        }
                } else if (key == "size") {
                        if (!parse_size(val, &opts->size)) {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "failed to parse size\n");
                                return VIDCAP_INIT_FAIL;
                        }
                        opts->size_set = true;
                } else if (key == "sensor") {
                        libcamera::Size sensor_size = {};
                        if (!parse_size(val, &sensor_size) ||
                                        sensor_size.width % 2 != 0 ||
                                        sensor_size.height % 2 != 0) {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "failed to parse "
                                                "sensor size; expected "
                                                "sensor=WxH with positive "
                                                "even dimensions\n");
                                return VIDCAP_INIT_FAIL;
                        }
                        opts->sensor_width = sensor_size.width;
                        opts->sensor_height = sensor_size.height;
                        opts->sensor_set = true;
                        } else if (key == "hdr") {
                                if (val.empty() || (val.size() == strlen("on") &&
                                                strncasecmp(val.data(), "on",
                                                        val.size()) == 0)) {
                                        opts->hdr_set = true;
                                        opts->hdr_mode = libcamera_options::HdrMode::Sensor;
                                        opts->hdr_name = "on";
                                } else if (val.size() == strlen("off") &&
                                                strncasecmp(val.data(), "off",
                                                        val.size()) == 0) {
                                        opts->hdr_set = true;
                                        opts->hdr_mode = libcamera_options::HdrMode::Off;
                                        opts->hdr_name = "off";
                                } else if (val.size() == strlen("sensor") &&
                                                strncasecmp(val.data(), "sensor",
                                                        val.size()) == 0) {
                                        opts->hdr_set = true;
                                        opts->hdr_mode = libcamera_options::HdrMode::Sensor;
                                        opts->hdr_name = "sensor";
                                } else if (val.size() == strlen("single-exp") &&
                                                strncasecmp(val.data(), "single-exp",
                                                        val.size()) == 0) {
                                        log_msg(LOG_LEVEL_ERROR,
                                                        MOD_NAME "hdr=single-exp is not "
                                                        "implemented in UltraGrid "
                                                        "libcamera module\n");
                                        return VIDCAP_INIT_FAIL;
                                } else if (val.size() == strlen("auto") &&
                                                strncasecmp(val.data(), "auto",
                                                        val.size()) == 0) {
                                        log_msg(LOG_LEVEL_ERROR,
                                                        MOD_NAME "hdr=auto is deprecated "
                                                        "and ambiguous; use bare hdr or "
                                                        "hdr=on to enable IMX708 sensor "
                                                        "HDR, or hdr=off to disable it\n");
                                        return VIDCAP_INIT_FAIL;
                                } else {
                                        log_msg(LOG_LEVEL_ERROR,
                                                        MOD_NAME "unsupported hdr=%.*s; "
                                                        "valid public values are hdr, "
                                                        "hdr=on, hdr=off\n",
                                                        static_cast<int>(val.size()),
                                                        val.data());
                                        return VIDCAP_INIT_FAIL;
                                }
                } else if (key == "mode") {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "mode is not supported by "
                                        "libcamera module; use "
                                        "size/fps/format\n");
                        return VIDCAP_INIT_FAIL;
                } else if (key == "fps") {
                        if (!parse_fps(val, &opts->fps)) {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "failed to parse fps\n");
                                return VIDCAP_INIT_FAIL;
                        }
                        opts->fps_set = true;
                } else if (key == "focus") {
                        opts->focus_set = true;
                        if (val.size() == strlen("afc") &&
                                        strncasecmp(val.data(), "afc",
                                                val.size()) == 0) {
                                opts->focus_afc = true;
                        } else if (val.size() == strlen("manual") &&
                                        strncasecmp(val.data(), "manual",
                                                val.size()) == 0) {
                                opts->focus_manual = true;
                        }
                } else if (key == "focus_m") {
                        if (!parse_focus_m(val, opts)) {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "invalid focus_m "
                                                "value: expected positive "
                                                "finite metres or inf\n");
                                return VIDCAP_INIT_FAIL;
                        }
                } else if (key == "af_speed") {
                        if (val.size() == strlen("normal") &&
                                        strncasecmp(val.data(), "normal",
                                        val.size()) == 0) {
                                opts->af_speed_set = true;
                                opts->af_speed = libcamera_options::AfSpeed::Normal;
                        } else if (val.size() == strlen("fast") &&
                                        strncasecmp(val.data(), "fast",
                                                val.size()) == 0) {
                                opts->af_speed_set = true;
                                opts->af_speed = libcamera_options::AfSpeed::Fast;
                        } else {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "invalid af_speed "
                                                "value: expected normal or "
                                                "fast\n");
                                return VIDCAP_INIT_FAIL;
                        }
                } else if (key == "af_range") {
                        if (val.size() == strlen("normal") &&
                                        strncasecmp(val.data(), "normal",
                                        val.size()) == 0) {
                                opts->af_range_set = true;
                                opts->af_range = libcamera_options::AfRange::Normal;
                        } else if (val.size() == strlen("macro") &&
                                        strncasecmp(val.data(), "macro",
                                                val.size()) == 0) {
                                opts->af_range_set = true;
                                opts->af_range = libcamera_options::AfRange::Macro;
                        } else if (val.size() == strlen("full") &&
                                        strncasecmp(val.data(), "full",
                                                val.size()) == 0) {
                                opts->af_range_set = true;
                                opts->af_range = libcamera_options::AfRange::Full;
                        } else {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "invalid af_range "
                                                "value: expected normal, "
                                                "macro, or full\n");
                                return VIDCAP_INIT_FAIL;
                        }
                } else if (key == "af_area") {
                        const af_area_preset *preset =
                                find_af_area_preset(val);
                        if (preset == nullptr) {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "invalid af_area "
                                                "value: expected full, mid, "
                                                "or center\n");
                                return VIDCAP_INIT_FAIL;
                        }
                        opts->af_area_set = true;
                        opts->af_area_name = preset->name;
                } else if (key == "ae") {
                        bool ae_enable = false;
                        if (!parse_on_off(val, &ae_enable)) {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "invalid ae value: "
                                                "expected on or off\n");
                                return VIDCAP_INIT_FAIL;
                        }
                        opts->ae_set = true;
                        opts->ae_enable = ae_enable;
                } else if (key == "metering") {
                        if (!parse_metering_mode(val, opts)) {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "invalid metering "
                                                "value: expected centre, "
                                                "spot, or matrix\n");
                                return VIDCAP_INIT_FAIL;
                        }
                } else if (key == "night") {
                        if (!parse_night_mode(val, opts)) {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "invalid night "
                                                "value: expected off, ir, "
                                                "or lowlight\n");
                                return VIDCAP_INIT_FAIL;
                        }
                } else if (key == "ae_diag") {
                        if (!parse_ae_diag_interval(val, opts)) {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "invalid ae_diag "
                                                "value: expected positive "
                                                "frame interval\n");
                                return VIDCAP_INIT_FAIL;
                        }
                } else if (key == "ev") {
                        if (!parse_float_option(val, "ev", &opts->ev,
                                        &opts->ev_set)) {
                                return VIDCAP_INIT_FAIL;
                        }
                } else if (key == "brightness") {
                        if (!parse_float_option(val, "brightness",
                                        &opts->brightness,
                                        &opts->brightness_set)) {
                                return VIDCAP_INIT_FAIL;
                        }
                } else if (key == "saturation") {
                        if (!parse_float_option(val, "saturation",
                                        &opts->saturation,
                                        &opts->saturation_set)) {
                                return VIDCAP_INIT_FAIL;
                        }
                } else if (key == "contrast") {
                        if (!parse_float_option(val, "contrast",
                                        &opts->contrast,
                                        &opts->contrast_set)) {
                                return VIDCAP_INIT_FAIL;
                        }
                } else if (key == "format") {
                        const libcamera_format_mapping *mapping =
                                find_supported_format(val);
                        if (mapping != nullptr) {
                                opts->format_set = true;
                                opts->format_name = mapping->name;
                                opts->pixel_format = mapping->pixel_format;
                                opts->codec = mapping->codec;
                        } else if (val.size() == strlen("YUV422") &&
                                        strncasecmp(val.data(), "YUV422",
                                                val.size()) == 0) {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "format=YUV422 is "
                                                "unsupported: this UltraGrid "
                                                "tree has no "
                                                "direct matching internal "
                                                "planar 8-bit 4:2:2 codec "
                                                "mapping\n");
                                return VIDCAP_INIT_FAIL;
                        } else if (val.size() == strlen("YUV444") &&
                                        strncasecmp(val.data(), "YUV444",
                                                val.size()) == 0) {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "format=YUV444 is "
                                                "unsupported: this UltraGrid "
                                                "tree has no direct matching "
                                                "internal planar 8-bit 4:4:4 "
                                                "codec mapping\n");
                                return VIDCAP_INIT_FAIL;
                        } else {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "unsupported format "
                                                "%.*s; supported values are "
                                                "YUV420, I420, UYVY, YUYV\n",
                                                static_cast<int>(val.size()),
                                                val.data());
                                return VIDCAP_INIT_FAIL;
                        }
                } else if (!key.empty()) {
                        log_msg(LOG_LEVEL_ERROR, MOD_NAME "unknown parameter: %.*s\n",
                                        static_cast<int>(key.size()), key.data());
                        return VIDCAP_INIT_FAIL;
                }
        }

        if (opts->focus_afc && opts->focus_manual) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "focus accepts one mode only: afc or "
                                "manual\n");
                return VIDCAP_INIT_FAIL;
        }
        if (opts->focus_manual && !opts->focus_m_set) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "focus=manual requires focus_m="
                                "<metres|inf>\n");
                return VIDCAP_INIT_FAIL;
        }
        if (opts->focus_m_set && !opts->focus_manual) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "focus_m is valid only with "
                                "focus=manual\n");
                return VIDCAP_INIT_FAIL;
        }
        if (opts->af_area_set && !opts->focus_afc) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "af_area is valid only with "
                                "focus=afc\n");
                return VIDCAP_INIT_FAIL;
        }
        if (opts->af_speed_set && !opts->focus_afc) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "af_speed is valid only with "
                                "focus=afc\n");
                return VIDCAP_INIT_FAIL;
        }
        if (opts->af_range_set && !opts->focus_afc) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "af_range is valid only with "
                                "focus=afc\n");
                return VIDCAP_INIT_FAIL;
        }
        if (opts->night_set &&
                        (opts->night_mode ==
                                libcamera_options::NightMode::Ir ||
                         opts->night_mode ==
                                libcamera_options::NightMode::Lowlight) &&
                        opts->ae_set && !opts->ae_enable) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "night=%s requires AE enabled; "
                                "cannot combine with ae=off\n",
                                opts->night_name.c_str());
                return VIDCAP_INIT_FAIL;
        }
        if (opts->focus_set && !opts->focus_afc) {
                if (opts->focus_manual) {
                        return VIDCAP_INIT_OK;
                }
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "unsupported focus value: expected "
                                "manual or afc\n");
                return VIDCAP_INIT_FAIL;
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

void print_known_sensor_caps(const std::shared_ptr<libcamera::Camera> &camera)
{
        const std::string model = camera_model_string(camera);
        printf("  Camera model: %s\n", model.c_str());
        if (!is_known_imx708_camera(model)) {
                printf("  Known sensor mode caps: unavailable for this camera\n");
                printf("  If sensor= is used, fps limits will be logged but not hard-failed.\n");
                return;
        }

                printf("  Known IMX708 SDR sensor mode caps:\n");
                for (const sensor_mode_cap &cap : imx708_sensor_caps) {
                        if (cap.hdr_mode == libcamera_options::HdrMode::Off) {
                                printf("    sensor=%s max_fps=%u (%s)\n",
                                                cap.sensor_size.toString().c_str(),
                                                cap.max_fps, cap.note);
                        }
                }
                printf("  Known IMX708 HDR sensor mode caps:\n");
                printf("    hdr / hdr=on: internal sensor=2304x1296 max_fps=30 "
                                "(experimental sensor HDR)\n");
                printf("    hdr=off explicitly disables IMX708 WDR/HDR; without hdr, "
                                "WDR/HDR state is not touched.\n");
                printf("  Note: size=1280x720 alone may select the cropped 1536x864 sensor mode.\n");
                printf("  Example full-FOV-ish 720p56: -t libcamera:d=0:sensor=2304x1296:size=1280x720:fps=56:format=YUV420\n");
                printf("  Example cropped 720p60+: -t libcamera:d=0:sensor=1536x864:size=1280x720:fps=60:format=YUV420\n");
                printf("  Example HDR 720p30: -t libcamera:d=0:hdr:size=1280x720:fps=30:format=YUV420\n");
        }

void print_focus_caps(const std::shared_ptr<libcamera::Camera> &camera)
{
        const bool has_af_mode = camera->controls().count(
                        libcamera::controls::AfMode.id()) > 0;
        const bool has_lens_position = camera->controls().count(
                        libcamera::controls::LensPosition.id()) > 0;
        const bool has_af_speed = camera->controls().count(
                        libcamera::controls::AfSpeed.id()) > 0;
        const bool has_af_range = camera->controls().count(
                        libcamera::controls::AfRange.id()) > 0;
        const bool has_af_metering = camera->controls().count(
                        libcamera::controls::AfMetering.id()) > 0;
        const bool has_af_windows = camera->controls().count(
                        libcamera::controls::AfWindows.id()) > 0;
        const std::optional<libcamera::Rectangle> scaler_crop =
                camera->properties().get(libcamera::properties::ScalerCropMaximum);

        printf("  Focus controls:\n");
        printf("    AfMode: %s\n", has_af_mode ? "yes" : "no");
        printf("    LensPosition: %s", has_lens_position ? "yes" : "no");
        if (has_lens_position) {
                const auto info = camera->controls().find(
                                libcamera::controls::LensPosition.id());
                if (info != camera->controls().end() &&
                                !info->second.min().isNone() &&
                                !info->second.max().isNone() &&
                                !info->second.def().isNone()) {
                        printf(" min=%.3fD max=%.3fD default=%.3fD",
                                        info->second.min().get<float>(),
                                        info->second.max().get<float>(),
                                        info->second.def().get<float>());
                } else {
                        printf(" min/max/default=n/a");
                }
        }
        printf("\n");
        printf("    AfSpeed: %s\n", has_af_speed ? "yes" : "no");
        printf("    AfRange: %s\n", has_af_range ? "yes" : "no");
        printf("    AfMetering: %s\n", has_af_metering ? "yes" : "no");
        printf("    AfWindows: %s\n", has_af_windows ? "yes" : "no");
        printf("    ScalerCropMaximum: %s", scaler_crop ? "yes" : "no");
        if (scaler_crop) {
                printf(" %s", scaler_crop->toString().c_str());
        }
        printf("\n");
}

void print_exposure_caps(const std::shared_ptr<libcamera::Camera> &camera)
{
        const bool has_ae_enable = camera->controls().count(
                        libcamera::controls::AeEnable.id()) > 0;
        const bool has_ae_metering = camera->controls().count(
                        libcamera::controls::AeMeteringMode.id()) > 0;
        const bool has_ae_exposure = camera->controls().count(
                        libcamera::controls::AeExposureMode.id()) > 0;
        const bool has_exposure_value = camera->controls().count(
                        libcamera::controls::ExposureValue.id()) > 0;
        const bool has_brightness = camera->controls().count(
                        libcamera::controls::Brightness.id()) > 0;
        const bool has_saturation = camera->controls().count(
                        libcamera::controls::Saturation.id()) > 0;
        const bool has_contrast = camera->controls().count(
                        libcamera::controls::Contrast.id()) > 0;

        printf("  Exposure controls:\n");
        printf("    AeEnable: %s\n", has_ae_enable ? "yes" : "no");
        printf("    AeMeteringMode: %s\n", has_ae_metering ? "yes" : "no");
        printf("    AeExposureMode: %s\n", has_ae_exposure ? "yes" : "no");
        printf("    ExposureValue: %s\n", has_exposure_value ? "yes" : "no");
        printf("    Brightness: %s\n", has_brightness ? "yes" : "no");
        printf("    Saturation: %s\n", has_saturation ? "yes" : "no");
        printf("    Contrast: %s\n", has_contrast ? "yes" : "no");
        printf("    night=ir: image preset for NoIR/IR illumination; "
                        "does not switch hardware IR-cut\n");
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
                        print_known_sensor_caps(camera);
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
                        printf("Device %s\n", camera->id().c_str());
                        print_known_sensor_caps(camera);
                        print_exposure_caps(camera);
                        print_focus_caps(camera);
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
        s->opts = opts;
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

        const libcamera::PixelFormat requested_pixfmt = opts.pixel_format;
        const libcamera::Size requested_size = opts.size;
        libcamera::Size effective_requested_size = requested_size;
        bool check_requested_format = opts.format_set;
        bool check_requested_size = opts.size_set;

        if (opts.size_set) {
                stream_config.size = requested_size;
        }
        if (opts.format_set) {
                if (!is_format_available(stream_config, requested_pixfmt)) {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "format=%s (%s) is not "
                                        "advertised by libcamera for this "
                                        "stream; use -t libcamera:caps to list "
                                        "available formats\n",
                                        opts.format_name.c_str(),
                                        requested_pixfmt.toString().c_str());
                        return false;
                }
                stream_config.pixelFormat = requested_pixfmt;
        }
        if (opts.sensor_set) {
                libcamera::SensorConfiguration sensor_config = {};
                sensor_config.outputSize = {
                        opts.sensor_width,
                        opts.sensor_height,
                };
                sensor_config.bitDepth = 10;
                config->sensorConfig = sensor_config;
        }
        if (stream_config.bufferCount < 4) {
                stream_config.bufferCount = 4;
        }

        log_msg(LOG_LEVEL_INFO,
                        MOD_NAME "requested stream: size=%s pixelFormat=%s\n",
                        stream_config.size.toString().c_str(),
                        stream_config.pixelFormat.toString().c_str());
        if (opts.sensor_set) {
                log_msg(LOG_LEVEL_INFO,
                                MOD_NAME "requested sensor output: %ux%u "
                                "bitDepth=10\n",
                                opts.sensor_width, opts.sensor_height);
        }

        log_msg(LOG_LEVEL_INFO,
                        MOD_NAME "requested libcamera pixel format: %s "
                        "(format=%s); UltraGrid codec selected: %s\n",
                        requested_pixfmt.toString().c_str(),
                        opts.format_name.c_str(), get_codec_name(opts.codec));

        if (opts.size_set || opts.sensor_set || opts.format_set ||
                        opts.fps_set || opts.hdr_set) {
                log_msg(LOG_LEVEL_INFO,
                                MOD_NAME "requested overrides: size=%s "
                                "sensor=%s format=%s fps=%s hdr=%s "
                                "bufferCount=%u\n",
                                check_requested_size ?
                                        effective_requested_size.toString().c_str() :
                                        "default",
                                opts.sensor_set ?
                                        config->sensorConfig->outputSize.toString().c_str() :
                                        "default",
                                check_requested_format ?
                                        requested_pixfmt.toString().c_str() :
                                        "default",
                                opts.fps_set ? "set" : "default",
                                opts.hdr_set ? opts.hdr_name.c_str() : "default",
                                stream_config.bufferCount);
        }
        if (opts.fps_set) {
                log_msg(LOG_LEVEL_INFO,
                                MOD_NAME "requested fps=%.3f; will apply during "
                                "camera start if supported\n",
                                opts.fps);
        }
        if (opts.hdr_set &&
                        opts.hdr_mode == libcamera_options::HdrMode::Sensor &&
                        opts.sensor_set &&
                        is_known_imx708_camera(camera_model_string(s->camera))) {
                const libcamera::Size requested_sensor_size(opts.sensor_width,
                                opts.sensor_height);
                if (find_known_sensor_cap(s->camera, requested_sensor_size,
                                opts.hdr_mode) == nullptr) {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "IMX708 sensor HDR supports "
                                        "only sensor=2304x1296 max_fps=30; "
                                        "omit sensor= or use sensor=2304x1296.\n");
                        return false;
                }
        }

        libcamera::CameraConfiguration::Status status = config->validate();
        log_msg(LOG_LEVEL_INFO, MOD_NAME "configuration validation: %s\n",
                        validation_status_to_string(status));
        log_stream_config("validated stream", stream_config);
        log_msg(LOG_LEVEL_INFO,
                        MOD_NAME "validated stream: size=%s pixelFormat=%s\n",
                        stream_config.size.toString().c_str(),
                        stream_config.pixelFormat.toString().c_str());
        if (config->sensorConfig) {
                log_msg(LOG_LEVEL_INFO,
                                MOD_NAME "validated sensor output: %s "
                                "bitDepth=%u analogCrop=%s\n",
                                config->sensorConfig->outputSize.toString().c_str(),
                                config->sensorConfig->bitDepth,
                                config->sensorConfig->analogCrop.toString().c_str());
        } else if (opts.sensor_set) {
                log_msg(LOG_LEVEL_INFO,
                                MOD_NAME "validated sensor output: none\n");
        }
        log_msg(LOG_LEVEL_INFO,
                        MOD_NAME "negotiated libcamera pixel format after "
                        "validate: %s\n",
                        stream_config.pixelFormat.toString().c_str());

        if (status == libcamera::CameraConfiguration::Invalid) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "requested configuration is invalid\n");
                return false;
        }
        if (opts.hdr_set &&
                        opts.hdr_mode == libcamera_options::HdrMode::Sensor &&
                        !config->sensorConfig) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "hdr requires a known sensor mode; "
                                "omit sensor= or use sensor=2304x1296 for "
                                "IMX708 sensor HDR\n");
                return false;
        }
        if (opts.sensor_set && config->sensorConfig) {
                const std::string camera_model = camera_model_string(s->camera);
                const libcamera::Size validated_sensor_size =
                        config->sensorConfig->outputSize;
                const sensor_mode_cap *cap = find_known_sensor_cap(s->camera,
                                validated_sensor_size, opts.hdr_mode);
                if (opts.hdr_set &&
                                opts.hdr_mode == libcamera_options::HdrMode::Sensor &&
                                cap == nullptr) {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "IMX708 sensor HDR supports "
                                        "only sensor=2304x1296 max_fps=30; "
                                        "omit sensor= or use sensor=2304x1296.\n");
                        return false;
                }
                if (cap != nullptr && opts.fps_set &&
                                opts.fps > cap->max_fps + 0.001) {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "requested fps=%.3f exceeds "
                                        "max_fps=%u for sensor=%s hdr=%s on "
                                        "%s.\n",
                                        opts.fps, cap->max_fps,
                                        validated_sensor_size.toString().c_str(),
                                        hdr_mode_to_string(opts.hdr_mode),
                                        camera_model.c_str());
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "Use fps=%u, choose a faster "
                                        "cropped sensor mode such as "
                                        "sensor=1536x864, or use a different "
                                        "camera.\n",
                                        cap->max_fps);
                        return false;
                }
                if (cap == nullptr) {
                        if (is_known_imx708_camera(camera_model)) {
                                log_msg(LOG_LEVEL_WARNING,
                                                MOD_NAME "no known max_fps cap "
                                                "for sensor=%s hdr=%s on %s; "
                                                "not rejecting requested fps\n",
                                                validated_sensor_size.toString().c_str(),
                                                hdr_mode_to_string(opts.hdr_mode),
                                                camera_model.c_str());
                        } else {
                                log_msg(LOG_LEVEL_WARNING,
                                                MOD_NAME "fps caps are not "
                                                "known for camera %s; not "
                                                "rejecting requested fps\n",
                                                camera_model.c_str());
                        }
                } else {
                        log_msg(LOG_LEVEL_INFO,
                                        MOD_NAME "known sensor cap: sensor=%s "
                                        "hdr=%s max_fps=%u (%s)\n",
                                        validated_sensor_size.toString().c_str(),
                                        cap->hdr_name, cap->max_fps,
                                        cap->note);
                }
        }
        if (stream_config.pixelFormat != requested_pixfmt) {
                if (check_requested_format) {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "requested format %s (%s) was "
                                        "adjusted by libcamera to %s\n",
                                        opts.format_name.c_str(),
                                        requested_pixfmt.toString().c_str(),
                                        stream_config.pixelFormat.toString().c_str());
                } else {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "default YUV420/I420 handoff "
                                        "requires libcamera format %s, got %s\n",
                                        requested_pixfmt.toString().c_str(),
                                        stream_config.pixelFormat.toString().c_str());
                }
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
        log_stream_config("configured stream", stream_config);
        log_msg(LOG_LEVEL_INFO,
                        MOD_NAME "negotiated libcamera pixel format after "
                        "configure: %s\n",
                        stream_config.pixelFormat.toString().c_str());
        if (stream_config.pixelFormat != requested_pixfmt) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "requested format %s (%s) was "
                                "changed by libcamera configure to %s\n",
                                opts.format_name.c_str(),
                                requested_pixfmt.toString().c_str(),
                                stream_config.pixelFormat.toString().c_str());
                return false;
        }

        s->stream = stream_config.stream();
        if (s->stream == nullptr) {
                log_msg(LOG_LEVEL_ERROR, MOD_NAME "configured stream is null\n");
                return false;
        }
        s->stream_config = stream_config;

        const unsigned int width = stream_config.size.width;
        const unsigned int height = stream_config.size.height;
        const unsigned int expected_stride =
                static_cast<unsigned int>(vc_get_linesize(width, opts.codec));
        if (is_packed_422(opts.codec) &&
                        stream_config.stride > expected_stride) {
                log_msg(LOG_LEVEL_INFO,
                                MOD_NAME "%s stride %u includes padding; "
                                "packed copy mode: row-by-row to "
                                "UltraGrid stride %u\n",
                                get_codec_name(opts.codec),
                                stream_config.stride, expected_stride);
        } else if (is_packed_422(opts.codec)) {
                log_msg(LOG_LEVEL_INFO,
                                MOD_NAME "%s stride %u; packed copy mode: "
                                "contiguous\n",
                                get_codec_name(opts.codec),
                                stream_config.stride);
        } else if (opts.codec == I420) {
                if (stream_config.stride < width) {
                        log_msg(LOG_LEVEL_ERROR,
                                        MOD_NAME "unsupported %s stride %u "
                                        "for width %u; expected at least %u "
                                        "for direct UltraGrid %s handoff\n",
                                        opts.format_name.c_str(),
                                        stream_config.stride, width, width,
                                        get_codec_name(opts.codec));
                        return false;
                }
                log_msg(LOG_LEVEL_INFO,
                                MOD_NAME "%s luma stride %u; planar copy "
                                "mode: %s\n",
                                get_codec_name(opts.codec),
                                stream_config.stride,
                                stream_config.stride == width ?
                                        "contiguous" : "row-by-row");
        } else if (stream_config.stride != expected_stride) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "unsupported %s stride %u for width "
                                "%u; expected %u for direct UltraGrid %s "
                                "handoff\n",
                                opts.format_name.c_str(), stream_config.stride,
                                width, expected_stride,
                                get_codec_name(opts.codec));
                return false;
        }

        const double configured_fps =
                opts.fps_set ? static_cast<double>(opts.fps) : 0.0;
        log_msg(LOG_LEVEL_INFO,
                        MOD_NAME "selected UltraGrid codec=%s size=%ux%u "
                        "fps=%.2f%s\n",
                        get_codec_name(opts.codec), width, height,
                        configured_fps,
                        opts.fps_set ? "" : " (unspecified)");

        s->desc = {
                width,
                height,
                opts.codec,
                configured_fps,
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
                        MOD_NAME "libcamera capture started: %ux%u %s\n",
                        width, height, get_codec_name(opts.codec));
        return true;
}

struct fps_test_target {
        std::string camera_id;
        libcamera::Size size = {};
};

struct fps_test_result {
        unsigned int requested_fps = 0;
        int64_t duration_us = 0;
        const char *result = "FAIL";
        double measured_fps = 0.0;
        unsigned int frames = 0;
        std::string note;
};

struct saved_env {
        std::string name;
        std::string value;
        bool was_set = false;
};

saved_env set_env_temporarily(const char *name, const char *value)
{
        saved_env saved;
        saved.name = name;
        const char *old_value = getenv(name);
        if (old_value != nullptr) {
                saved.was_set = true;
                saved.value = old_value;
        }
        setenv(name, value, 1);
        return saved;
}

void restore_env(const saved_env &saved)
{
        if (saved.was_set) {
                setenv(saved.name.c_str(), saved.value.c_str(), 1);
        } else {
                unsetenv(saved.name.c_str());
        }
}

struct fps_test_state {
        std::shared_ptr<libcamera::Camera> camera;
        libcamera::Stream *stream = nullptr;
        std::mutex lock;
        std::condition_variable cv;
        bool stopping = false;
        unsigned int completed = 0;
        unsigned int measured_frames = 0;
        uint64_t first_timestamp = 0;
        uint64_t last_timestamp = 0;
        bool queue_failed = false;

        void request_completed(libcamera::Request *request)
        {
                std::lock_guard<std::mutex> guard(lock);
                if (stopping) {
                        cv.notify_all();
                        return;
                }

                if (request->status() == libcamera::Request::RequestComplete) {
                        completed += 1;
                        const unsigned int warmup_frames = 3;
                        if (completed > warmup_frames) {
                                const libcamera::FrameBuffer *buffer =
                                        request->findBuffer(stream);
                                if (buffer != nullptr) {
                                        const uint64_t timestamp =
                                                buffer->metadata().timestamp;
                                        if (measured_frames == 0) {
                                                first_timestamp = timestamp;
                                        }
                                        last_timestamp = timestamp;
                                        measured_frames += 1;
                                }
                        }
                }

                request->reuse(libcamera::Request::ReuseBuffers);
                int queue_ret = camera->queueRequest(request);
                if (queue_ret != 0) {
                        queue_failed = true;
                }
                cv.notify_all();
        }
};

bool configure_test_stream(libcamera::Camera *camera,
                const libcamera_options &opts,
                std::unique_ptr<libcamera::CameraConfiguration> *config_out,
                std::string *error)
{
        std::unique_ptr<libcamera::CameraConfiguration> config =
                camera->generateConfiguration({
                        libcamera::StreamRole::VideoRecording });
        if (!config || config->empty()) {
                *error = "generateConfiguration failed";
                return false;
        }

        libcamera::StreamConfiguration &stream_config = config->at(0);
        if (opts.size_set) {
                stream_config.size = opts.size;
        }
        if (!is_format_available(stream_config, opts.pixel_format)) {
                *error = "requested format " + opts.format_name +
                        " is not advertised by libcamera";
                return false;
        }
        stream_config.pixelFormat = opts.pixel_format;
        if (opts.sensor_set) {
                libcamera::SensorConfiguration sensor_config = {};
                sensor_config.outputSize = {
                        opts.sensor_width,
                        opts.sensor_height,
                };
                sensor_config.bitDepth = 10;
                config->sensorConfig = sensor_config;
        }
        if (stream_config.bufferCount < 4) {
                stream_config.bufferCount = 4;
        }

        libcamera::CameraConfiguration::Status status = config->validate();
        if (status == libcamera::CameraConfiguration::Invalid) {
                *error = "configuration invalid";
                return false;
        }
        if (stream_config.pixelFormat != opts.pixel_format) {
                *error = "requested format " + opts.format_name +
                        " adjusted to " + stream_config.pixelFormat.toString();
                return false;
        }
        if (opts.size_set && stream_config.size != opts.size) {
                *error = "requested size adjusted to " +
                        stream_config.size.toString();
                return false;
        }

        *config_out = std::move(config);
        return true;
}

bool get_fps_test_target(const libcamera_options &opts, fps_test_target *target)
{
        libcamera::CameraManager camera_manager;
        if (camera_manager.start() != 0) {
                log_msg(LOG_LEVEL_ERROR, MOD_NAME "failed to start CameraManager\n");
                return false;
        }

        bool ok = false;
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
                        target->camera_id = camera->id();
                        if (camera->acquire() != 0) {
                                log_msg(LOG_LEVEL_ERROR,
                                                MOD_NAME "failed to acquire "
                                                "camera\n");
                        } else {
                                std::unique_ptr<libcamera::CameraConfiguration>
                                        config;
                                std::string error;
                                ok = configure_test_stream(camera.get(), opts,
                                                &config, &error);
                                if (ok) {
                                        target->size = config->at(0).size;
                                } else {
                                        log_msg(LOG_LEVEL_ERROR,
                                                        MOD_NAME "%s\n",
                                                        error.c_str());
                                }
                                camera->release();
                        }
                }
        }

        camera_manager.stop();
        return ok;
}

fps_test_result run_fps_test_attempt(const libcamera_options &opts,
                unsigned int fps)
{
        fps_test_result result;
        result.requested_fps = fps;
        result.duration_us = static_cast<int64_t>(
                        std::llround(1000000.0 / fps));

        libcamera::CameraManager camera_manager;
        if (camera_manager.start() != 0) {
                result.note = "CameraManager start failed";
                return result;
        }

        std::shared_ptr<libcamera::Camera> camera;
        bool camera_acquired = false;
        bool callback_connected = false;
        bool camera_started = false;
        std::unique_ptr<libcamera::FrameBufferAllocator> allocator;
        std::vector<std::unique_ptr<libcamera::Request>> requests;
        std::unique_ptr<libcamera::CameraConfiguration> config;
        fps_test_state test_state;

        auto cleanup = [&] {
                {
                        std::lock_guard<std::mutex> guard(test_state.lock);
                        test_state.stopping = true;
                }
                test_state.cv.notify_all();
                if (camera && camera_started) {
                        camera->stop();
                        camera_started = false;
                }
                if (camera && callback_connected) {
                        camera->requestCompleted.disconnect(&test_state);
                        callback_connected = false;
                }
                requests.clear();
                allocator.reset();
                config.reset();
                test_state.camera.reset();
                if (camera && camera_acquired) {
                        camera->release();
                        camera_acquired = false;
                }
                camera.reset();
                camera_manager.stop();
        };

        auto cameras = camera_manager.cameras();
        if (cameras.empty()) {
                result.note = "no cameras found";
                cameras.clear();
                cleanup();
                return result;
        }
        if (opts.camera_index >= cameras.size()) {
                result.note = "camera index out of range";
                cameras.clear();
                cleanup();
                return result;
        }

        camera = cameras[opts.camera_index];
        cameras.clear();
        test_state.camera = camera;
        if (camera->acquire() != 0) {
                result.note = "camera acquire failed";
                cleanup();
                return result;
        }
        camera_acquired = true;

        std::string error;
        if (!configure_test_stream(camera.get(), opts, &config, &error)) {
                result.note = error;
                cleanup();
                return result;
        }

        int configure_ret = camera->configure(config.get());
        if (configure_ret != 0) {
                result.note = "configure failed";
                cleanup();
                return result;
        }

        libcamera::StreamConfiguration &stream_config = config->at(0);
        libcamera::Stream *stream = stream_config.stream();
        if (stream == nullptr) {
                result.note = "configured stream is null";
                cleanup();
                return result;
        }
        test_state.stream = stream;

        allocator = std::make_unique<libcamera::FrameBufferAllocator>(camera);
        int alloc_ret = allocator->allocate(stream);
        if (alloc_ret < 0) {
                result.note = "buffer allocation failed";
                cleanup();
                return result;
        }

        const auto &buffers = allocator->buffers(stream);
        if (buffers.empty()) {
                result.note = "no buffers allocated";
                cleanup();
                return result;
        }
        for (const std::unique_ptr<libcamera::FrameBuffer> &buffer : buffers) {
                std::unique_ptr<libcamera::Request> request =
                        camera->createRequest();
                if (!request) {
                        result.note = "request creation failed";
                        cleanup();
                        return result;
                }
                int add_ret = request->addBuffer(stream, buffer.get());
                if (add_ret != 0) {
                        result.note = "addBuffer failed";
                        cleanup();
                        return result;
                }
                requests.push_back(std::move(request));
        }

        libcamera::ControlList controls(camera->controls());
        if (camera->controls().count(
                        libcamera::controls::FrameDurationLimits.id()) == 0) {
                result.note = "FrameDurationLimits unavailable";
                cleanup();
                return result;
        }
        controls.set(libcamera::controls::FrameDurationLimits,
                        { result.duration_us, result.duration_us });

        camera->requestCompleted.connect(&test_state,
                        &fps_test_state::request_completed);
        callback_connected = true;

        int start_ret = camera->start(&controls);
        if (start_ret != 0) {
                result.note = "start failed";
                cleanup();
                return result;
        }
        camera_started = true;

        for (const std::unique_ptr<libcamera::Request> &request : requests) {
                int queue_ret = camera->queueRequest(request.get());
                if (queue_ret != 0) {
                        result.note = "queueRequest failed";
                        cleanup();
                        return result;
                }
        }

        const auto deadline = std::chrono::steady_clock::now() +
                std::chrono::seconds(3);
        bool enough_samples = false;
        {
                std::unique_lock<std::mutex> lock(test_state.lock);
                while (!test_state.queue_failed &&
                                std::chrono::steady_clock::now() < deadline) {
                        if (test_state.measured_frames >= 8 &&
                                        test_state.last_timestamp >
                                                test_state.first_timestamp) {
                                const double span_sec =
                                        (test_state.last_timestamp -
                                                test_state.first_timestamp) /
                                        1000000000.0;
                                if (span_sec >= 1.0) {
                                        enough_samples = true;
                                        break;
                                }
                        }
                        test_state.cv.wait_until(lock, deadline);
                }
        }

        {
                std::lock_guard<std::mutex> guard(test_state.lock);
                result.frames = test_state.measured_frames;
                if (test_state.queue_failed) {
                        result.note = "requeue failed";
                } else if (test_state.measured_frames >= 2 &&
                                test_state.last_timestamp >
                                        test_state.first_timestamp) {
                        const double span_sec =
                                (test_state.last_timestamp -
                                        test_state.first_timestamp) /
                                1000000000.0;
                        result.measured_fps =
                                (test_state.measured_frames - 1) / span_sec;
                }
        }

        if (result.note.empty()) {
                if (!enough_samples && result.frames == 0) {
                        result.result = "TIMEOUT";
                        result.note = "no completed requests";
                } else if (!enough_samples && result.frames < 8) {
                        result.result = "TIMEOUT";
                        result.note = "insufficient completed requests";
                } else if (result.measured_fps <= 0.0) {
                        result.result = "TIMEOUT";
                        result.note = "timestamp measurement unavailable";
                } else {
                        const double tolerance = fps <= 30 ? 0.15 : 0.10;
                        const double rel_error =
                                std::abs(result.measured_fps - fps) / fps;
                        result.result = rel_error <= tolerance ? "OK" :
                                "ADJUSTED";
                }
        }

        cleanup();
        return result;
}

int run_fps_test(const libcamera_options &opts)
{
        std::vector<saved_env> saved_envs;
        if (!opts.test_verbose) {
                saved_envs.push_back(set_env_temporarily(
                                "LIBCAMERA_LOG_LEVELS", "*:ERROR"));
        }

        if (opts.fps_set) {
                log_msg(LOG_LEVEL_WARNING,
                                MOD_NAME "test mode: fps=%.3f limits test to "
                                "that single value\n",
                                opts.fps);
        }

        fps_test_target target;
        if (!get_fps_test_target(opts, &target)) {
                for (const saved_env &saved : saved_envs) {
                        restore_env(saved);
                }
                return VIDCAP_INIT_FAIL;
        }

        printf("libcamera FPS test\n");
        printf("Device %zu) %s\n", opts.camera_index, target.camera_id.c_str());
        printf("Format: %s\n", opts.format_name.c_str());
        printf("Size: %s\n\n", target.size.toString().c_str());
        if (!opts.test_verbose) {
                printf("Running tests...\n\n");
        }

        const std::vector<unsigned int> default_fps = {
                24, 25, 30, 50, 60, 75, 90, 100, 120,
        };
        const std::vector<unsigned int> fps_values =
                opts.fps_set ? std::vector<unsigned int>{
                        static_cast<unsigned int>(std::llround(opts.fps)) } :
                default_fps;

        std::vector<fps_test_result> results;
        bool have_ok = false;
        unsigned int consecutive_failures_after_ok = 0;
        for (unsigned int fps : fps_values) {
                fps_test_result result = run_fps_test_attempt(opts, fps);
                if (strcmp(result.result, "ADJUSTED") == 0 &&
                                result.note.empty()) {
                        result.note = "limited/adjusted by camera pipeline";
                }
                const bool ok_like = strcmp(result.result, "OK") == 0 ||
                        strcmp(result.result, "ADJUSTED") == 0;
                if (ok_like) {
                        have_ok = true;
                        consecutive_failures_after_ok = 0;
                } else if (have_ok) {
                        consecutive_failures_after_ok += 1;
                }
                results.push_back(result);

                if (consecutive_failures_after_ok >= 2) {
                        fps_test_result stop_marker;
                        stop_marker.requested_fps = fps;
                        stop_marker.note = "stopped after 2 consecutive failures";
                        results.back().note = results.back().note.empty() ?
                                stop_marker.note :
                                results.back().note + "; " + stop_marker.note;
                        break;
                }
        }

        for (const saved_env &saved : saved_envs) {
                restore_env(saved);
        }

        printf("%-9s | %-8s | %-8s | %-6s | %s\n",
                        "Requested", "Result", "Measured", "Frames", "Note");
        double highest_ok = 0.0;
        double first_adjusted = 0.0;
        double adjusted_limit = 0.0;
        std::vector<unsigned int> ok_values;
        for (const fps_test_result &result : results) {
                char measured[32] = "-";
                if (result.measured_fps > 0.0) {
                        snprintf(measured, sizeof measured, "%.1f",
                                        result.measured_fps);
                }
                printf("%-9u | %-8s | %-8s | %-6u | %s\n",
                                result.requested_fps, result.result, measured,
                                result.frames, result.note.c_str());

                if (strcmp(result.result, "OK") == 0) {
                        highest_ok = result.requested_fps;
                        ok_values.push_back(result.requested_fps);
                } else if (strcmp(result.result, "ADJUSTED") == 0) {
                        if (first_adjusted == 0.0) {
                                first_adjusted = result.requested_fps;
                        }
                        if (adjusted_limit == 0.0 ||
                                        (result.measured_fps > 0.0 &&
                                                result.measured_fps <
                                                        adjusted_limit)) {
                                adjusted_limit = result.measured_fps;
                        }
                }
        }

        printf("\nSummary:\n");
        if (highest_ok > 0.0) {
                printf("OK up to: %.0f fps\n", highest_ok);
        } else {
                printf("OK up to: none\n");
        }
        if (first_adjusted > 0.0) {
                if (adjusted_limit > 0.0) {
                        printf("Pipeline limit appears around: %.1f fps\n",
                                        adjusted_limit);
                } else {
                        printf("Pipeline limit appears around: %.0f fps\n",
                                        first_adjusted);
                }
        } else {
                printf("Pipeline limit appears around: not detected\n");
        }
        printf("Recommended stable choices:");
        if (ok_values.empty()) {
                printf(" none\n");
        } else {
                const size_t max_recommendations =
                        std::min<size_t>(ok_values.size(), 5);
                for (size_t i = 0; i < max_recommendations; ++i) {
                        printf("%s%u", i == 0 ? " " : ", ", ok_values[i]);
                }
                printf("\n");
        }

        return VIDCAP_INIT_NOERR;
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
        if (opts.action == libcamera_options::Action::Test) {
                return run_fps_test(opts);
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
                                print_known_sensor_caps(cameras[i]);
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
                        bool hdr_changed = false;
                        if (!resolve_and_apply_hdr_mode(&opts, camera,
                                        &hdr_changed)) {
                                have_config = false;
                                goto init_done;
                        }
                        if (hdr_changed) {
                                log_msg(LOG_LEVEL_INFO,
                                                MOD_NAME "WDR/HDR control "
                                                "changed; restarting "
                                                "CameraManager before final "
                                                "configuration\n");
                                camera.reset();
                                cameras.clear();
                                camera_manager->stop();
                                camera_manager.reset();
                                camera_manager =
                                        std::make_unique<libcamera::CameraManager>();
                                if (camera_manager->start() != 0) {
                                        log_msg(LOG_LEVEL_ERROR,
                                                        MOD_NAME "failed to "
                                                        "restart CameraManager "
                                                        "after HDR setup\n");
                                        have_config = false;
                                        goto init_done;
                                }
                                cameras = camera_manager->cameras();
                                if (opts.camera_index >= cameras.size()) {
                                        log_msg(LOG_LEVEL_ERROR,
                                                        MOD_NAME "camera index "
                                                        "%zu out of range after "
                                                        "HDR setup (found %zu "
                                                        "cameras)\n",
                                                        opts.camera_index,
                                                        cameras.size());
                                        have_config = false;
                                        goto init_done;
                                }
                                camera = cameras[opts.camera_index];
                                log_msg(LOG_LEVEL_INFO,
                                                MOD_NAME "using camera %zu "
                                                "after HDR setup: %s\n",
                                                opts.camera_index,
                                                camera->id().c_str());
                        }
                        new_state = new vidcap_libcamera_state();
                        new_state->camera_manager = std::move(camera_manager);
                        new_state->camera = camera;
                        have_config = configure_camera(new_state, opts);
                }
        }
init_done:

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
                        log_ae_diag_if_needed(s, request,
                                        buffer->metadata().sequence);
                        log_focus_metadata_if_needed(s, request);

                        const unsigned int width = s->stream_config.size.width;
                        const unsigned int height = s->stream_config.size.height;
                        const codec_t codec = s->desc.color_spec;
                        const size_t frame_size =
                                vc_get_datalen(width, height, codec);
                        const auto planes = buffer->planes();
                        const auto metadata_planes = buffer->metadata().planes();
                        auto mapped_it = s->mapped_buffers.find(buffer);

                        if (mapped_it == s->mapped_buffers.end() ||
                                        s->frame->tiles[0].data_len <
                                                frame_size) {
                                log_msg(LOG_LEVEL_ERROR, MOD_NAME
                                                "unexpected mapped buffer, "
                                                "skipping frame\n");
                        } else {
                                char *dst = s->frame->tiles[0].data;
                                const mapped_buffer &mapped = mapped_it->second;
                                if (s->copied_frames == 0) {
                                        log_frame_buffer(*buffer);
                                }
                                bool copied = false;
                                if (codec == I420) {
                                        const size_t y_row_bytes = width;
                                        const size_t chroma_row_bytes =
                                                width / 2;
                                        const size_t y_stride =
                                                s->stream_config.stride;
                                        const size_t chroma_stride =
                                                y_stride / 2;
                                        const size_t y_size =
                                                y_row_bytes * height;
                                        const size_t chroma_size =
                                                chroma_row_bytes * (height / 2);
                                        const size_t y_needed =
                                                y_stride * (height - 1) +
                                                y_row_bytes;
                                        const size_t chroma_needed =
                                                chroma_stride *
                                                        (height / 2 - 1) +
                                                chroma_row_bytes;
                                        const char *copy_mode =
                                                y_stride == y_row_bytes ?
                                                "contiguous" : "row-by-row";
                                        if (planes.size() != 3 ||
                                                        metadata_planes.size() < 3 ||
                                                        mapped.planes.size() != 3 ||
                                                        y_stride < y_row_bytes ||
                                                        chroma_stride < chroma_row_bytes ||
                                                        mapped.planes[0].len < y_needed ||
                                                        mapped.planes[1].len < chroma_needed ||
                                                        mapped.planes[2].len < chroma_needed ||
                                                        metadata_planes[0].bytesused < y_needed ||
                                                        metadata_planes[1].bytesused < chroma_needed ||
                                                        metadata_planes[2].bytesused < chroma_needed) {
                                                log_msg(LOG_LEVEL_ERROR,
                                                                MOD_NAME
                                                                "unexpected "
                                                                "YUV420 buffer "
                                                                "layout, "
                                                                "skipping "
                                                                "frame\n");
                                        } else {
                                                if (y_stride == y_row_bytes) {
                                                        memcpy(dst,
                                                                        mapped.planes[0].data,
                                                                        y_size);
                                                        memcpy(dst + y_size,
                                                                        mapped.planes[1].data,
                                                                        chroma_size);
                                                        memcpy(dst + y_size + chroma_size,
                                                                        mapped.planes[2].data,
                                                                        chroma_size);
                                                } else {
                                                        for (unsigned int y = 0;
                                                                        y < height;
                                                                        ++y) {
                                                                memcpy(dst + y * y_row_bytes,
                                                                                mapped.planes[0].data + y * y_stride,
                                                                                y_row_bytes);
                                                        }
                                                        for (unsigned int y = 0;
                                                                        y < height / 2;
                                                                        ++y) {
                                                                memcpy(dst + y_size + y * chroma_row_bytes,
                                                                                mapped.planes[1].data + y * chroma_stride,
                                                                                chroma_row_bytes);
                                                                memcpy(dst + y_size + chroma_size + y * chroma_row_bytes,
                                                                                mapped.planes[2].data + y * chroma_stride,
                                                                                chroma_row_bytes);
                                                        }
                                                }
                                                copied = true;
                                                if (s->copied_frames < 3) {
                                                        log_msg(LOG_LEVEL_INFO,
                                                                        MOD_NAME
                                                                        "grab copied "
                                                                        "frame %u: "
                                                                        "%ux%u Y=%zu "
                                                                        "U=%zu V=%zu "
                                                                        "stride=%zu "
                                                                        "copy=%s "
                                                                        "sequence=%u\n",
                                                                        s->copied_frames + 1,
                                                                        width, height,
                                                                        y_size,
                                                                        chroma_size,
                                                                        chroma_size,
                                                                        y_stride,
                                                                        copy_mode,
                                                                        buffer->metadata().sequence);
                                                }
                                        }
                                } else {
                                        const size_t row_bytes =
                                                vc_get_linesize(width, codec);
                                        const size_t src_stride =
                                                s->stream_config.stride;
                                        const size_t src_needed =
                                                src_stride * (height - 1) +
                                                row_bytes;
                                        const char *copy_mode =
                                                src_stride == row_bytes ?
                                                "contiguous" : "row-by-row";
                                        if (planes.size() != 1 ||
                                                        metadata_planes.empty() ||
                                                        mapped.planes.size() != 1 ||
                                                        src_stride < row_bytes ||
                                                        mapped.planes[0].len < src_needed ||
                                                        metadata_planes[0].bytesused < src_needed) {
                                                log_msg(LOG_LEVEL_ERROR,
                                                                MOD_NAME
                                                                "unexpected %s "
                                                                "buffer layout, "
                                                                "skipping "
                                                                "frame\n",
                                                                get_codec_name(codec));
                                        } else {
                                                if (src_stride == row_bytes) {
                                                        memcpy(dst,
                                                                        mapped.planes[0].data,
                                                                        frame_size);
                                                } else {
                                                        for (unsigned int y = 0;
                                                                        y < height;
                                                                        ++y) {
                                                                memcpy(dst + y * row_bytes,
                                                                                mapped.planes[0].data + y * src_stride,
                                                                                row_bytes);
                                                        }
                                                }
                                                copied = true;
                                                if (s->copied_frames < 3) {
                                                        log_msg(LOG_LEVEL_INFO,
                                                                        MOD_NAME
                                                                        "grab copied "
                                                                        "frame %u: "
                                                                        "%ux%u %s=%zu "
                                                                        "stride=%zu "
                                                                        "copy=%s "
                                                                        "sequence=%u\n",
                                                                        s->copied_frames + 1,
                                                                        width, height,
                                                                        get_codec_name(codec),
                                                                        frame_size,
                                                                        src_stride,
                                                                        copy_mode,
                                                                        buffer->metadata().sequence);
                                                }
                                        }
                                }
                                if (copied) {
                                        s->frame->tiles[0].data_len = frame_size;
                                        s->frame->timestamp =
                                                buffer->metadata().timestamp *
                                                90 / 1000000;
                                        s->copied_frames += 1;

                                        request->reuse(
                                                        libcamera::Request::ReuseBuffers);
                                        if (!request_focus_controls(s, s->opts,
                                                        request)) {
                                                return nullptr;
                                        }
                                        int queue_ret =
                                                s->camera->queueRequest(request);
                                        if (queue_ret != 0) {
                                                log_msg(LOG_LEVEL_ERROR,
                                                                MOD_NAME
                                                                "failed to "
                                                                "requeue "
                                                                "request: %d\n",
                                                                queue_ret);
                                        }
                                        return s->frame;
                                }
                        }
                }

                request->reuse(libcamera::Request::ReuseBuffers);
                if (!request_focus_controls(s, s->opts, request)) {
                        return nullptr;
                }
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
