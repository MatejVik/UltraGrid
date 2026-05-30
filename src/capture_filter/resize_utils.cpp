/**
 * @file    capture_filter/resize_utils.cpp
 * @author  Gerard Castillo     <gerard.castillo@i2cat.net>
 *          Marc Palau          <marc.palau@i2cat.net>
 *          Martin Pulec        <martin.pulec@cesnet.cz>
 */
/*
 * Copyright (c) 2014      Fundació i2CAT, Internet I Innovació Digital a Catalunya
 * Copyright (c) 2015-2023 CESNET, z. s. p. o.
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
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *
 *      This product includes software developed by the Fundació i2CAT,
 *      Internet I Innovació Digital a Catalunya. This product also includes
 *      software developed by CESNET z.s.p.o.
 *
 * 4. Neither the name of the University nor of the Institute may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESSED OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <cstdlib>
#include <vector>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
#pragma GCC diagnostic ignored "-Wcast-qual"
#ifdef HAVE_OPENCV2_OPENCV_HPP
#include <opencv2/opencv.hpp>
#else
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#endif
#pragma GCC diagnostic pop

#include "capture_filter/resize_utils.h"
#include "debug.h"
#include "utils/color_out.h"
#include "utils/debug.h"                 // for DEBUG_TIMER_*
#include "video.h"

#define DEFAULT_ALGO INTER_LINEAR
#define MOD_NAME "[resize] "

using cv::INTER_AREA;
using cv::INTER_CUBIC;
using cv::INTER_LANCZOS4;
using cv::INTER_LINEAR;
// using cv::INTER_LINEAR_EXACT;
using cv::INTER_NEAREST;
// using cv::INTER_NEAREST_EXACT;
using cv::Mat;
using cv::Rect;
using cv::Size;

static Mat ug_to_rgb_mat(codec_t codec, int width, int height, char *indata) {
    Mat yuv;
    Mat rgb;
    int pix_fmt = CV_8UC2;
    int cv_color = 0;
    int num = 1;
    int den = 1;

    switch (codec) {
    case RG48:
        rgb.create(height, width, CV_16UC3);
        rgb.data = (uchar*)indata;
        return rgb;
    case RGB:
        rgb.create(height, width, CV_8UC3);
        rgb.data = (uchar*)indata;
        return rgb;
    case RGBA:
        cv_color = CV_RGBA2RGB;
        pix_fmt = CV_8UC4;
        break;
    case I420:
        pix_fmt = CV_8U;
        num = 3;
        den = 2;
        cv_color = CV_YUV2RGB_I420;
        break;
    case UYVY:
        cv_color = CV_YUV2RGB_UYVY;
        break;
    case YUYV:
        cv_color = CV_YUV2RGB_YUYV;
        break;
    default:
        LOG(LOG_LEVEL_ERROR) << MOD_NAME "Unsupported codec: " << codec << "\n";
        abort();
    }
    yuv.create(height * num / den, width, pix_fmt);
    yuv.data = (uchar*)indata;
    cvtColor(yuv, rgb, cv_color);

    return rgb;
}

static int
get_out_cv_data_type(codec_t pixfmt)
{
    return get_bits_per_component(pixfmt) == DEPTH16 ? CV_16UC3 : CV_8UC3;
}

static void
resize_frame_dimensions(char *indata, codec_t in_color, char *outdata, int width,
             int height, int target_width,
             int target_height, int algo)
{
    const codec_t out_color =
        get_bits_per_component(in_color) == 16 ? RG48 : RGB;
    Mat rgb = ug_to_rgb_mat(in_color, (int) width, (int) height, indata);

    double in_aspect = (double) width / height;
    double out_aspect = (double) target_width / target_height;
    Rect r;
    if (in_aspect == out_aspect) {
        r.x = 0;
        r.y = 0;
        r.width = target_width;
        r.height = target_height;
    } else if (in_aspect > out_aspect) {
        r.x = 0;
        r.width = target_width;
        r.height = (int) (target_width / in_aspect);
        r.y = (target_height - r.height) / 2;
        // clear top and bottom margin
        size_t linesize = vc_get_linesize(target_width, out_color);
        size_t top_margin_size = r.y * linesize;
        size_t bottom_margin_size = (target_height - r.y - r.height) * linesize;
        memset(outdata, 0, top_margin_size);
        memset(outdata + linesize * target_height - bottom_margin_size, 0, bottom_margin_size);
    } else {
        r.y = 0;
        r.height = target_height;
        r.width = (int) (target_height * in_aspect);
        r.x = (target_width - r.width) / 2;
        // clear left and right margins
        size_t linesize = vc_get_linesize(target_width, out_color);
        size_t left_margin_size = vc_get_linesize(r.x, out_color);
        size_t right_margin_size = vc_get_linesize(target_width - r.x - r.width, out_color);
        for (int i = 0; i < target_height; ++i) {
            memset(outdata + i * linesize, 0, left_margin_size);
            memset(outdata + (i + 1) * linesize - right_margin_size, 0, right_margin_size);
        }
    }

    Mat out((int) target_height, (int) target_width,
            get_out_cv_data_type(in_color), outdata);
    resize(rgb, out(r), r.size(), 0, 0, algo);
}

void
resize_frame(char *indata, codec_t in_color, char *outdata, int width,
             int height, struct resize_param *resize_spec)
{
    if (resize_spec->algo == RESIZE_ALGO_DFL) {
        resize_spec->algo = resize_algo_get_default();
        MSG(NOTICE, "using resize algorithm: %s\n",
          resize_algo_to_string(resize_spec->algo));
    }

    DEBUG_TIMER_START(resize);
    if (resize_spec->mode == resize_param::USE_FRACTION) {
        const double factor = resize_spec->factor;
        Mat rgb = ug_to_rgb_mat(in_color, (int) width, (int) height, indata);
        Mat out((int) (height * factor), (int) (width * factor),
                get_out_cv_data_type(in_color), outdata);
        resize(rgb, out, Size(0, 0), factor, factor, resize_spec->algo);
    } else if (resize_spec->mode == resize_param::USE_DIMENSIONS) {
        resize_frame_dimensions(indata, in_color, outdata, width, height,
                                resize_spec->target_width,
                                resize_spec->target_height, resize_spec->algo);
    } else {
        abort();
    }
    DEBUG_TIMER_STOP(resize);
}

void
resize_i420_frame(char *indata, char *outdata, int width, int height,
                  struct resize_param *resize_spec)
{
    if (resize_spec->algo == RESIZE_ALGO_DFL) {
        resize_spec->algo = resize_algo_get_default();
        MSG(NOTICE, "using resize algorithm: %s\n",
          resize_algo_to_string(resize_spec->algo));
    }

    int target_width = 0;
    int target_height = 0;
    if (resize_spec->mode == resize_param::USE_FRACTION) {
        target_width = width * resize_spec->factor;
        target_height = height * resize_spec->factor;
    } else if (resize_spec->mode == resize_param::USE_DIMENSIONS) {
        target_width = resize_spec->target_width;
        target_height = resize_spec->target_height;
    } else {
        abort();
    }

    const size_t in_y_size = width * height;
    const size_t out_y_size = target_width * target_height;

    Mat in_y(height, width, CV_8UC1, indata);
    Mat in_u(height / 2, width / 2, CV_8UC1, indata + in_y_size);
    Mat in_v(height / 2, width / 2, CV_8UC1,
             indata + in_y_size + in_y_size / 4);

    Mat out_y(target_height, target_width, CV_8UC1, outdata);
    Mat out_u(target_height / 2, target_width / 2, CV_8UC1,
              outdata + out_y_size);
    Mat out_v(target_height / 2, target_width / 2, CV_8UC1,
              outdata + out_y_size + out_y_size / 4);

    DEBUG_TIMER_START(resize);
    resize(in_y, out_y, out_y.size(), 0, 0, resize_spec->algo);
    resize(in_u, out_u, out_u.size(), 0, 0, resize_spec->algo);
    resize(in_v, out_v, out_v.size(), 0, 0, resize_spec->algo);
    DEBUG_TIMER_STOP(resize);
}

static void
get_resize_target_size(int width, int height, const struct resize_param *resize_spec,
                       int *target_width, int *target_height)
{
    if (resize_spec->mode == resize_param::USE_FRACTION) {
        *target_width = width * resize_spec->factor;
        *target_height = height * resize_spec->factor;
    } else if (resize_spec->mode == resize_param::USE_DIMENSIONS) {
        *target_width = resize_spec->target_width;
        *target_height = resize_spec->target_height;
    } else {
        abort();
    }
}

static void
unpack_packed422(const char *indata, codec_t in_color, int width, int height,
                 unsigned char *y, unsigned char *u, unsigned char *v)
{
    const unsigned char *src = reinterpret_cast<const unsigned char *>(indata);
    const int chroma_width = width / 2;
    const int in_linesize = vc_get_linesize(width, in_color);

    for (int row = 0; row < height; ++row) {
        const unsigned char *src_row = src + row * in_linesize;
        unsigned char *y_row = y + row * width;
        unsigned char *u_row = u + row * chroma_width;
        unsigned char *v_row = v + row * chroma_width;

        for (int pair = 0; pair < chroma_width; ++pair) {
            const unsigned char *p = src_row + pair * 4;
            if (in_color == UYVY) {
                u_row[pair] = p[0];
                y_row[pair * 2] = p[1];
                v_row[pair] = p[2];
                y_row[pair * 2 + 1] = p[3];
            } else {
                y_row[pair * 2] = p[0];
                u_row[pair] = p[1];
                y_row[pair * 2 + 1] = p[2];
                v_row[pair] = p[3];
            }
        }
    }
}

static void
pack_packed422(const unsigned char *y, const unsigned char *u,
               const unsigned char *v, codec_t out_color, char *outdata,
               int width, int height)
{
    unsigned char *dst = reinterpret_cast<unsigned char *>(outdata);
    const int chroma_width = width / 2;
    const int out_linesize = vc_get_linesize(width, out_color);

    for (int row = 0; row < height; ++row) {
        unsigned char *dst_row = dst + row * out_linesize;
        const unsigned char *y_row = y + row * width;
        const unsigned char *u_row = u + row * chroma_width;
        const unsigned char *v_row = v + row * chroma_width;

        for (int pair = 0; pair < chroma_width; ++pair) {
            unsigned char *p = dst_row + pair * 4;
            if (out_color == UYVY) {
                p[0] = u_row[pair];
                p[1] = y_row[pair * 2];
                p[2] = v_row[pair];
                p[3] = y_row[pair * 2 + 1];
            } else {
                p[0] = y_row[pair * 2];
                p[1] = u_row[pair];
                p[2] = y_row[pair * 2 + 1];
                p[3] = v_row[pair];
            }
        }
    }
}

void
resize_packed422_frame(char *indata, codec_t in_color, char *outdata,
                       int width, int height, struct resize_param *resize_spec)
{
    if (resize_spec->algo == RESIZE_ALGO_DFL) {
        resize_spec->algo = resize_algo_get_default();
        MSG(NOTICE, "using resize algorithm: %s\n",
          resize_algo_to_string(resize_spec->algo));
    }

    int target_width = 0;
    int target_height = 0;
    get_resize_target_size(width, height, resize_spec, &target_width,
                           &target_height);

    if (width % 2 != 0 || target_width % 2 != 0) {
        LOG(LOG_LEVEL_ERROR) << MOD_NAME "Packed 4:2:2 resize requires even "
                             << "input and output widths.\n";
        abort();
    }

    const int chroma_width = width / 2;
    const int target_chroma_width = target_width / 2;
    std::vector<unsigned char> in_y(width * height);
    std::vector<unsigned char> in_u(chroma_width * height);
    std::vector<unsigned char> in_v(chroma_width * height);
    std::vector<unsigned char> out_y(target_width * target_height);
    std::vector<unsigned char> out_u(target_chroma_width * target_height);
    std::vector<unsigned char> out_v(target_chroma_width * target_height);

    unpack_packed422(indata, in_color, width, height, in_y.data(), in_u.data(),
                     in_v.data());

    Mat in_y_mat(height, width, CV_8UC1, in_y.data());
    Mat in_u_mat(height, chroma_width, CV_8UC1, in_u.data());
    Mat in_v_mat(height, chroma_width, CV_8UC1, in_v.data());
    Mat out_y_mat(target_height, target_width, CV_8UC1, out_y.data());
    Mat out_u_mat(target_height, target_chroma_width, CV_8UC1, out_u.data());
    Mat out_v_mat(target_height, target_chroma_width, CV_8UC1, out_v.data());

    DEBUG_TIMER_START(resize);
    resize(in_y_mat, out_y_mat, out_y_mat.size(), 0, 0, resize_spec->algo);
    resize(in_u_mat, out_u_mat, out_u_mat.size(), 0, 0, resize_spec->algo);
    resize(in_v_mat, out_v_mat, out_v_mat.size(), 0, 0, resize_spec->algo);
    DEBUG_TIMER_STOP(resize);

    pack_packed422(out_y.data(), out_u.data(), out_v.data(), in_color, outdata,
                   target_width, target_height);
}

static const struct {
    int         val;
    const char *name;
} interp_map[] = {
        {INTER_NEAREST,        "nearest"      },
        { INTER_LINEAR,        "linear"       },
        { INTER_CUBIC,         "cubic"        },
        { INTER_AREA,          "area"         },
        { INTER_LANCZOS4,      "lanczos4"     },
        // { INTER_LINEAR_EXACT,  "linear_exact" },
        // { INTER_NEAREST_EXACT, "nearest_exact"},
};

int
resize_algo_from_string(const char *str)
{
    if (strcmp(str, "help") == 0) {
        color_printf("Available resize algorithms:\n");
        for (auto const &i : interp_map) {
            color_printf("\t" TBOLD("%s") "%s\n", i.name,
                         i.val == DEFAULT_ALGO ? " (default)" : "");
        }
        return RESIZE_ALGO_HELP_SHOWN;
    }
    for (auto const &i : interp_map) {
        if (strcmp(i.name, str) == 0) {
            return i.val;
        }
    }
    return RESIZE_ALGO_UNKN;
}

int
resize_algo_get_default(void)
{
    return DEFAULT_ALGO;
}

const char *
resize_algo_to_string(int algo)
{
    for (auto const &i : interp_map) {
        if (i.val == algo) {
            return i.name;
        }
    }
    return "(unknown algo!)";
}

/* vim: set expandtab sw=4: */
