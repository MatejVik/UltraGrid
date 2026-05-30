/**
 * @file   capture_filter/interlace.c
 * @author CESNET z.s.p.o.
 *
 * Converts progressive capture frames to merged interlaced frames by weaving
 * scanlines from pairs of consecutive progressive frames.
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
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "capture_filter.h"
#include "compat/c23.h"      // IWYU pragma: keep
#include "debug.h"
#include "lib_common.h"
#include "types.h"
#include "utils/color_out.h"
#include "utils/macros.h"
#include "video_codec.h"
#include "video_frame.h"

struct module;

#define MAGIC    to_fourcc('C', 'F', 'i', 'l')
#define MOD_NAME "[interlace filter] "

struct state_interlace_cf {
        uint32_t magic;
        struct video_desc saved_desc;
        struct video_desc out_desc;
        struct video_frame *prev;
        bool configured;
        bool failed;
        unsigned int produced_frames;
};

static void
usage(void)
{
        color_printf("Capture filter " TBOLD("interlace")
                     " converts progressive frames to merged interlaced "
                     "frames by weaving pairs of input frames.\n\n");
        color_printf("Usage:\n\t" TBOLD("--capture-filter interlace")
                     " -t <progressive capture>\n\n");
        color_printf("Input fps N becomes output fps N/2 with "
                     TBOLD("INTERLACED_MERGED") " metadata.\n");
        color_printf("UYVY/YUYV 4:2:2 input is recommended. Other codecs are "
                     "kept in their original pixel format but may produce "
                     "chroma/field artefacts.\n");
}

static int
init(struct module *parent, const char *cfg, void **state)
{
        UNUSED(parent);

        if (strcasecmp(cfg, "help") == 0) {
                usage();
                return 1;
        }
        if (strlen(cfg) > 0) {
                log_msg(LOG_LEVEL_ERROR, MOD_NAME "unknown option: %s\n", cfg);
                return -1;
        }

        struct state_interlace_cf *s = calloc(1, sizeof *s);
        if (s == NULL) {
                return -1;
        }
        s->magic = MAGIC;
        *state = s;
        return 0;
}

static void
done(void *state)
{
        struct state_interlace_cf *s = state;
        assert(s->magic == MAGIC);
        vf_free(s->prev);
        free(s);
}

static bool
codec_is_recommended_422(codec_t codec)
{
        return codec == UYVY || codec == YUYV;
}

static bool
codec_is_supported_for_line_weave(codec_t codec)
{
        return codec == UYVY || codec == YUYV || codec == I420;
}

static void
copy_metadata(struct video_frame *out, const struct video_frame *in)
{
        memcpy(&out->VF_METADATA_START, &in->VF_METADATA_START,
                        VF_METADATA_SIZE);
}

static bool
configure_if_needed(struct state_interlace_cf *s, const struct video_frame *in)
{
        struct video_desc desc = video_desc_from_frame(in);
        if (s->configured && video_desc_eq(desc, s->saved_desc)) {
                return !s->failed;
        }

        vf_free(s->prev);
        s->prev = NULL;
        s->saved_desc = desc;
        s->out_desc = desc;
        s->out_desc.fps = desc.fps / 2.0;
        s->out_desc.interlacing = INTERLACED_MERGED;
        s->configured = true;
        s->failed = false;
        s->produced_frames = 0;

        if (desc.interlacing != PROGRESSIVE) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "input must be progressive, got %s\n",
                                get_interlacing_description(desc.interlacing));
                s->failed = true;
                return false;
        }
        if (desc.height % 2 != 0) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "input height %u is not even; cannot "
                                "field-weave safely\n",
                                desc.height);
                s->failed = true;
                return false;
        }
        if (desc.fps <= 0.0) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "input fps %.2f is invalid; cannot "
                                "derive interlaced output rate\n",
                                desc.fps);
                s->failed = true;
                return false;
        }
        if (!codec_is_supported_for_line_weave(desc.color_spec)) {
                log_msg(LOG_LEVEL_ERROR,
                                MOD_NAME "input codec %s cannot be safely "
                                "field-weaved without pixel conversion\n",
                                get_codec_name(desc.color_spec));
                s->failed = true;
                return false;
        }

        if (!codec_is_recommended_422(desc.color_spec)) {
                color_printf(TRED(MOD_NAME "WARNING: input codec/chroma is not "
                                "4:2:2; field-weave may create chroma "
                                "artefacts; UYVY/YUYV recommended") "\n");
        }

        log_msg(LOG_LEVEL_INFO,
                        MOD_NAME "enabled: %ux%up%.2g -> %ux%ui%.2g; "
                        "output desc fps %.2f, interlacing %s\n",
                        desc.width, desc.height, desc.fps,
                        desc.width, desc.height, desc.fps,
                        s->out_desc.fps,
                        get_interlacing_description(
                                s->out_desc.interlacing));
        return true;
}

static void
copy_frame_data(struct video_frame *dst, const struct video_frame *src)
{
        for (unsigned int tile = 0; tile < src->tile_count; ++tile) {
                memcpy(dst->tiles[tile].data, src->tiles[tile].data,
                                src->tiles[tile].data_len);
                dst->tiles[tile].data_len = src->tiles[tile].data_len;
        }
}

static void
weave_packed_tile(struct tile *out, const struct tile *first,
                const struct tile *second, codec_t codec)
{
        const size_t linesize = vc_get_linesize(out->width, codec);
        for (unsigned int y = 0; y < out->height; ++y) {
                const struct tile *src = y % 2 == 0 ? first : second;
                memcpy(out->data + y * linesize, src->data + y * linesize,
                                linesize);
        }
}

static void
weave_planar_tile(struct tile *out, const struct tile *first,
                const struct tile *second, codec_t codec)
{
        int subsampling[8] = { 0 };
        codec_get_planes_subsampling(codec, subsampling);

        size_t offset = 0;
        for (int plane = 0; plane < 4; ++plane) {
                const int x_sub = subsampling[plane * 2];
                const int y_sub = subsampling[plane * 2 + 1];
                if (x_sub == 0 || y_sub == 0) {
                        break;
                }

                const size_t plane_width = (out->width + x_sub - 1) / x_sub;
                const size_t plane_height = (out->height + y_sub - 1) / y_sub;
                for (size_t y = 0; y < plane_height; ++y) {
                        const struct tile *src =
                                (y * y_sub) % 2 == 0 ? first : second;
                        memcpy(out->data + offset + y * plane_width,
                                        src->data + offset + y * plane_width,
                                        plane_width);
                }
                offset += plane_width * plane_height;
        }
}

static void
weave_frame(struct video_frame *out, const struct video_frame *first,
                const struct video_frame *second)
{
        for (unsigned int tile = 0; tile < out->tile_count; ++tile) {
                if (codec_is_planar(out->color_spec)) {
                        weave_planar_tile(&out->tiles[tile],
                                        &first->tiles[tile],
                                        &second->tiles[tile],
                                        out->color_spec);
                } else {
                        weave_packed_tile(&out->tiles[tile],
                                        &first->tiles[tile],
                                        &second->tiles[tile],
                                        out->color_spec);
                }
        }
}

static struct video_frame *
filter(void *state, struct video_frame *in)
{
        if (in == NULL) {
                return NULL;
        }

        struct state_interlace_cf *s = state;
        assert(s->magic == MAGIC);

        if (!configure_if_needed(s, in)) {
                VIDEO_FRAME_DISPOSE(in);
                return NULL;
        }

        if (s->prev == NULL) {
                s->prev = vf_alloc_desc_data(video_desc_from_frame(in));
                if (s->prev == NULL) {
                        VIDEO_FRAME_DISPOSE(in);
                        return NULL;
                }
                copy_metadata(s->prev, in);
                copy_frame_data(s->prev, in);
                VIDEO_FRAME_DISPOSE(in);
                return NULL;
        }

        struct video_frame *out = vf_alloc_desc_data(s->out_desc);
        if (out == NULL) {
                VIDEO_FRAME_DISPOSE(in);
                return NULL;
        }
        out->callbacks.dispose = vf_free;
        copy_metadata(out, s->prev);
        weave_frame(out, s->prev, in);

        vf_free(s->prev);
        s->prev = NULL;
        VIDEO_FRAME_DISPOSE(in);

        s->produced_frames += 1;
        if (s->produced_frames <= 3) {
                log_msg(LOG_LEVEL_INFO,
                                MOD_NAME "weaved frame %u: %ux%u %s "
                                "fps=%.2f interlacing=%s\n",
                                s->produced_frames,
                                out->tiles[0].width, out->tiles[0].height,
                                get_codec_name(out->color_spec), out->fps,
                                get_interlacing_description(out->interlacing));
        }

        return out;
}

static const struct capture_filter_info capture_filter_interlace = {
        .init = init,
        .done = done,
        .filter = filter,
};

REGISTER_MODULE(interlace, &capture_filter_interlace,
                LIBRARY_CLASS_CAPTURE_FILTER, CAPTURE_FILTER_ABI_VERSION);
