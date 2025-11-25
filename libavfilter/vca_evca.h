
/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * functions and constants for EVCA
 */



#ifndef AVFILTER_EVCA_H
#define AVFILTER_EVCA_H

#include "avfilter.h"
#include "libavutil/eval.h"
#include "libavformat/avio.h"
#include "vca_dct.h"

int ff_init_evca(VCAAlgoContext *ctx, int n_blocks, int blocksize);
void ff_perform_evca(AVFilterContext *ctx, AVFrame *in, FilterLink *inl, VCAContext *v, int plane_i);
void ff_uninit_evca(VCAAlgoContext *ctx);

typedef struct EVCAAlgoContext {
    VCAAlgoContext base;
    uint32_t *energy;
    uint32_t *energy_weight_pxl;
    uint32_t *energy_weight_pxl_prev;
    double *energy_dif;
} EVCAAlgoContext;

typedef struct ThreadDataEVCA {
    void (*perform_dct)(const int16_t* block, int16_t* dst, int bit_depth);
    int stride;
    int blocksize;

    int enable_lowpass;
    int is_first_frame;

    uint8_t *src;

    VCAPlaneInfo *plane;
    EVCAAlgoContext *algoctx;
    
    uint32_t *partial_sums_E;
    double *partial_sums_h;
} ThreadDataEVCA;


static const VCAAlgoVTable evca_vtable = {
    .init_algo = ff_init_evca,
    .perform_algo = ff_perform_evca,
    .uninit_algo = ff_uninit_evca,
};

#endif