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
 * functions and constants for VCA
 */

 
#ifndef AVFILTER_VCA_H
#define AVFILTER_VCA_H

#include "avfilter.h"
#include "libavutil/eval.h"
#include "libavformat/avio.h"
#include "vca_dct.h"

int ff_init_ovca(VCAAlgoContext *ctx, int n_blocks, int blocksize);
void ff_perform_ovca(AVFilterContext *ctx, AVFrame *in, FilterLink *inl, VCAContext *v, int plane_i);
void ff_uninit_ovca(VCAAlgoContext *ctx);

typedef struct OVCAAlgoContext {
    VCAAlgoContext base;
    uint32_t *energy;
    uint32_t *energy_prev;
    double *energy_dif;
} OVCAAlgoContext;

typedef struct ThreadDataOVCA {
    void (*perform_dct)(const int16_t* block, int16_t* dst, int bit_depth);
    int stride;
    int blocksize;

    int enable_lowpass;

    uint8_t *src;

    VCAPlaneInfo *plane;
    OVCAAlgoContext *algoctx;
    
    uint32_t *partial_sums;
} ThreadDataOVCA;

static const VCAAlgoVTable ovca_vtable = {
    .init_algo = ff_init_ovca,
    .perform_algo = ff_perform_ovca,
    .uninit_algo = ff_uninit_ovca,
};


#endif

// void ff_perform_vca(AVFilterContext *ctx, AVFilterLink *inlink, AVFrame *in, FilterLink *inl,
//                    VCAContext *v, int plane_i);


