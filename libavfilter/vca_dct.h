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
 * functions and constants for descrete cosine transform for VCA
 */

#include "libavutil/mem.h"
#include "filters.h"

#include "avfilter.h"
#include "libavutil/eval.h"
#include "libavformat/avio.h"
#include "filters.h"

#ifndef AVFILTER_VCADCT_H
#define AVFILTER_VCADCT_H

#if defined(__GNUC__)
#define ALIGN_VAR_32(T, var) T var __attribute__((aligned(32)))
#elif defined(_MSC_VER)
#define ALIGN_VAR_32(T, var) __declspec(align(32)) T var
#endif

enum VCAAlgorithmType {
    ALGO_STANDARD_VCA,   // 
    ALGO_ENHANCED_VCA,   // 
    ALGO_STEREO_VCA,     //
    ALGO_INTER_VCA,      //
};


typedef struct VCAPlaneInfo {
    int pxl_depth;
    int bit_depth;

    int w_pxls_src;
    int h_pxls_src;
    
    int n_blocks;

    int w_blocks;
    int h_blocks;

    int w_pxls;
    int h_pxls;
} VCAPlaneInfo;

typedef struct VCAResults {
    // globals
    uint32_t *energy;
    uint32_t *energy_prev;
    double *energy_dif;

    // EVCA fields
    uint32_t *energy_weight_pxl;
    uint32_t *energy_weight_pxl_prev;

    // SVCA fields
    uint32_t **energy_prev_stereo;

    // results summary
    double max_h;
    double max_E;
    
    double min_h;
    double min_E;

    uint32_t *energy_frames;
    double *energy_dif_frames;
} VCAResults;

typedef struct VCAContext {
    const AVClass *class;    
    AVIOContext *avio_context;
    void (*print)(AVFilterContext *ctx, int lvl, const char *msg, ...); // av_printf_format(2, 3);
    void (*perform_dct)(const int16_t* block, int16_t* dst, int bit_depth);
    void (*perform_xvca)(AVFilterContext *ctx, AVFilterLink *inlink, AVFrame *in, FilterLink *inl, struct VCAContext *v, int plane_i);

    // options 
    int algo;                   // < VCAAlgorithmType
    unsigned blocksize;
    int enable_lowpass;
    int enable_chroma;
    int enable_texture;
    int enable_simd;
    int summary;
    //int verbose;
    int yuview;
    int n_frames;
    char *file_str;

    // video frame properties
    VCAPlaneInfo **plane;
    int n_frames_processed;

    // results
    VCAResults **result;
} VCAContext;


static const int16_t weights_dct8[64];
static const int16_t weights_dct16[256];
static const int16_t weights_dct32[1024];

static const int16_t g_t4[4][4];
static const int16_t g_t8[8][8];
static const int16_t g_t16[16][16];
static const int16_t g_t32[32][32];

static const double E_norm_factor = 90;
static const double h_norm_factor = 18;

uint32_t ff_calc_weighted_coeff(unsigned blocksize, int16_t *coeff_buffer, int enable_lowpass);

void ff_calc_weighted_coeff_w_diff(unsigned blocksize, int16_t *coeff_buffer, uint32_t *energy_weight, uint32_t *energy_weight_prev,
                                   int offset, int enable_lowpass, int is_first_frame, uint32_t *weight, double *weight_diff);

void ff_vca_dct4_c(const int16_t* src, int16_t* dst, int bit_depth);

void ff_vca_dct8_c(const int16_t* src, int16_t* dst, int bit_depth);

void ff_vca_dct16_c(const int16_t* src, int16_t* dst, int bit_depth);

void ff_vca_dct32_c(const int16_t* src, int16_t* dst, int bit_depth);

void ff_vca_lowpass_dct8_c(const int16_t* src, int16_t* dst, int bit_depth);

void ff_vca_lowpass_dct16_c(const int16_t* src, int16_t* dst, int bit_depth);

void ff_vca_lowpass_dct32_c(const int16_t* src, int16_t* dst, int bit_depth);

int ff_vca_dct_init_x86(VCAContext *v);

void ff_copy_vals_buffer(unsigned pxl_depth, unsigned offset, unsigned blocksize, uint8_t *src, unsigned stride, int16_t *buffer, unsigned padding_r, unsigned padding_b);

#endif

