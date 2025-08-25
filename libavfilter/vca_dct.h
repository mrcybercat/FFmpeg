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

#include "avfilter.h"
#include "libavutil/eval.h"

#ifndef AVFILTER_VCADCT_H
#define AVFILTER_VCADCT_H

static const int16_t weights_dct8[64];
static const int16_t weights_dct16[256];
static const int16_t weights_dct32[1024];

const int16_t g_t4[4][4];
const int16_t g_t8[8][8];
const int16_t g_t16[16][16];
const int16_t g_t32[32][32];

uint32_t calc_weighted_coeff(unsigned blocksize, int16_t *coeff_buffer, int enable_lowpass);

void ff_vca_dct8(const int16_t* src, int16_t* dst, int bit_depth);

void ff_vca_dct16(const int16_t* src, int16_t* dst, int bit_depth);

void ff_vca_dct32(const int16_t* src, int16_t* dst, int bit_depth);

void ff_vca_lowpass_dct8(const int16_t* src, int16_t* dst, int bit_depth);

void ff_vca_lowpass_dct16(const int16_t* src, int16_t* dst, int bit_depth);

void ff_vca_lowpass_dct32(const int16_t* src, int16_t* dst, int bit_depth);

#endif

