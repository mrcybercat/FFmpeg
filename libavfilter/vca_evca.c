/*
 *
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

#include "vca_dct.h"
#include "vca_evca.h"

#define DEFINE_CALC_EVCA_SLICE(BLOCKSIZE)                                                                                                       \
static uint32_t calc_evca_##BLOCKSIZE##_slice(int stride, uint8_t *src, VCAPlaneInfo *plane, EVCAAlgoContext *result, int enable_lowpass, \
                                     int is_first_frame, int slice_start, int slice_end, uint32_t *partial_sum_E, double *partial_sum_h,    \
                                     void (*perform_dct)(const int16_t* block, int16_t* dst, int bit_depth)) {                              \
    int block_i = (slice_start / BLOCKSIZE) * plane->w_blocks;                                                                          \
    uint32_t sliceTexture = 0, energy;                                                                                                  \
    double sliceDiff = 0, energy_diff;                                                                                                  \
    ALIGN_VAR_32(int16_t, block_buffer[BLOCKSIZE * BLOCKSIZE]);                                                                         \
    ALIGN_VAR_32(int16_t, out_buffer[BLOCKSIZE * BLOCKSIZE]);                                                                           \
    const unsigned bit_depth = plane->bit_depth;                                                                                        \
    if (bit_depth != 8 && bit_depth != 10 && bit_depth != 12)                                                                           \
        return AVERROR(AVERROR_INVALIDDATA);                                                                                            \
    for (unsigned blockY = slice_start; blockY < slice_end; blockY += BLOCKSIZE) {                                                      \
        int padding_b = FFMAX(((int)(blockY + BLOCKSIZE) - (int)(plane->h_pxls_src)), 0);                                               \
        for (unsigned blockX = 0; blockX < plane->w_pxls; blockX += BLOCKSIZE){                                                         \
            int offset = blockX * plane->pxl_depth + (blockY * stride);                                                                 \
            int padding_r = FFMAX((int)(blockX + BLOCKSIZE) - (int)(plane->w_pxls_src), 0);                                             \
            ff_copy_vals_buffer(plane->pxl_depth, offset, BLOCKSIZE, src, stride, block_buffer, padding_r, padding_b);                  \
            perform_dct(block_buffer, out_buffer, bit_depth);                                                                           \
            ff_calc_weighted_coeff_w_diff(BLOCKSIZE, out_buffer, result->energy_weight_pxl, result->energy_weight_pxl_prev,             \
                                          block_i*BLOCKSIZE*BLOCKSIZE, enable_lowpass, is_first_frame, &energy, &energy_diff);          \
            result->energy[block_i] = energy;                                                                                           \
            result->energy_dif[block_i] = energy_diff;                                                                                  \
            sliceTexture += result->energy[block_i];                                                                                    \
            sliceDiff += result->energy_dif[block_i];                                                                                   \
            block_i++;                                                                                                                  \
        }                                                                                                                               \
    }                                                                                                                                   \
    *partial_sum_E = sliceTexture;                                                                                                      \
    *partial_sum_h = sliceDiff;                                                                                                         \
    return  sliceTexture;                                                                                                               \
}

DEFINE_CALC_EVCA_SLICE(8)
DEFINE_CALC_EVCA_SLICE(16)
DEFINE_CALC_EVCA_SLICE(32)

static int calc_evca_filter_slice(AVFilterContext *ctx, void *arg, int job, int nb_jobs)
{
    ThreadDataEVCA *th = arg;

    int block_row_start = (th->plane->h_blocks * job)     / nb_jobs;
    int block_row_end   = (th->plane->h_blocks * (job+1)) / nb_jobs;

    int slice_start = block_row_start * th->blocksize;
    int slice_end   = block_row_end   * th->blocksize;

    switch (th->blocksize) {
        case 32:
            calc_evca_32_slice(th->stride, th->src, th->plane, th->algoctx, th->enable_lowpass, th->is_first_frame,
                               slice_start, slice_end, &th->partial_sums_E[job], &th->partial_sums_h[job], th->perform_dct);
            break;
        case 16:
            calc_evca_16_slice(th->stride, th->src, th->plane, th->algoctx, th->enable_lowpass, th->is_first_frame,
                               slice_start, slice_end, &th->partial_sums_E[job], &th->partial_sums_h[job], th->perform_dct);
            break;
        case 8:
            calc_evca_8_slice(th->stride, th->src, th->plane, th->algoctx, th->enable_lowpass, th->is_first_frame,
                              slice_start, slice_end, &th->partial_sums_E[job], &th->partial_sums_h[job], th->perform_dct);
            break;
        default:
            return AVERROR(AVERROR_INVALIDDATA);
    }
    return 0;
}

av_cold int ff_init_evca(VCAAlgoContext *ctx, int n_blocks, int blocksize) {
    EVCAAlgoContext *evca = (EVCAAlgoContext *)ctx;

    av_freep(&evca->energy);
    av_freep(&evca->energy_weight_pxl);
    av_freep(&evca->energy_weight_pxl_prev);
    av_freep(&evca->energy_dif);

    size_t weight_sz = n_blocks * blocksize * blocksize  * sizeof(uint32_t);
            
    evca->energy_weight_pxl = av_malloc(weight_sz);
    evca->energy_weight_pxl_prev = av_malloc(weight_sz);        
    evca->energy = av_malloc(n_blocks * sizeof(uint32_t));
    evca->energy_dif = av_malloc(n_blocks * sizeof(double)); 

    if (!evca->energy_weight_pxl || !evca->energy_weight_pxl_prev || !evca->energy || !evca->energy_dif)
        return AVERROR(ENOMEM);
    return 0;
}

av_cold void ff_uninit_evca(VCAAlgoContext *ctx) {
    EVCAAlgoContext *evca = (EVCAAlgoContext *)ctx;

    av_freep(&evca->energy);
    av_freep(&evca->energy_weight_pxl);
    av_freep(&evca->energy_weight_pxl_prev);
    av_freep(&evca->energy_dif);
}

void ff_perform_evca(AVFilterContext *ctx, AVFrame *in, FilterLink *inl, VCAContext *v, int plane_i)
{
    //calc_energy(ctx, in->linesize[plane_i], in->data[plane_i], v->plane[plane_i], v->result[plane_i], v->blocksize, v->enable_lowpass, v->perform_dct);
    // On first frame instead of calculating difference assign difference to 0
    uint32_t frameTexture = 0;
    double energyDifference = 0;

    uint32_t E = 0;
    double h = 0;

    EVCAAlgoContext *evca = (EVCAAlgoContext *) v->algoctx[plane_i];


    int is_first_frame = inl->frame_count_out == 0;

    int stride = in->linesize[plane_i] / v->plane[plane_i]->pxl_depth;

    int nb_threads = ff_filter_get_nb_threads(ctx);

    ThreadDataEVCA th = {
        .stride = stride,
        .blocksize = v->blocksize,
        .enable_lowpass = v->enable_lowpass,
        .is_first_frame = is_first_frame,
        .src = in->data[plane_i],
        .plane = v->plane[plane_i],
        .algoctx = evca,
        .partial_sums_E = av_calloc(nb_threads, sizeof(uint32_t)),
        .partial_sums_h = av_calloc(nb_threads, sizeof(double)),
        .perform_dct = v->perform_dct
    };

    ff_filter_execute(ctx, calc_evca_filter_slice, &th, NULL, FFMIN(v->plane[plane_i]->h_blocks, nb_threads));

    for (int i = 0; i < nb_threads; i++)
        frameTexture += th.partial_sums_E[i];

    for (int i = 0; i < nb_threads; i++)
        energyDifference += th.partial_sums_h[i];

    av_free(th.partial_sums_E);
    av_free(th.partial_sums_h);

    E = (uint32_t)((double)frameTexture /(v->plane[plane_i]->n_blocks * E_norm_factor));
    h = energyDifference /(v->plane[plane_i]->n_blocks * h_norm_factor);


    // At the end copy current energy to the previous
    memcpy(evca->energy_weight_pxl_prev, evca->energy_weight_pxl,
           v->plane[plane_i]->n_blocks * v->blocksize * v->blocksize * sizeof(uint32_t));

    // Dump info
    v->print(ctx, AV_LOG_INFO,
        "%4"PRId64,
        inl->frame_count_out);
    v->print(ctx, AV_LOG_INFO,
            ",%d,%f",
            E, h);

    v->print(ctx, AV_LOG_INFO, "\n");
}