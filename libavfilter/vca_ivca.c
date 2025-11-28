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
#include "vca_ivca.h"

#define FLT_WINDOW          5   
#define GOP_SIZE            4
#define IVCA_MAGIC_NUMBER 500

static double weights[4] = { 0.11, 0.04, 0.0001, 0.0005 };


#define DEFINE_CALC_ENERGY(BLOCKSIZE)                                                                                   \
static uint32_t calc_energy_##BLOCKSIZE##_slice(int stride, uint8_t *src, VCAPlaneInfo *plane, IVCAAlgoContext *result, \
                                    int enable_lowpass, int slice_start, int slice_end, uint32_t *partial_sum,          \
                                    void (*perform_dct)(const int16_t* block, int16_t* dst, int bit_depth)) {           \
    int block_i = (slice_start / BLOCKSIZE) * plane->w_blocks;                                                          \
    uint32_t sliceTexture = 0;                                                                                          \
    ALIGN_VAR_32(int16_t, block_buffer[BLOCKSIZE * BLOCKSIZE]);                                                         \
    ALIGN_VAR_32(int16_t, out_buffer[BLOCKSIZE * BLOCKSIZE]);                                                           \
    const unsigned bit_depth = plane->bit_depth;                                                                        \
    if (bit_depth != 8 && bit_depth != 10 && bit_depth != 12)                                                           \
        return AVERROR(AVERROR_INVALIDDATA);                                                                            \
    for (unsigned blockY = slice_start; blockY < slice_end; blockY += BLOCKSIZE) {                                      \
        int padding_b = FFMAX(((int)(blockY + BLOCKSIZE) - (int)(plane->h_pxls_src)), 0);                               \
        for (unsigned blockX = 0; blockX < plane->w_pxls; blockX += BLOCKSIZE) {                                        \
            int offset = blockX * plane->pxl_depth + (blockY * stride);                                                 \
            int padding_r = FFMAX((int)(blockX + BLOCKSIZE) - (int)(plane->w_pxls_src), 0);                             \
            ff_copy_vals_buffer(plane->pxl_depth, offset, BLOCKSIZE, src, stride, block_buffer, padding_r, padding_b);  \
            perform_dct(block_buffer, out_buffer, bit_depth);                                                           \
            result->energy[block_i] = ff_calc_weighted_coeff(BLOCKSIZE, out_buffer, enable_lowpass);                    \
            sliceTexture += result->energy[block_i];                                                                    \
            for (uint16_t l = 0; l != FLT_WINDOW; ++l) {                                                                \
                int16_t undershot = blockY / BLOCKSIZE < (int16_t)(FLT_WINDOW / 2 - l);                                 \
                int16_t overshot = blockY / BLOCKSIZE > (plane->h_blocks - 1) - (int16_t)(l - FLT_WINDOW / 2);          \
                if (overshot || undershot)                                                                              \
                    continue;                                                                                           \
                result->energy_ver_shift[l][block_i + (int16_t)(l - FLT_WINDOW / 2) * plane->w_blocks] = result->energy[block_i];\
            }                                                                                                   \
            block_i++;                                                                                          \
        }                                                                                                       \
    }                                                                                                           \
    *partial_sum = sliceTexture;                                                                                \
    return sliceTexture;                                                                                        \
}
                                      
DEFINE_CALC_ENERGY(32)
DEFINE_CALC_ENERGY(16)
DEFINE_CALC_ENERGY(8)

static int calc_energy_filter_slice(AVFilterContext *ctx, void *arg, int job, int nb_jobs)
{
    ThreadDataIVCA *th = arg;

    int block_row_start = (th->plane->h_blocks * job)     / nb_jobs;
    int block_row_end   = (th->plane->h_blocks * (job+1)) / nb_jobs;

    int slice_start = block_row_start * th->blocksize;
    int slice_end   = block_row_end   * th->blocksize;

    switch (th->blocksize) {
        case 32:
            calc_energy_32_slice(th->stride, th->src, th->plane, th->algoctx, th->enable_lowpass,
                                 slice_start, slice_end, &th->partial_sums[job], th->perform_dct);
            break;
        case 16:
            calc_energy_16_slice(th->stride, th->src, th->plane, th->algoctx, th->enable_lowpass, 
                                 slice_start, slice_end, &th->partial_sums[job], th->perform_dct);
            break;
        case 8:
            calc_energy_8_slice(th->stride, th->src, th->plane, th->algoctx, th->enable_lowpass, 
                                slice_start, slice_end, &th->partial_sums[job], th->perform_dct);
            break;
        default:
            return AVERROR(AVERROR_INVALIDDATA);
    }
    return 0;
}


static uint32_t calc_energy(AVFilterContext *ctx, int linesize, uint8_t *src, VCAPlaneInfo *plane,
                            IVCAAlgoContext *algoctx, int blocksize, int enable_lowpass, void* perform_dct)
{
    uint32_t frameTexture = 0;
    int stride = linesize / plane->pxl_depth;

    int nb_threads = ff_filter_get_nb_threads(ctx);

    ThreadDataIVCA th = {
        .stride = stride,
        .blocksize = blocksize,
        .enable_lowpass = enable_lowpass,
        .src = src,
        .plane = plane,
        .algoctx = algoctx,
        .partial_sums = av_calloc(nb_threads, sizeof(uint32_t)),
        .perform_dct = perform_dct
    };

    ff_filter_execute(ctx, calc_energy_filter_slice, &th, NULL, FFMIN(plane->h_blocks, nb_threads));

    for (int i = 0; i < nb_threads; i++)
        frameTexture += th.partial_sums[i];

    av_free(th.partial_sums);

    return (uint32_t)((double)frameTexture /(plane->n_blocks * E_norm_factor));
}

static double calc_energy_diff(VCAPlaneInfo *plane, IVCAAlgoContext *result, int n_blocks, int blocksize) {
	double diff_sum = 0.0;

    for (size_t i = 0; i < n_blocks; i++) {
		result->energy_dif[i] = (uint32_t)(SAFE_ABS((int)(result->energy[i]) - (int)(result->energy_prev[i])));
        diff_sum += result->energy_dif[i];
        size_t yIdx = i / (plane->w_pxls_src  / blocksize);
        size_t xIdx = i % (plane->w_pxls_src  / blocksize);
        if (xIdx > (FLT_WINDOW / 2 - 1) && 
            xIdx < ((plane->w_pxls_src / blocksize) - 1 - (FLT_WINDOW / 2 - 1)) && 
            yIdx > (FLT_WINDOW / 2 - 1) && 
            yIdx < ((plane->h_pxls_src / blocksize) - 1 - (FLT_WINDOW / 2 - 1)))
        {
            uint32_t *energy_window = calloc(FLT_WINDOW, sizeof(uint32_t));
            uint32_t *energy_window_prev = calloc(FLT_WINDOW, sizeof(uint32_t));
            uint32_t horr_best_corr = 0, ver_best_corr = 0;
            for (uint16_t m = 0; m != 2; ++m) {
                for (uint32_t l = 0; l != FLT_WINDOW; ++l) {
                    if (m == 0) {
                        energy_window[l] = result->energy[i + l - FLT_WINDOW / 2] / IVCA_MAGIC_NUMBER;
                        energy_window_prev[l] = result->energy_prev[i + l - FLT_WINDOW / 2] / IVCA_MAGIC_NUMBER;
                    } else{
                        energy_window[l] = result->energy_ver_shift[FLT_WINDOW - 1 - l][i] / IVCA_MAGIC_NUMBER;
                        energy_window_prev[l] = result->energy_ver_shift_prev[FLT_WINDOW - 1 - l][i] / IVCA_MAGIC_NUMBER;
                    }
                }
                uint32_t best_corr = 0, test_corr = 0, range_abs = 1;
                uint32_t calcRangeHalf = FLT_WINDOW / 2 - range_abs;
                for (uint32_t l = 0; l != 2 * range_abs + 1; ++l) {
                    int32_t offset   = l - range_abs;
                    uint32_t product = 0, sum_a = 0, sum_b = 0;
                    for (uint32_t n = FLT_WINDOW / 2 - calcRangeHalf; n != FLT_WINDOW / 2 + calcRangeHalf + 1; ++n) {
                        product += energy_window[n + offset] * energy_window_prev[n];
                        sum_a += energy_window[n + offset] * energy_window[n + offset];
                        sum_b += energy_window_prev[n] * energy_window_prev[n];
                    }
                    if (sum_a < 100 || sum_b < 100)
                        break;
                    test_corr = 256 * product * product / sum_a / sum_b;
                    if (test_corr > best_corr)
                        best_corr = test_corr;
                    }
                    if (m == 0)
                        horr_best_corr = best_corr;
                    else
                        ver_best_corr = best_corr;
            }
            uint32_t best_corr = (horr_best_corr + ver_best_corr) > 255
                                        ? (horr_best_corr > ver_best_corr ? horr_best_corr : ver_best_corr)
                                        : (horr_best_corr + ver_best_corr);

            diff_sum -= result->energy_dif[i];
            result->energy_dif[i] = result->energy_dif[i] * (256 - best_corr) / 256;
            diff_sum += result->energy_dif[i];                
        }        
    }
    return  diff_sum / (plane->n_blocks * h_norm_factor); 
}


av_cold int ff_init_ivca(VCAAlgoContext *ctx, int n_blocks, int blocksize) {
    IVCAAlgoContext *ivca = (IVCAAlgoContext *)ctx;

    // Free previous buffers in case they are allocated already
    av_freep(&ivca->energy_prev);
    av_freep(&ivca->energy_dif);
    av_freep(&ivca->energy);
            
    ivca->energy = av_malloc(n_blocks * sizeof(uint32_t));
    ivca->energy_prev = av_malloc(n_blocks * sizeof(uint32_t));
    ivca->energy_dif = av_malloc(n_blocks * sizeof(double)); 

    ivca->energy_ver_shift = av_malloc(FLT_WINDOW * sizeof(*ivca->energy_ver_shift));
    if (ivca->energy_ver_shift) 
        for (int i = 0; i < FLT_WINDOW; i++) 
            ivca->energy_ver_shift[i] = av_malloc(n_blocks * sizeof(**ivca->energy_ver_shift));
 
    ivca->energy_ver_shift_prev = av_malloc(FLT_WINDOW * sizeof(*ivca->energy_ver_shift_prev));
    if (ivca->energy_ver_shift_prev) 
        for (int i = 0; i < FLT_WINDOW; i++) 
            ivca->energy_ver_shift_prev[i] = av_malloc(n_blocks * sizeof(**ivca->energy_ver_shift_prev));

    ivca->pred = 0.0;
 
    if (!ivca->energy || ! ivca->energy_prev || !ivca->energy_dif || !ivca->energy_ver_shift || !ivca->energy_ver_shift_prev)
        return AVERROR(ENOMEM);
    return 0;
}

av_cold void ff_uninit_ivca(VCAAlgoContext *ctx) {
    IVCAAlgoContext *ivca = (IVCAAlgoContext *)ctx;

    av_freep(&ivca->energy_prev);
    av_freep(&ivca->energy_dif);
    av_freep(&ivca->energy);
    if (ivca->energy_ver_shift_prev) 
        for (int i = 0; i < FLT_WINDOW; i++) 
            av_freep(&ivca->energy_ver_shift_prev[i]);
    av_freep(&ivca->energy_ver_shift_prev);
    if (ivca->energy_ver_shift_prev) 
        for (int i = 0; i < FLT_WINDOW; i++) 
            av_freep(&ivca->energy_ver_shift_prev[i]);
    av_freep(&ivca->energy_ver_shift_prev);    
}


void ff_perform_ivca(AVFilterContext *ctx, AVFrame *in, FilterLink *inl, VCAContext *v, int plane_i)
{   
    uint32_t E = 0;
    double h = 0;

    IVCAAlgoContext *ivca = (IVCAAlgoContext *) v->algoctx[plane_i];

    E = calc_energy(ctx, in->linesize[plane_i], in->data[plane_i], v->plane[plane_i],
                    ivca, v->blocksize, v->enable_lowpass, v->perform_dct);

    // On first frame instead of calculating difference assign difference to 0
    if (inl->frame_count_out != 0)
        h = calc_energy_diff(v->plane[plane_i], ivca, v->plane[plane_i]->n_blocks, v->blocksize);
    else
        h = 0;
    
    // At the end copy current energy to the previous
    memcpy(ivca->energy_prev, ivca->energy, v->plane[plane_i]->n_blocks * sizeof(uint32_t));
    for (uint32_t l = 0; l != FLT_WINDOW; ++l) 
        memcpy(ivca->energy_ver_shift_prev[l], ivca->energy_ver_shift[l], v->plane[plane_i]->n_blocks * sizeof(uint32_t));

    switch ((inl->frame_count_out % IVCA_MAGIC_NUMBER / 2)% GOP_SIZE) {
        case 0:
            ivca->pred += E * weights[0];
            break;
        case 1:
            ivca->pred += h * weights[1];
            break;
        case 2:
            ivca->pred += h * weights[2];
            break;
        case 3:
            ivca->pred += h * weights[3];
            break;
    }

    // Dump info
    v->print(ctx, AV_LOG_INFO,
            "%4"PRId64,
            inl->frame_count_out);
    v->print(ctx, AV_LOG_INFO,
            ",%d,%f,%f",
            E, h, (ivca->pred / (inl->frame_count_out + 1)));
    v->print(ctx, AV_LOG_INFO, "\n");
}   