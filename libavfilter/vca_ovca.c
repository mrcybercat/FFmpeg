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
#include "vca_ovca.h"


static uint32_t calc_energy_32_slice(int stride, uint8_t *src, VCAPlaneInfo *plane, OVCAAlgoContext *result, 
                                    int enable_lowpass, int slice_start, int slice_end, uint32_t *partial_sum,
                                    void (*perform_dct)(const int16_t* block, int16_t* dst, int bit_depth)) 
{
    int block_i = (slice_start / 32) * plane->w_blocks;
    uint32_t sliceTexture = 0;

    ALIGN_VAR_32(int16_t, block_buffer[32 * 32]);
    ALIGN_VAR_32(int16_t, out_buffer[32 * 32]);

    const unsigned bit_depth = plane->bit_depth;
    if (bit_depth != 8 && bit_depth != 10 && bit_depth != 12)
        return AVERROR(AVERROR_INVALIDDATA);

    for (unsigned blockY = slice_start; blockY < slice_end; blockY += 32) { 
        int padding_b = FFMAX(((int)(blockY + 32) - (int)(plane->h_pxls_src)), 0);
        for (unsigned blockX = 0; blockX < plane->w_pxls; blockX += 32){
            int offset = blockX * plane->pxl_depth + (blockY * stride);
            int padding_r = FFMAX((int)(blockX + 32) - (int)(plane->w_pxls_src), 0);

            // Copy values to block buffer 
            ff_copy_vals_buffer(plane->pxl_depth, offset, 32, src, stride, block_buffer, padding_r, padding_b);
            perform_dct(block_buffer, out_buffer, bit_depth);
            // Calculate energy and brightness
            // result.brightnessPerBlock[blockIndex] = uint32_t(sqrt(coeffBuffer[0]));
            result->energy[block_i] = ff_calc_weighted_coeff(32, out_buffer, enable_lowpass);
            
            sliceTexture += result->energy[block_i];
            block_i++;
        }
    }
    //av_freep(block_buffer);
    //av_freep(out_buffer);
    *partial_sum = sliceTexture;
    return  sliceTexture;
}

static uint32_t calc_energy_16_slice(int stride, uint8_t *src, VCAPlaneInfo *plane, OVCAAlgoContext *result, 
                                    int enable_lowpass, int slice_start, int slice_end, uint32_t *partial_sum,
                                    void (*perform_dct)(const int16_t* block, int16_t* dst, int bit_depth)) 
{    
    int block_i = (slice_start / 16) * plane->w_blocks;
    uint32_t sliceTexture = 0;

    ALIGN_VAR_32(int16_t, block_buffer[16 * 16]);
    ALIGN_VAR_32(int16_t, out_buffer[16 * 16]);

    const unsigned bit_depth = plane->bit_depth;
    if (bit_depth != 8 && bit_depth != 10 && bit_depth != 12)
        return AVERROR(AVERROR_INVALIDDATA);

    for (unsigned blockY = slice_start; blockY < slice_end; blockY += 16) { 
        int padding_b = FFMAX(((int)(blockY + 16) - (int)(plane->h_pxls_src)), 0);
        for (unsigned blockX = 0; blockX < plane->w_pxls; blockX += 16){
            int offset = blockX * plane->pxl_depth + (blockY * stride);
            int padding_r = FFMAX((int)(blockX + 16) - (int)(plane->w_pxls_src), 0);

            ff_copy_vals_buffer(plane->pxl_depth, offset, 16, src, stride, block_buffer, padding_r, padding_b);

            perform_dct(block_buffer, out_buffer, bit_depth);

            result->energy[block_i] = ff_calc_weighted_coeff(16, out_buffer, enable_lowpass);
            
            sliceTexture += result->energy[block_i];
            block_i++;
        }
    }
    *partial_sum = sliceTexture;
    return sliceTexture;
}

static uint32_t calc_energy_8_slice(int stride, uint8_t *src, VCAPlaneInfo *plane, OVCAAlgoContext *result, 
                                    int enable_lowpass, int slice_start, int slice_end, uint32_t *partial_sum,
                                    void (*perform_dct)(const int16_t* block, int16_t* dst, int bit_depth)) 
{
    int block_i = (slice_start / 8) * plane->w_blocks;
    uint32_t sliceTexture = 0;

    ALIGN_VAR_32(int16_t, block_buffer[8 * 8]);
    ALIGN_VAR_32(int16_t, out_buffer[8 * 8]);

    const unsigned bit_depth = plane->bit_depth;
    if (bit_depth != 8 && bit_depth != 10 && bit_depth != 12)
        return AVERROR(AVERROR_INVALIDDATA);

    for (unsigned blockY = slice_start; blockY < slice_end; blockY += 8) { 
        int padding_b = FFMAX(((int)(blockY + 8) - (int)(plane->h_pxls_src)), 0);
        for (unsigned blockX = 0; blockX < plane->w_pxls; blockX += 8) {
            int offset = blockX * plane->pxl_depth + (blockY * stride);
            int padding_r = FFMAX((int)(blockX + 8) - (int)(plane->w_pxls_src), 0);

            ff_copy_vals_buffer(plane->pxl_depth, offset, 8, src, stride, block_buffer, padding_r, padding_b);

            perform_dct(block_buffer, out_buffer, bit_depth);

            result->energy[block_i] = ff_calc_weighted_coeff(8, out_buffer, enable_lowpass);
            
            sliceTexture += result->energy[block_i];
            block_i++;
        }
    }
    *partial_sum = sliceTexture;
    return sliceTexture;
}

static int calc_energy_filter_slice(AVFilterContext *ctx, void *arg, int job, int nb_jobs)
{
    ThreadDataOVCA *th = arg;

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
                            OVCAAlgoContext *algoctx, int blocksize, int enable_lowpass, void* perform_dct)
{
    uint32_t frameTexture = 0;
    int stride = linesize / plane->pxl_depth;

    int nb_threads = ff_filter_get_nb_threads(ctx);

    ThreadDataOVCA th = {
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

static double calc_energy_diff(VCAPlaneInfo *plane, OVCAAlgoContext *result)
{
    int block_i = 0u;
    double diff_sum = 0;

    for (; block_i < plane->n_blocks; block_i++) {
        result->energy_dif[block_i] = abs((int)result->energy[block_i] - (int)result->energy_prev[block_i]);
        diff_sum += result->energy_dif[block_i];
    }
    return  diff_sum /(plane->n_blocks * h_norm_factor); 
}


void ff_init_ovca(VCAAlgoContext *ctx, int n_blocks, int blocksize) {
    OVCAAlgoContext *ovca = (OVCAAlgoContext *)ctx;

    // Free previous buffers in case they are allocated already
    av_freep(&ovca->energy_prev);
    av_freep(&ovca->energy_dif);
    av_freep(&ovca->energy);
            
    ovca->energy = av_malloc(n_blocks * sizeof(uint32_t));
    ovca->energy_prev = av_malloc(n_blocks * sizeof(uint32_t));
    ovca->energy_dif = av_malloc(n_blocks * sizeof(double)); 
    if (!ovca->energy || ! ovca->energy_prev || !ovca->energy_dif)
        return AVERROR(ENOMEM);
}

void ff_uninit_ovca(VCAAlgoContext *ctx) {
    OVCAAlgoContext *ovca = (OVCAAlgoContext *)ctx;

    av_freep(&ovca->energy_prev);
    av_freep(&ovca->energy_dif);
    av_freep(&ovca->energy);
}


void ff_perform_ovca(AVFilterContext *ctx, AVFrame *in, FilterLink *inl, VCAContext *v, int plane_i)
{   
    uint32_t E = 0;
    double h = 0;

    OVCAAlgoContext *ovca = (OVCAAlgoContext *) v->algoctx[plane_i];

    E = calc_energy(ctx, in->linesize[plane_i], in->data[plane_i], v->plane[plane_i],
                    ovca, v->blocksize, v->enable_lowpass, v->perform_dct);

    // On first frame instead of calculating difference assign difference to 0
    if (inl->frame_count_out != 0)
        h = calc_energy_diff(v->plane[plane_i], ovca);
    else
        h = 0;
    
    // At the end copy current energy to the previous
    memcpy(ovca->energy_prev, ovca->energy, v->plane[plane_i]->n_blocks * sizeof(uint32_t));

    // Dump info
    v->print(ctx, AV_LOG_INFO,
            "%4"PRId64,
            inl->frame_count_out);
    v->print(ctx, AV_LOG_INFO,
            ",%d,%f",
            E, h);
    v->print(ctx, AV_LOG_INFO, "\n");
}


/*
#define DEFINE_CALC_ENERGY(block_sz)                                                                        \
static uint32_t calc_energy_##block_sz##_slice(int stride, uint8_t *src, VCAPlaneInfo *plane,               \
                                               VCAResults *result, int enable_lowpass,                      \
                                               int slice_start, int slice_end, uint32_t *partial_sum,       \   
                                    void (*perform_dct)(const int16_t* block, int16_t* dst, int bit_depth)) \
{                                                                                       \
    int block_i = (slice_start / 32) * plane->w_blocks;                                 \
    uint32_t sliceTexture = 0;                                                          \         
    ALIGN_VAR_32(int16_t, block_buffer[32 * 32]);                                       \
    ALIGN_VAR_32(int16_t, out_buffer[32 * 32]);                                         \
    const unsigned bit_depth = plane->bit_depth;                                        \
    if (bit_depth != 8 && bit_depth != 10 && bit_depth != 12)                           \
        return AVERROR(AVERROR_INVALIDDATA);                                            \
    for (unsigned blockY = slice_start; blockY < slice_end; blockY += 32) {             \ 
        int padding_b = FFMAX(((int)(blockY + 32) - (int)(plane->h_pxls_src)), 0);      \              
        for (unsigned blockX = 0; blockX < plane->w_pxls; blockX += 32){                \
            int offset = blockX * plane->pxl_depth + (blockY * stride);                 \
            int padding_r = FFMAX((int)(blockX + 32) - (int)(plane->w_pxls_src), 0);    \
            copy_vals_buffer(plane->pxl_depth, offset, 32, src, stride,                 \
                            block_buffer, padding_r, padding_b);                        \
            perform_dct(block_buffer, out_buffer, bit_depth);                           \
            result->energy[block_i] =                                                   \
                calc_weighted_coeff(32, out_buffer, enable_lowpass);                    \            
            sliceTexture += result->energy[block_i];                                    \
            block_i++;                                                                  \
        }                                                                               \
    }                                                                                   \
    *partial_sum = sliceTexture;                                                        \
    return  sliceTexture;                                                               \
}
    //av_freep(block_buffer);
    //av_freep(out_buffer);
            // result.brightnessPerBlock[blockIndex] = uint32_t(sqrt(coeffBuffer[0]));
            // Calculate energy and brightness
            // Copy values to block buffer                                              

DEFINE_CALC_ENERGY(32)
DEFINE_CALC_ENERGY(16)
DEFINE_CALC_ENERGY(8)
*/