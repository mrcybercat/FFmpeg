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


static const double E_norm_factor = 90;
static const double h_norm_factor = 18;

static uint32_t calc_evca_32_slice(int stride, uint8_t *src, VCAPlaneInfo *plane, VCAResults *result, int enable_lowpass, 
                                     int is_first_frame, int slice_start, int slice_end, uint32_t *partial_sum_E, double *partial_sum_h,
                                     void (*perform_dct)(const int16_t* block, int16_t* dst, int bit_depth)) 
{
    int block_i = (slice_start / 32) * plane->w_blocks;
    uint32_t sliceTexture = 0, energy;
    double sliceDiff = 0, energy_diff;

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

            //int offset_weight = blockX * 1 + (blockY *  plane->w_pxls);
            ff_calc_weighted_coeff_w_diff(32, out_buffer, result->energy_weight_pxl, result->energy_weight_pxl_prev,
                                          block_i*32*32, enable_lowpass, is_first_frame, &energy, &energy_diff);
    
            result->energy[block_i] = energy;
            result->energy_dif[block_i] = energy_diff;
            
            sliceTexture += result->energy[block_i];
            sliceDiff += result->energy_dif[block_i];
            block_i++;
        }
    }
    //av_freep(block_buffer);
    //av_freep(out_buffer);
    *partial_sum_E = sliceTexture;
    *partial_sum_h = sliceDiff;
    return  sliceTexture;
}

static uint32_t calc_evca_16_slice(int stride, uint8_t *src, VCAPlaneInfo *plane, VCAResults *result, int enable_lowpass, 
                                     int is_first_frame, int slice_start, int slice_end, uint32_t *partial_sum_E, double *partial_sum_h,
                                     void (*perform_dct)(const int16_t* block, int16_t* dst, int bit_depth)) 
{
    int block_i = (slice_start / 16) * plane->w_blocks;
    uint32_t sliceTexture = 0, energy;
    double sliceDiff = 0, energy_diff;

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

            // Copy values to block buffer 
            ff_copy_vals_buffer(plane->pxl_depth, offset, 16, src, stride, block_buffer, padding_r, padding_b);
            perform_dct(block_buffer, out_buffer, bit_depth);

            ff_calc_weighted_coeff_w_diff(16, out_buffer, result->energy_weight_pxl, result->energy_weight_pxl_prev,
                                          block_i*16*16, enable_lowpass, is_first_frame, &energy, &energy_diff);
    
            result->energy[block_i] = energy;
            result->energy_dif[block_i] = energy_diff;
            
            sliceTexture += result->energy[block_i];
            sliceDiff += result->energy_dif[block_i];
            block_i++;
        }
    }
    //av_freep(block_buffer);
    //av_freep(out_buffer);
    *partial_sum_E = sliceTexture;
    *partial_sum_h = sliceDiff;
    return  sliceTexture;
}

static uint32_t calc_evca_8_slice(int stride, uint8_t *src, VCAPlaneInfo *plane, VCAResults *result, int enable_lowpass, 
                                     int is_first_frame, int slice_start, int slice_end, uint32_t *partial_sum_E, double *partial_sum_h,
                                     void (*perform_dct)(const int16_t* block, int16_t* dst, int bit_depth)) 
{
    int block_i = (slice_start / 8) * plane->w_blocks;
    uint32_t sliceTexture = 0, energy;
    double sliceDiff = 0, energy_diff;

    ALIGN_VAR_32(int16_t, block_buffer[8 * 8]);
    ALIGN_VAR_32(int16_t, out_buffer[8 * 8]);

    const unsigned bit_depth = plane->bit_depth;
    if (bit_depth != 8 && bit_depth != 10 && bit_depth != 12)
        return AVERROR(AVERROR_INVALIDDATA);

    for (unsigned blockY = slice_start; blockY < slice_end; blockY += 8) { 
        int padding_b = FFMAX(((int)(blockY + 8) - (int)(plane->h_pxls_src)), 0);
        for (unsigned blockX = 0; blockX < plane->w_pxls; blockX += 8){
            int offset = blockX * plane->pxl_depth + (blockY * stride);
            int padding_r = FFMAX((int)(blockX + 8) - (int)(plane->w_pxls_src), 0);

            // Copy values to block buffer 
            ff_copy_vals_buffer(plane->pxl_depth, offset, 8, src, stride, block_buffer, padding_r, padding_b);

            perform_dct(block_buffer, out_buffer, bit_depth);

            ff_calc_weighted_coeff_w_diff(8, out_buffer, result->energy_weight_pxl, result->energy_weight_pxl_prev,
                                          block_i*8*8, enable_lowpass, is_first_frame, &energy, &energy_diff);
    
            result->energy[block_i] = energy;
            result->energy_dif[block_i] = energy_diff;
            
            sliceTexture += result->energy[block_i];
            sliceDiff += result->energy_dif[block_i];
            block_i++;
        }
    }
    //av_freep(block_buffer);
    //av_freep(out_buffer);
    *partial_sum_E = sliceTexture;
    *partial_sum_h = sliceDiff;
    return  sliceTexture;
}

static int calc_evca_filter_slice(AVFilterContext *ctx, void *arg, int job, int nb_jobs)
{
    ThreadDataEVCA *th = arg;

    int block_row_start = (th->plane->h_blocks * job)     / nb_jobs;
    int block_row_end   = (th->plane->h_blocks * (job+1)) / nb_jobs;

    int slice_start = block_row_start * th->blocksize;
    int slice_end   = block_row_end   * th->blocksize;

    switch (th->blocksize) {
        case 32:
            calc_evca_32_slice(th->stride, th->src, th->plane, th->result, th->enable_lowpass, th->is_first_frame,
                               slice_start, slice_end, &th->partial_sums_E[job], &th->partial_sums_h[job], th->perform_dct);
            break;
        case 16:
            calc_evca_16_slice(th->stride, th->src, th->plane, th->result, th->enable_lowpass, th->is_first_frame,
                               slice_start, slice_end, &th->partial_sums_E[job], &th->partial_sums_h[job], th->perform_dct);
            break;
        case 8:
            calc_evca_8_slice(th->stride, th->src, th->plane, th->result, th->enable_lowpass, th->is_first_frame, 
                              slice_start, slice_end, &th->partial_sums_E[job], &th->partial_sums_h[job], th->perform_dct);
            break;
        default:
            return AVERROR(AVERROR_INVALIDDATA);
    }
    return 0;
}

void ff_perform_evca(AVFilterContext *ctx, AVFilterLink *inlink, AVFrame *in, FilterLink *inl,
                     VCAContext *v, int plane_i, double* h, uint32_t* E)
{
    //calc_energy(ctx, in->linesize[plane_i], in->data[plane_i], v->vca_plane[plane_i], v->vca_result[plane_i], v->blocksize, v->enable_lowpass, v->perform_dct);
    // On first frame instead of calculating difference assign difference to 0
    
    uint32_t frameTexture = 0;
    double energyDifference = 0;
    int is_first_frame = inl->frame_count_out == 0;

    int stride = in->linesize[plane_i] / v->vca_plane[plane_i]->pxl_depth;

    int nb_threads = ff_filter_get_nb_threads(ctx);

    ThreadDataEVCA th = {
        .stride = stride,
        .blocksize = v->blocksize,
        .enable_lowpass = v->enable_lowpass,
        .is_first_frame = is_first_frame,
        .src = in->data[plane_i],
        .plane = v->vca_plane[plane_i],
        .result = v->vca_result[plane_i],
        .partial_sums_E = av_calloc(nb_threads, sizeof(uint32_t)),
        .partial_sums_h = av_calloc(nb_threads, sizeof(double)),
        .perform_dct = v->perform_dct
    };

    ff_filter_execute(ctx, calc_evca_filter_slice, &th, NULL, FFMIN(v->vca_plane[plane_i]->h_blocks, nb_threads));

    for (int i = 0; i < nb_threads; i++)
        frameTexture += th.partial_sums_E[i];

    for (int i = 0; i < nb_threads; i++)
        energyDifference += th.partial_sums_h[i];

    av_free(th.partial_sums_E);
    av_free(th.partial_sums_h);

    E[plane_i] = (uint32_t)((double)frameTexture /(v->vca_plane[plane_i]->n_blocks * E_norm_factor));
    h[plane_i] = energyDifference /(v->vca_plane[plane_i]->n_blocks * h_norm_factor);


    // At the end copy current energy to the previous
    memcpy(v->vca_result[plane_i]->energy_weight_pxl_prev, v->vca_result[plane_i]->energy_weight_pxl,
           v->vca_plane[plane_i]->n_blocks * v->blocksize * v->blocksize * sizeof(uint32_t));

    if (v->summary) {
        v->vca_result[plane_i]->min_E  = v->n_frames_processed == 0 ? E[plane_i] : FFMIN(E[plane_i], v->vca_result[plane_i]->min_E);
        v->vca_result[plane_i]->min_h  = v->n_frames_processed == 0 ? h[plane_i] : FFMIN(h[plane_i], v->vca_result[plane_i]->min_h);
        
        v->vca_result[plane_i]->max_E = FFMAX(E[plane_i], v->vca_result[plane_i]->max_E);
        v->vca_result[plane_i]->max_h = FFMAX(h[plane_i], v->vca_result[plane_i]->max_h);

        v->vca_result[plane_i]->energy_frames[inl->frame_count_out] = E[plane_i];
        v->vca_result[plane_i]->energy_dif_frames[inl->frame_count_out] = h[plane_i];
    }
}