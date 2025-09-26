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

 #include "libavutil/stereo3d.h"


#include "vca_dct.h"
#include "vca_svca.h"

enum VCAStereoView {
    LEFT,       // 
    RIGHT,      // 
};

static void unpack_sidebyside(VCAPlaneInfo *plane, uint8_t *data, uint8_t *left_d, uint8_t *right_d, int width, int height, int stride)
{
    const int half_width = width / 2;

    for (int i = 0; i < height; i++) {
        memcpy(&left_d[i * half_width], &data[i * stride], half_width * sizeof(uint8_t));
        memcpy(&right_d[i * half_width], &data[half_width + i * stride], half_width * sizeof(uint8_t));
    }
    
    plane->w_pxls_src = half_width;
}

static void unpack_topbottom(VCAPlaneInfo *plane, uint8_t *data, uint8_t *left_d, uint8_t *right_d, int width, int height, int stride)
{
    for (int i = 0; i < height / 2; i++) 
        memcpy(&left_d[i * width], &data[i * stride], width * sizeof(uint8_t));
    for (int i = height / 2; i < height; i++) 
        memcpy(&right_d[i * width], &data[i * stride], width * sizeof(uint8_t));

    plane->h_pxls_src = height / 2;
}


static void unpack_framesequence(uint8_t *data, uint8_t *left_d, uint8_t *right_d, int width, int height, int stride, int poc)
{
    if(poc % 2 == 0)
        for (int i = 0; i < height; i++) 
            memcpy(&left_d[i * width], &data[i * stride], width * sizeof(uint8_t));
    else
        for (int i = 0; i < height; i++) 
            memcpy(&right_d[i * width], &data[i * stride], width * sizeof(uint8_t));
}

static void unpack_checkerboard(VCAPlaneInfo *plane, uint8_t *data, uint8_t *left_d, uint8_t *right_d, int width, int height, int stride)
{
    int index = 0; 

    for (int i = 0; i < height; i++)
        for (int j = 0; j < width; j++){
            uint8_t pxl = data[j + stride * i];

            if ((i + j) % 2 == 0) {
                left_d[index / 2] = pxl;
            } else {
                right_d[index / 2] = pxl;
            }
            index++;

        }
    plane->w_pxls_src = height / 2;
}

static void unpack_lines(VCAPlaneInfo *plane, uint8_t* data, uint8_t* left_d, uint8_t* right_d, int width, int height, int stride)
{
    for (int i = 0; i < height; i++) 
        if(i % 2 == 0)
            memcpy(&left_d[i * width], &data[i * stride], width * sizeof(uint8_t));
        else
            memcpy(&right_d[i * width], &data[i * stride], width * sizeof(uint8_t));

    plane->h_pxls_src = height / 2;
}

static void unpack_collums(VCAPlaneInfo *plane, uint8_t* data, uint8_t* left_d, uint8_t* right_d, int width, int height, int stride)
{
    int index = 0;
    for (int i = 0; i < height; i++)
        for (int j = 0; j < width; j++){
            if(index % 2 == 0)
                left_d[index / 2] = data[j + stride * i];
            else
                right_d[index / 2] = data[j + stride * i];

            index++;
        }

    plane->w_pxls_src = width / 2;
}

static int init_adjust_plane_info(VCAPlaneInfo *plane, VCAResults *result, int blocksize)
{
    plane->w_blocks = (plane->w_pxls_src + blocksize - 1) / blocksize;
    plane->h_blocks = (plane->h_pxls_src + blocksize - 1) / blocksize;

    plane->n_blocks = plane->w_blocks * plane->h_blocks;    
                
    plane->w_pxls = plane->w_blocks * blocksize;
    plane->h_pxls = plane->h_blocks * blocksize;

    // Free previous buffers in case they are allocated already
    //av_freep(&result->energy_prev);
    av_freep(&result->energy_dif);
    av_freep(&result->energy);

    
    if (result->energy_prev_stereo) {
        av_freep(&result->energy_prev_stereo[LEFT]);
        av_freep(&result->energy_prev_stereo[RIGHT]);   
        av_freep(&result->energy_prev_stereo);
    }
    
    result->energy = av_malloc(plane->n_blocks * sizeof(uint32_t));
    result->energy_dif = av_malloc(plane->n_blocks * sizeof(double)); 

    result->energy_prev_stereo = av_malloc(2 * sizeof(*result->energy_prev_stereo));
    if (result->energy_prev_stereo) {
        result->energy_prev_stereo[LEFT] =
            av_malloc(plane->n_blocks * sizeof(**result->energy_prev_stereo));
        result->energy_prev_stereo[RIGHT] =
            av_malloc(plane->n_blocks * sizeof(**result->energy_prev_stereo));
    }
            
    if (!result->energy || !result->energy_prev_stereo[LEFT] ||
        !result->energy_prev_stereo[RIGHT] || !result->energy_dif)
        return AVERROR(ENOMEM);
    
    return 0;
} 

static int adjust_plane_info(VCAPlaneInfo *plane, VCAResults *result, int blocksize)
{
    plane->w_blocks = (plane->w_pxls_src + blocksize - 1) / blocksize;
    plane->h_blocks = (plane->h_pxls_src + blocksize - 1) / blocksize;

    plane->n_blocks = plane->w_blocks * plane->h_blocks;    
                
    plane->w_pxls = plane->w_blocks * blocksize;
    plane->h_pxls = plane->h_blocks * blocksize;

    // Free previous buffers in case they are allocated already
    //av_freep(&result->energy_prev);
    av_freep(&result->energy_dif);
    av_freep(&result->energy);
    
    result->energy = av_malloc(plane->n_blocks * sizeof(uint32_t));
    result->energy_dif = av_malloc(plane->n_blocks * sizeof(double)); 
            
    if (!result->energy || !result->energy_dif)
        return AVERROR(ENOMEM);
    
    return 0;
} 


static int unpack_stereo3d(AVFrame *in, const AVStereo3D *stereo, VCAContext *v, VCAPlaneInfo *plane, VCAResults *result,
                           uint8_t *left_d, uint8_t *right_d, int plane_i, int width, int height) 
{
    switch(stereo->view) {
        case AV_STEREO3D_VIEW_PACKED:
            break;
        case AV_STEREO3D_VIEW_LEFT:
            //av_log(ctx, AV_LOG_ERROR, "Only left view avaliable.\n");
            return -1;
        case AV_STEREO3D_VIEW_RIGHT:
            //av_log(ctx, AV_LOG_ERROR, "Only right view avaliable.\n");
            return -1;
        case AV_STEREO3D_VIEW_UNSPEC:
            //av_log(ctx, AV_LOG_ERROR, "Unspecified packing.\n");
            return -1;
    }

    uint8_t* src = in->data[plane_i];
    int stride = in->linesize[plane_i];
    
    switch(stereo->type) {
        case AV_STEREO3D_2D:
            //av_log(ctx, AV_LOG_ERROR, "Video is not stereoscopic.\n");
            return -1;
        case AV_STEREO3D_SIDEBYSIDE:
            unpack_sidebyside(plane, src, left_d, right_d, width, height, stride);
            break;
        case AV_STEREO3D_TOPBOTTOM:
            unpack_topbottom(plane, src, left_d, right_d, width, height, stride);
            break;
        case AV_STEREO3D_FRAMESEQUENCE:
            //av_log(ctx, AV_LOG_ERROR, "Framesequence packing is not avaliable.\n");
            return -1;
        case AV_STEREO3D_CHECKERBOARD:
            unpack_checkerboard(plane, src, left_d, right_d, width, height, stride);
            break;        
        case AV_STEREO3D_SIDEBYSIDE_QUINCUNX:
            unpack_sidebyside(plane, src, left_d, right_d, width, height, stride);
            break;
        case AV_STEREO3D_LINES:
            unpack_lines(plane, src, left_d, right_d, width, height, stride);
            break;
        case AV_STEREO3D_COLUMNS:
            unpack_collums(plane, src, left_d, right_d, width, height, stride);
            break;
        case AV_STEREO3D_UNSPEC:
            //av_log(ctx, AV_LOG_ERROR, "Unspecified packing.\n");
            return -1;
    }

    /*
    switch(stereo->primary_eye) {
        case AV_PRIMARY_EYE_NONE:
        case AV_PRIMARY_EYE_LEFT:
        case AV_PRIMARY_EYE_RIGHT:
    }
    */
    return 0;
}

static uint32_t calc_energy_32_slice(int stride, uint8_t *src, VCAPlaneInfo *plane, VCAResults *result, 
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

static uint32_t calc_energy_16_slice(int stride, uint8_t *src, VCAPlaneInfo *plane, VCAResults *result, 
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

static uint32_t calc_energy_8_slice(int stride, uint8_t *src, VCAPlaneInfo *plane, VCAResults *result, 
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
    ThreadDataSVCA *th = arg;

    int block_row_start = (th->plane->h_blocks * job)     / nb_jobs;
    int block_row_end   = (th->plane->h_blocks * (job+1)) / nb_jobs;

    int slice_start = block_row_start * th->blocksize;
    int slice_end   = block_row_end   * th->blocksize;

    switch (th->blocksize) {
        case 32:
            calc_energy_32_slice(th->stride, th->src, th->plane, th->result, th->enable_lowpass,
                                 slice_start, slice_end, &th->partial_sums[job], th->perform_dct);
            break;
        case 16:
            calc_energy_16_slice(th->stride, th->src, th->plane, th->result, th->enable_lowpass, 
                                 slice_start, slice_end, &th->partial_sums[job], th->perform_dct);
            break;
        case 8:
            calc_energy_8_slice(th->stride, th->src, th->plane, th->result, th->enable_lowpass, 
                                slice_start, slice_end, &th->partial_sums[job], th->perform_dct);
            break;
        default:
            return AVERROR(AVERROR_INVALIDDATA);
    }
    return 0;
}


static uint32_t calc_energy(AVFilterContext *ctx, int linesize, uint8_t *src, VCAPlaneInfo *plane, VCAResults *result, int blocksize, int enable_lowpass, void* perform_dct)
{
    uint32_t frameTexture = 0;
    int stride = linesize / plane->pxl_depth;

    int nb_threads = ff_filter_get_nb_threads(ctx);

    ThreadDataSVCA th = {
        .stride = stride,
        .blocksize = blocksize,
        .enable_lowpass = enable_lowpass,
        .src = src,
        .plane = plane,
        .result = result,
        .partial_sums = av_calloc(nb_threads, sizeof(uint32_t)),
        .perform_dct = perform_dct
    };

    ff_filter_execute(ctx, calc_energy_filter_slice, &th, NULL, FFMIN(plane->h_blocks, nb_threads));

    for (int i = 0; i < nb_threads; i++)
        frameTexture += th.partial_sums[i];

    av_free(th.partial_sums);

    return (uint32_t)((double)frameTexture /(plane->n_blocks * E_norm_factor));
}

static double calc_energy_diff(VCAPlaneInfo *plane, VCAResults *result, int view)
{
    int block_i = 0u;
    double diff_sum = 0;

    for (; block_i < plane->n_blocks; block_i++) {
        result->energy_dif[block_i] = abs((int)result->energy[block_i] - (int)result->energy_prev_stereo[view][block_i]);
        diff_sum += result->energy_dif[block_i];
    }
    return  diff_sum /(plane->n_blocks * h_norm_factor); 
}

static double calc_lr_energy_diff(VCAPlaneInfo *plane, VCAResults *result)
{
    int block_i = 0u;
    double diff_sum = 0;

    for (; block_i < plane->n_blocks; block_i++) {
        result->energy_dif[block_i] = abs((int)result->energy_prev_stereo[LEFT][block_i] - (int)result->energy_prev_stereo[RIGHT][block_i]);
        diff_sum += result->energy_dif[block_i];
    }
    return  diff_sum /(plane->n_blocks * h_norm_factor); 
}

static void perform_svca_view(AVFilterContext *ctx, FilterLink *inl, VCAContext *v, uint8_t *data,
                              VCAPlaneInfo *plane, VCAResults *result, int view, uint32_t *E, double *h)
{
    *E = calc_energy(ctx, plane->w_pxls_src, data, plane, result, v->blocksize, v->enable_lowpass, v->perform_dct);
    // On first frame instead of calculating difference assign difference to 0
    if (inl->frame_count_out != 0)
        *h = calc_energy_diff(plane, result, view);
    else
        *h = 0;
    
    // At the end copy current energy to the previous
    // memcpy(result->energy_prev_stereo[view], result->energy, plane->n_blocks * sizeof(uint32_t));

     memcpy(result->energy_prev_stereo[view], result->energy, plane->n_blocks * sizeof(uint32_t));

}

void ff_perform_svca(AVFilterContext *ctx, AVFilterLink *inlink, AVFrame *in, FilterLink *inl,
                     VCAContext *v, int plane_i)
{
    uint32_t E_l, E_r = 0;
    double h_l, h_r, s = 0;

    //av_log(ctx, AV_LOG_ERROR,
    //   "v->sd=%p\n",
    //   v->sd);

    if(v->n_frames_processed == 0){
        AVFrameSideData* sd = av_frame_get_side_data(in, AV_FRAME_DATA_STEREO3D);
        memcpy(v->sd, sd, sizeof(AVFrameSideData));
    }
        
    if (!v) {
        av_log(ctx, AV_LOG_ERROR, "No VCAContext loaded\n");
        return;
    }

    if (!v->plane[plane_i]) {
        av_log(ctx, AV_LOG_ERROR, "No VCAContext plane loaded\n");
        return;
    }

    int width = v->plane[plane_i]->w_pxls_src; 
    int height = v->plane[plane_i]->h_pxls_src; 
    int plxs = width * height;

    if(plxs %2 != 0)
        return;

    uint8_t* l_src =  av_malloc(plxs * sizeof(uint8_t));
    uint8_t* r_src = av_malloc(plxs * sizeof(uint8_t)); 
    
    // We need to avoid original values being overriden
    VCAPlaneInfo *pln_copy = av_mallocz(sizeof(VCAPlaneInfo));
    memcpy(pln_copy,  v->plane[plane_i], sizeof(VCAPlaneInfo));

    //VCAResults *res_copy = av_mallocz(sizeof(VCAResults));
    //memcpy(res_copy,  v->result[plane_i], sizeof(VCAResults));

    if(!v->sd)
        return;

    if(v->sd->type == AV_FRAME_DATA_STEREO3D){
        const AVStereo3D *stereo = (const AVStereo3D *)v->sd->data;
        int ret = unpack_stereo3d(in, stereo, v, pln_copy, v->result[plane_i], l_src, r_src, plane_i, width, height);

        if(v->n_frames_processed == 0)
            init_adjust_plane_info(pln_copy, v->result[plane_i], v->blocksize);
        else
            adjust_plane_info(pln_copy, v->result[plane_i], v->blocksize);

        if (ret != 0) {
            av_log(ctx, AV_LOG_ERROR, "Error unpacking stereographic video:ERROR_CODE");
            return;
        }
    }
    else{
        av_log(ctx, AV_LOG_ERROR, "No stereo data detected");
        return; 
    }
        
    perform_svca_view(ctx, inl, v, l_src, pln_copy, v->result[plane_i], LEFT, &E_l, &h_l);
    perform_svca_view(ctx, inl, v, r_src, pln_copy, v->result[plane_i], RIGHT, &E_r, &h_r);

    s = calc_lr_energy_diff(pln_copy, v->result[plane_i]);

    // Dump info;
    v->print(ctx, AV_LOG_INFO,
        "%4"PRId64,
        inl->frame_count_out);
    v->print(ctx, AV_LOG_INFO,
            ",%d,%f,%d,%f,%f",
            E_l, h_l, E_r, h_r, s);

    v->print(ctx, AV_LOG_INFO, "\n");

    if (v->summary) {
        v->result[plane_i]->min_E  = v->n_frames_processed == 0 ? E_l : FFMIN(E_l, v->result[plane_i]->min_E);
        v->result[plane_i]->min_h  = v->n_frames_processed == 0 ? h_l : FFMIN(h_l, v->result[plane_i]->min_h);
        
        v->result[plane_i]->max_E = FFMAX(E_l, v->result[plane_i]->max_E);
        v->result[plane_i]->max_h = FFMAX(h_l, v->result[plane_i]->max_h);

        v->result[plane_i]->energy_frames[inl->frame_count_out] = E_l;
        v->result[plane_i]->energy_dif_frames[inl->frame_count_out] = h_l;
    }

    av_freep(&l_src);
    av_freep(&r_src);

    av_freep(&pln_copy);
    //av_freep(&res_copy);
}
