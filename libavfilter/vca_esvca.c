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
#include "vca_esvca.h"

enum VCAStereoView {
    LEFT,       // 
    RIGHT,      // 
};

#define DEFINE_CALC_ENERGY_SLICE_PACKING(BLOCKSIZE, PACKING, BLOCK_UNPACK, VIEW)                \
static uint32_t calc_energy_##BLOCKSIZE##_##PACKING##_slice(int stride, uint8_t *src,           \
                             VCAPlaneInfo *plane, ESVCAAlgoContext *result, int is_first_frame, \
                             int enable_lowpass, int slice_start, int slice_end, uint32_t *partial_sum_E, double *partial_sum_h,\
                             void (*perform_dct)(const int16_t*, int16_t*, int)) {              \
    int block_i = (slice_start / BLOCKSIZE) * plane->w_blocks;                              \
    uint32_t sliceTexture = 0, energy;\
    double sliceDiff = 0, energy_diff;\
    ALIGN_VAR_32(int16_t, block_buffer[BLOCKSIZE * BLOCKSIZE]);                             \
    ALIGN_VAR_32(int16_t, out_buffer[BLOCKSIZE * BLOCKSIZE]);                               \
    for (unsigned blockY = slice_start; blockY < slice_end; blockY += BLOCKSIZE) {          \
        int padding_b = FFMAX(((int)(blockY + BLOCKSIZE) - (int)(plane->h_pxls_src)), 0);   \
        for (unsigned blockX = 0; blockX < plane->w_pxls; blockX += BLOCKSIZE) {            \
            int padding_r = FFMAX((int)(blockX + BLOCKSIZE) - (int)(plane->w_pxls_src), 0); \
            int offset = blockX * plane->pxl_depth + (blockY * stride);                     \
            BLOCK_UNPACK;                                                                   \
            perform_dct(block_buffer, out_buffer, plane->bit_depth);                        \
            ff_calc_weighted_coeff_w_diff(BLOCKSIZE, out_buffer, result->energy_weight_pxl, \
                                          result->energy_weight_pxl_prev_stereo[VIEW],      \
                                          block_i*BLOCKSIZE*BLOCKSIZE, enable_lowpass,      \
                                          is_first_frame, &energy, &energy_diff);           \
            result->energy[block_i] = energy;                                               \
            result->energy_dif[block_i] = energy_diff;                                      \
            sliceTexture += result->energy[block_i];                                        \
            sliceDiff += result->energy_dif[block_i];                                       \
            block_i++;                                                                      \
        }                                                                                   \
    }                                                                                       \
    *partial_sum_E = sliceTexture;                                                          \
    *partial_sum_h = sliceDiff;                                                             \
    return sliceTexture;                                                                    \
}

#define DEFINE_CALC_ENERGY_SLICE(PACKING, BLOCK_UNPACK, VIEW)       \
    DEFINE_CALC_ENERGY_SLICE_PACKING(32, PACKING, BLOCK_UNPACK(32), VIEW) \
    DEFINE_CALC_ENERGY_SLICE_PACKING(16, PACKING, BLOCK_UNPACK(16), VIEW) \
    DEFINE_CALC_ENERGY_SLICE_PACKING(8 , PACKING, BLOCK_UNPACK(8), VIEW) 

#define UNPACK_PLAIN(BLOCKSIZE)                 \
    ff_copy_vals_buffer(plane->pxl_depth, offset, BLOCKSIZE, src, stride, block_buffer, padding_r, padding_b);                      

#define UNPACK_SBS_RIGHT_OFFSET(BLOCKSIZE)      \
    ff_copy_vals_buffer(plane->pxl_depth, offset + plane->w_pxls_src, BLOCKSIZE, src, stride, block_buffer, padding_r, padding_b)   

#define UNPACK_TB_BOTTOM_OFFSET(BLOCKSIZE)      \
    ff_copy_vals_buffer(plane->pxl_depth, offset + stride * plane->h_pxls_src, BLOCKSIZE, src, stride, block_buffer, padding_r, padding_b)

//#define UNPACK_CH_LEFT_OFFSET(BLOCKSIZE)        \
    copy_vals_buffer_chl()
//#define UNPACK_CH_RIGHT_OFFSET(BLOCKSIZE)       \
    copy_vals_buffer_chr()

//#define UNPACK_LINES_TOP_OFFSET(BLOCKSIZE)      \
    copy_vals_buffer_lines()
//#define UNPACK_LINES_BOTTOM_OFFSET(BLOCKSIZE)   \
    copy_vals_buffer_lines()
    
//#define UNPACK_COLUMNS_LEFT_OFFSET(BLOCKSIZE)   \
    copy_vals_buffer_columns()
//#define UNPACK_COLUMNS_RIGHT_OFFSET(BLOCKSIZE)  \
    copy_vals_buffer_columns()


#define DEFINE_CALC_ENERGY_FILTER_SLICE_PACKING(PACKING, VIEW)                      \
    static int calc_energy_filter_##PACKING##_##VIEW##_slice(AVFilterContext *ctx, void *arg, int job, int nb_jobs) {\
        ThreadDataESVCA *th = arg;                                                          \
        int block_row_start = (th->plane->h_blocks * job)     / nb_jobs;                    \
        int block_row_end   = (th->plane->h_blocks * (job+1)) / nb_jobs;                    \
        int slice_start = block_row_start * th->blocksize;                                  \
        int slice_end   = block_row_end   * th->blocksize;                                  \
        switch (th->blocksize) {                                                            \
            case 32:                                                                        \
                calc_energy_32_##PACKING##_##VIEW##_slice(                                  \
                    th->stride, th->src, th->plane, th->algoctx,                            \
                    th->is_first_frame, th->enable_lowpass, slice_start, slice_end,         \
                    &th->partial_sums_E[job], &th->partial_sums_h[job], th->perform_dct);   \
                break;                                                                      \
            case 16:                                                                        \
                calc_energy_16_##PACKING##_##VIEW##_slice(                                  \
                    th->stride, th->src, th->plane, th->algoctx,                            \
                    th->is_first_frame, th->enable_lowpass, slice_start, slice_end,         \
                    &th->partial_sums_E[job], &th->partial_sums_h[job], th->perform_dct);   \
                break;                                                                      \
            case 8:                                                                         \
                calc_energy_8_##PACKING##_##VIEW##_slice(                                   \
                    th->stride, th->src, th->plane, th->algoctx,                            \
                    th->is_first_frame, th->enable_lowpass, slice_start, slice_end,         \
                    &th->partial_sums_E[job], &th->partial_sums_h[job], th->perform_dct);   \
                break;                                                                      \
            default:                                                                        \
                return AVERROR(AVERROR_INVALIDDATA);                                        \
        }                                                                                   \
        return 0;                                                                           \
    }

#define DEFINE_CALC_ENERGY_FILTER_SLICE(PACKING)            \
    DEFINE_CALC_ENERGY_FILTER_SLICE_PACKING(PACKING, left)  \
    DEFINE_CALC_ENERGY_FILTER_SLICE_PACKING(PACKING, right) 

// sbs energy calc marco call
DEFINE_CALC_ENERGY_SLICE(sbs_left, UNPACK_PLAIN, LEFT);
DEFINE_CALC_ENERGY_SLICE(sbs_right, UNPACK_SBS_RIGHT_OFFSET, RIGHT);
DEFINE_CALC_ENERGY_FILTER_SLICE(sbs);

// tb energy calc marco call
DEFINE_CALC_ENERGY_SLICE(tb_left, UNPACK_PLAIN, LEFT);
DEFINE_CALC_ENERGY_SLICE(tb_right, UNPACK_TB_BOTTOM_OFFSET, RIGHT);
DEFINE_CALC_ENERGY_FILTER_SLICE(tb);

// fs energy calc marco call
DEFINE_CALC_ENERGY_SLICE(fs_left, UNPACK_PLAIN, LEFT);
DEFINE_CALC_ENERGY_SLICE(fs_right, UNPACK_PLAIN, RIGHT);
DEFINE_CALC_ENERGY_FILTER_SLICE(fs)


/*
// ch energy calc marco call
DEFINE_CALC_ENERGY_SLICE(ch_left, UNPACK_CH_LEFT_OFFSET);
DEFINE_CALC_ENERGY_SLICE(ch_right, UNPACK_CH_RIGHT_OFFSET);
DEFINE_CALC_ENERGY_FILTER_SLICE(ch);

// lines energy lines marco call
DEFINE_CALC_ENERGY_SLICE(lines_left, UNPACK_CH_RIGHT_OFFSET);
DEFINE_CALC_ENERGY_SLICE(lines_right, UNPACK_CH_RIGHT_OFFSET);
DEFINE_CALC_ENERGY_FILTER_SLICE(lines);

// lines energy  marco call
DEFINE_CALC_ENERGY_SLICE(columns_left, UNPACK_CH_RIGHT_OFFSET);
DEFINE_CALC_ENERGY_SLICE(columns_right, UNPACK_CH_RIGHT_OFFSET);
DEFINE_CALC_ENERGY_FILTER_SLICE(columns);

*/
// fs energy calc marco call


static int reinit_algoctx_over_stereo(ESVCAAlgoContext *result, VCAPlaneInfo *plane, int blocksize){
    //iidnt wth = plane->w_pxls_src;
    //int height = plane->h_pxls_src;

    switch(result->stereo->type) {
        case AV_STEREO3D_2D:
            //av_log(ctx, AV_LOG_ERROR, "Video is not stereoscopic.\n");
            return -1;
        case AV_STEREO3D_SIDEBYSIDE:
            plane->w_pxls_src = plane->w_pxls_src / 2;
            break;
        case AV_STEREO3D_TOPBOTTOM:
            plane->h_pxls_src = plane->h_pxls_src / 2;
            break;
        case AV_STEREO3D_FRAMESEQUENCE:
            // no plane adjument required
            break;
        case AV_STEREO3D_CHECKERBOARD:
            plane->w_pxls_src = plane->w_pxls_src / 2;
            break;        
        case AV_STEREO3D_SIDEBYSIDE_QUINCUNX:
            plane->w_pxls_src = plane->w_pxls_src / 2;
            break;
        case AV_STEREO3D_LINES:
            plane->h_pxls_src = plane->h_pxls_src / 2;
            break;
        case AV_STEREO3D_COLUMNS:
            plane->w_pxls_src = plane->w_pxls_src / 2;
            break;
        case AV_STEREO3D_UNSPEC:
            //av_log(ctx, AV_LOG_ERROR, "Unspecified packing.\n");
            return -1;
    }
    plane->w_blocks = (plane->w_pxls_src + blocksize - 1) / blocksize;
    plane->h_blocks = (plane->h_pxls_src + blocksize - 1) / blocksize;

    plane->n_blocks = plane->w_blocks * plane->h_blocks;    
                
    plane->w_pxls = plane->w_blocks * blocksize;
    plane->h_pxls = plane->h_blocks * blocksize;
   // Free previous buffers in case they are allocated already
    //av_freep(&result->energy_prev);
    av_freep(&result->energy);
    av_freep(&result->energy_dif);
    av_freep(&result->energy_weight_pxl);

    if (result->energy_weight_pxl_prev_stereo) {
        av_freep(&result->energy_weight_pxl_prev_stereo[LEFT]);
        av_freep(&result->energy_weight_pxl_prev_stereo[RIGHT]);   
        av_freep(&result->energy_weight_pxl_prev_stereo);
    }
     
    result->energy = av_malloc(plane->n_blocks * sizeof(uint32_t)); 
    result->energy_dif = av_malloc(plane->n_blocks * sizeof(double)); 
    result->energy_weight_pxl = av_malloc(plane->n_blocks * blocksize * blocksize * sizeof(uint32_t)); 

    result->energy_weight_pxl_prev_stereo = av_malloc(2 * sizeof(*result->energy_weight_pxl_prev_stereo));
    if (result->energy_weight_pxl_prev_stereo) {
        result->energy_weight_pxl_prev_stereo[LEFT] =
            av_malloc(plane->n_blocks * blocksize * blocksize * sizeof(**result->energy_weight_pxl_prev_stereo));
        result->energy_weight_pxl_prev_stereo[RIGHT] =
            av_malloc(plane->n_blocks * blocksize * blocksize * sizeof(**result->energy_weight_pxl_prev_stereo));
    }
            
    if (!result->energy || !result->energy_dif || !result->energy_weight_pxl 
        || !result->energy_weight_pxl_prev_stereo[LEFT] || !result->energy_weight_pxl_prev_stereo[RIGHT])
        return AVERROR(ENOMEM);
    return 0;
}

static int calc_weightdiff_lr_slice(AVFilterContext *ctx, void *arg, int job, int nb_jobs)
{
    ThreadDataESVCA *th = arg;
    double diff_weight_sum = 0;

    int block_row_start = (th->plane->h_blocks * job)     / nb_jobs;
    int block_row_end   = (th->plane->h_blocks * (job+1)) / nb_jobs;

    int slice_start = block_row_start * th->blocksize;
    int slice_end   = block_row_end   * th->blocksize;

    uint32_t* energy_weight_l = th->algoctx->energy_weight_pxl_prev_stereo[LEFT];
    uint32_t* energy_weight_r = th->algoctx->energy_weight_pxl_prev_stereo[RIGHT];


    for (unsigned blockY = slice_start; blockY < slice_end; blockY ++) { 
        for (unsigned blockX = 0; blockX < th->plane->w_pxls; blockX ++) {            
        int offset = blockX + (blockY * th->plane->w_pxls);      
        
        double weight_diff_lr = abs((int)energy_weight_l[offset] - (int)energy_weight_r[offset]);
        diff_weight_sum += weight_diff_lr;                                                               
        }                                                                                   
    }                                     
    th->partial_sums_s[job] = diff_weight_sum;  
    return 0;                                             
}


static void dispatch_esvca(void* calc_energy_left, void* calc_energy_right, AVFilterContext *ctx, ThreadDataESVCA th,
                          int nb_threads, int blocksize, VCAPlaneInfo *plane, ESVCAAlgoContext *result,
                          uint32_t *E_l, double *h_l, uint32_t *E_r, double *h_r, double *s){
    uint32_t frameTexture = 0;
    double energyDifference = 0;
    double viewEnergyDifference = 0; 

    ff_filter_execute(ctx, calc_energy_left, &th, NULL, FFMIN(plane->h_blocks, nb_threads));
    memcpy(result->energy_weight_pxl_prev_stereo[LEFT], 
        result->energy_weight_pxl,
        plane->n_blocks * blocksize *  blocksize * sizeof(uint32_t));
    for (int i = 0; i < nb_threads; i++)
        frameTexture += th.partial_sums_E[i];
    *E_l = (uint32_t)((double)frameTexture /(plane->n_blocks * E_norm_factor));
    for (int i = 0; i < nb_threads; i++)
        energyDifference += th.partial_sums_h[i];
    *h_l = energyDifference /(plane->n_blocks * h_norm_factor); 

    frameTexture = 0;
    energyDifference = 0;

    ff_filter_execute(ctx, calc_energy_right, &th, NULL, FFMIN(plane->h_blocks, nb_threads));
    memcpy(result->energy_weight_pxl_prev_stereo[RIGHT], 
        result->energy_weight_pxl, 
        plane->n_blocks * blocksize *  blocksize * sizeof(uint32_t));
    for (int i = 0; i < nb_threads; i++)
        frameTexture += th.partial_sums_E[i];
    *E_r = (uint32_t)((double)frameTexture /(plane->n_blocks * E_norm_factor));
    for (int i = 0; i < nb_threads; i++)
        energyDifference += th.partial_sums_h[i];
    *h_r = energyDifference /(plane->n_blocks * h_norm_factor); 


    ff_filter_execute(ctx, calc_weightdiff_lr_slice, &th, NULL, FFMIN(plane->h_blocks, nb_threads));
    for (int i = 0; i < nb_threads; i++)
        viewEnergyDifference += th.partial_sums_s[i];
    *s = viewEnergyDifference/(plane->n_blocks * h_norm_factor); 

}

av_cold int ff_init_esvca(VCAAlgoContext *ctx, int n_blocks, int blocksize) {
    ESVCAAlgoContext *esvca = (ESVCAAlgoContext *)ctx;
    esvca->stereo = av_mallocz(sizeof(AVStereo3D));

    av_freep(&esvca->energy);    
    av_freep(&esvca->energy_dif);
    av_freep(&esvca->energy_weight_pxl);

    if (esvca->energy_weight_pxl_prev_stereo) {
        av_freep(&esvca->energy_weight_pxl_prev_stereo[LEFT]);
        av_freep(&esvca->energy_weight_pxl_prev_stereo[RIGHT]);   
        av_freep(&esvca->energy_weight_pxl_prev_stereo);
    }

    esvca->energy_weight_pxl_prev_stereo = av_malloc(2 * sizeof(*esvca->energy_weight_pxl_prev_stereo));
    if (esvca->energy_weight_pxl_prev_stereo) {
        esvca->energy_weight_pxl_prev_stereo[LEFT] =
            av_malloc(n_blocks * blocksize * blocksize * sizeof(**esvca->energy_weight_pxl_prev_stereo));
        esvca->energy_weight_pxl_prev_stereo[RIGHT] =
            av_malloc(n_blocks * blocksize * blocksize * sizeof(**esvca->energy_weight_pxl_prev_stereo));
    }

    esvca->energy = av_malloc(n_blocks * sizeof(uint32_t)); 
    esvca->energy_dif = av_malloc(n_blocks * sizeof(double)); 
    esvca->energy_weight_pxl = av_malloc(n_blocks * blocksize * blocksize * sizeof(uint32_t)); 

    if (!esvca->energy_weight_pxl_prev_stereo || !esvca->energy_weight_pxl_prev_stereo[LEFT] || !esvca->energy_weight_pxl_prev_stereo[RIGHT])
        return AVERROR(ENOMEM);
    if (!esvca->energy_dif || !esvca->energy)
        return AVERROR(ENOMEM);
    return 0;
}

void ff_uninit_esvca(VCAAlgoContext *ctx) {
    ESVCAAlgoContext *esvca = (ESVCAAlgoContext *)ctx;

    av_freep(&esvca->energy);    
    av_freep(&esvca->energy_dif);
    av_freep(&esvca->energy_weight_pxl);

    if (esvca->energy_weight_pxl_prev_stereo) {
        av_freep(&esvca->energy_weight_pxl_prev_stereo[LEFT]);
        av_freep(&esvca->energy_weight_pxl_prev_stereo[RIGHT]);   
        av_freep(&esvca->energy_weight_pxl_prev_stereo);
    }

    av_freep(&esvca->stereo);
}

static void perform_esvca_flats (AVFilterContext *ctx, ThreadDataESVCA th, ESVCAAlgoContext *esvca, 
                VCAContext *v, VCAPlaneInfo *plane, int blocksize, int nb_threads, int frame_n) {
    uint32_t E_l, E_r = 0;
    double h_l, h_r, s = 0;

    switch(esvca->stereo->type) {
        case AV_STEREO3D_2D:
            av_log(ctx, AV_LOG_ERROR, "Video is not stereoscopic.\n");
            return;
        case AV_STEREO3D_SIDEBYSIDE:
            dispatch_esvca(calc_energy_filter_sbs_left_slice, calc_energy_filter_sbs_right_slice, 
                    ctx, th, nb_threads, blocksize, plane, esvca, &E_l, &h_l, &E_r, &h_r, &s);
            break;
        case AV_STEREO3D_TOPBOTTOM:
            dispatch_esvca(calc_energy_filter_tb_left_slice, calc_energy_filter_tb_right_slice, 
                    ctx, th, nb_threads, blocksize, plane, esvca, &E_l, &h_l, &E_r, &h_r, &s);
            break;
/*

        case AV_STEREO3D_CHECKERBOARD:
            dispatch_svca(calc_energy_filter_ch_left_slice, calc_energy_filter_ch_right_slice, 
                    ctx, th, nb_threads, plane, is_first_frame, svca, &E_l, &h_l, &E_r, &h_r, &s);
            break;        
        case AV_STEREO3D_SIDEBYSIDE_QUINCUNX:
            dispatch_svca(calc_energy_filter_sbs_left_slice, calc_energy_filter_sbs_right_slice, 
                    ctx, th, nb_threads, plane, is_first_frame, svca, &E_l, &h_l, &E_r, &h_r, &s);
            break;
        case AV_STEREO3D_LINES:
            dispatch_svca(calc_energy_filter_lines_left_slice, calc_energy_filter_lines_right_slice, 
                    ctx, th, nb_threads, plane, is_first_frame, svca, &E_l, &h_l, &E_r, &h_r, &s);
            break;
        case AV_STEREO3D_COLUMNS:
            dispatch_svca(calc_energy_filter_columns_left_slice, calc_energy_filter_columns_right_slice, 
                    ctx, th, nb_threads, plane, is_first_frame, svca, &E_l, &h_l, &E_r, &h_r, &s);
            break;
*/
            
    }

    av_free(th.partial_sums_E);    
    av_free(th.partial_sums_h);    
    av_free(th.partial_sums_s);    

    // Dump info;
    v->print(ctx, AV_LOG_INFO,
        "%4"PRId64,
        frame_n);
    v->print(ctx, AV_LOG_INFO,
            ",%d,%f,%d,%f,%f",
            E_l, h_l, E_r, h_r, s);

    v->print(ctx, AV_LOG_INFO, "\n");
}

static void perform_esvca_sequential (AVFilterContext *ctx, ThreadDataESVCA th, ESVCAAlgoContext *esvca, 
                VCAContext *v, VCAPlaneInfo *plane, int blocksize, int nb_threads, int frame_n) {
    uint32_t E_l, E_r = 0;
    double h_l, h_r, s = 0;
    uint32_t frameTexture = 0;
    double energyDifference = 0;
    double viewEnergyDifference = 0; 

    if ( frame_n % 2 == 0 ) {
        ff_filter_execute(ctx, calc_energy_filter_fs_left_slice, &th, NULL, FFMIN(plane->h_blocks, nb_threads));
        memcpy(esvca->energy_weight_pxl_prev_stereo[LEFT], 
            esvca->energy_weight_pxl,
            plane->n_blocks * blocksize *  blocksize * sizeof(uint32_t));
        for (int i = 0; i < nb_threads; i++)
            frameTexture += th.partial_sums_E[i];
        E_l = (uint32_t)((double)frameTexture /(plane->n_blocks * E_norm_factor));
        for (int i = 0; i < nb_threads; i++)
            energyDifference += th.partial_sums_h[i];
        h_l = energyDifference /(plane->n_blocks * h_norm_factor); 

        // Dump info;
        v->print(ctx, AV_LOG_INFO,
            "%4"PRId64,
            frame_n);
        v->print(ctx, AV_LOG_INFO,
                ",%d,%f,-,-,-",
                E_l, h_l);

        v->print(ctx, AV_LOG_INFO, "\n");
    } 
    else {
        th.is_first_frame = frame_n == 1;

        ff_filter_execute(ctx, calc_energy_filter_fs_right_slice, &th, NULL, FFMIN(plane->h_blocks, nb_threads));

        memcpy(esvca->energy_weight_pxl_prev_stereo[RIGHT], 
            esvca->energy_weight_pxl, 
            plane->n_blocks * blocksize *  blocksize * sizeof(uint32_t));
        for (int i = 0; i < nb_threads; i++)
            frameTexture += th.partial_sums_E[i];
        E_r = (uint32_t)((double)frameTexture /(plane->n_blocks * E_norm_factor));
        for (int i = 0; i < nb_threads; i++)
            energyDifference += th.partial_sums_h[i];
        h_r = energyDifference /(plane->n_blocks * h_norm_factor); 


        ff_filter_execute(ctx, calc_weightdiff_lr_slice, &th, NULL, FFMIN(plane->h_blocks, nb_threads));
        for (int i = 0; i < nb_threads; i++)
            viewEnergyDifference += th.partial_sums_s[i];
        s = viewEnergyDifference/(plane->n_blocks * h_norm_factor); 

        // Dump info;
        v->print(ctx, AV_LOG_INFO,
            "%4"PRId64,
            frame_n);
        v->print(ctx, AV_LOG_INFO,
                ",-,-,%d,%f,%f",
                E_r, h_r, s);

        v->print(ctx, AV_LOG_INFO, "\n");
    }
}

void ff_perform_esvca(AVFilterContext *ctx, AVFrame *in, FilterLink *inl,
                     VCAContext *v, int plane_i)
{
    ESVCAAlgoContext *esvca = (ESVCAAlgoContext *)v->algoctx[plane_i];
    VCAPlaneInfo *plane = v->plane[plane_i];
    int is_first_frame = v->n_frames_processed == 0 ? 1 : 0;

    if(is_first_frame){
        AVFrameSideData* sd = av_frame_get_side_data(in, AV_FRAME_DATA_STEREO3D);
        if(sd) {
            const AVStereo3D *stereo = (const AVStereo3D *)sd->data;
            memcpy(esvca->stereo, stereo, sizeof(esvca->stereo));
            int ret = reinit_algoctx_over_stereo(esvca, plane, v->blocksize);
            if(ret != 0) {
                av_log(ctx, AV_LOG_ERROR, "Problem with detected stereo file metadata\n");
                return;
            }
        }
        else {
            av_log(ctx, AV_LOG_ERROR, "No Stereo data detected in a file\n");
            return;
        }
    }

    int nb_threads = ff_filter_get_nb_threads(ctx);
    ThreadDataESVCA th = {
        .stride = in->linesize[plane_i],
        .src = in->data[plane_i],
        .blocksize = v->blocksize,
        .enable_lowpass = v->enable_lowpass,
        .is_first_frame = is_first_frame,
        .plane =  plane,
        .algoctx = esvca,
        .partial_sums_E = av_calloc(nb_threads, sizeof(uint32_t)),
        .partial_sums_h = av_calloc(nb_threads, sizeof(double)),
        .partial_sums_s = av_calloc(nb_threads, sizeof(double)),
        .perform_dct = v->perform_dct
    };

    switch(esvca->stereo->type) { 
        case AV_STEREO3D_SIDEBYSIDE:
        case AV_STEREO3D_TOPBOTTOM:
        //case AV_STEREO3D_CHECKERBOARD:
        //case AV_STEREO3D_SIDEBYSIDE_QUINCUNX:
        //case AV_STEREO3D_LINES:
        //case AV_STEREO3D_COLUMNS:
            perform_esvca_flats(ctx, th, esvca, v, plane, v->blocksize, nb_threads, inl->frame_count_out);
            break;
        case AV_STEREO3D_FRAMESEQUENCE:
            perform_esvca_sequential(ctx, th, esvca, v, plane, v->blocksize, nb_threads, inl->frame_count_out);
            break;
        case AV_STEREO3D_2D:
            av_log(ctx, AV_LOG_ERROR, "Video is not stereoscopic.\n");
            return;
        case AV_STEREO3D_UNSPEC:
            av_log(ctx, AV_LOG_ERROR, "Unspecified packing.\n");
            return;
        default:
            av_log(ctx, AV_LOG_ERROR, "Unrecognized or unsuported packing.\n");
            return;
    }
}