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

/**
 * @file
 * Calculate frame scores using Video Complexity Analyzer (VCA)
 */


#include "libavutil/timestamp.h"
#include "libavutil/mathematics.h"

#include "libavutil/avassert.h"
#include "libavutil/imgutils.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"

#include "libavformat/avio.h"

#include "avfilter.h"
#include "filters.h"
#include "formats.h"
#include "video.h"

#include "vca_dct.h"

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

    // results
    double max_h;
    double max_E;
    
    double min_h;
    double min_E;

    uint32_t *energy_frames;
    double *energy_dif_frames;
} VCAResults;

typedef struct ThreadData {
    int stride;
    int blocksize;

    int enable_lowpass;

    uint8_t *src;

    VCAPlaneInfo *plane;
    VCAResults *result;
    
    uint32_t *partial_sums;
} ThreadData;


typedef struct VCAContext {
    const AVClass *class;    
    AVIOContext *avio_context;
    void (*print)(AVFilterContext *ctx, int lvl, const char *msg, ...); // av_printf_format(2, 3);

    // options 
    unsigned blocksize;
    int enable_lowpass;
    int enable_chroma;
    int enable_texture;
    int summary;
    int verbose;
    int yuview;
    int n_frames;
    char *file_str;

    // video frame properties
    VCAPlaneInfo **vca_plane;
    int n_frames_processed;

    // results
    VCAResults **vca_result;
} VCAContext;

#define OFFSET(x) offsetof(VCAContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM

static const AVOption vca_options[] = {
    // Analysis config                                      
    { "blocksize", "Set size of block", OFFSET(blocksize), AV_OPT_TYPE_INT, {.i64=32}, 8, 32, FLAGS },
    { "n", "Set the frames batch size", OFFSET(n_frames), AV_OPT_TYPE_INT, {.i64=500}, 2, INT_MAX, FLAGS },
    // Performance
    { "lowpass", "Enable low-pass DCT", OFFSET(enable_lowpass), AV_OPT_TYPE_BOOL, { .i64=0 }, 0, 1, FLAGS },
    { "texture", "Enable analysis of texture", OFFSET(enable_texture), AV_OPT_TYPE_BOOL, { .i64=0 }, 0, 1, FLAGS },
    { "chroma", "Enable analysis of chroma channels", OFFSET(enable_chroma), AV_OPT_TYPE_BOOL, { .i64=0 }, 0, 1, FLAGS },
    // Output
    { "summary", "Print summary of metrics over whole video", OFFSET(summary), AV_OPT_TYPE_BOOL, { .i64=0 }, 0, 1, FLAGS },
    { "verbose", "Verbose logging option", OFFSET(verbose), AV_OPT_TYPE_BOOL, { .i64=0 }, 0, 1, FLAGS },
    { "file", "Set file where to print analysis information", OFFSET(file_str), AV_OPT_TYPE_STRING, {.str=NULL}, 0, 0, FLAGS },
    //{ "yuview", "Ignore extension detection and force output YUView stats to file", OFFSET(yuview), AV_OPT_TYPE_BOOL, { .i64=0 }, 0, 1, FLAGS },
     
    { NULL }
};

static const double E_norm_factor = 90;
static const double h_norm_factor = 18;

AVFILTER_DEFINE_CLASS(vca);

static const enum AVPixelFormat pxl_fmts[] = {
    AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV422P,
    AV_PIX_FMT_YUVJ420P, AV_PIX_FMT_YUVJ422P,
    AV_PIX_FMT_YUV420P10, AV_PIX_FMT_YUV422P10,
    AV_PIX_FMT_NONE
};

static void copy_vals_wo_padding(unsigned pxl_depth, unsigned blocksize, uint8_t *src, unsigned stride, int16_t *buffer)
{
    if (pxl_depth == 1)
    {
        uint8_t *srcptr = src;
        for (unsigned y = 0; y < blocksize; y++)
            for (unsigned x = 0; x < blocksize; x++)
                *(buffer++) = (int16_t)srcptr[x + stride*y];
    } else {
        uint16_t *srcptr = (uint16_t *) src;
        const unsigned bytes_per_line = blocksize * 2;
        for (unsigned y = 0; y < blocksize; ++y)
        {
            memcpy(buffer, srcptr, blocksize * sizeof(uint16_t));
            srcptr += stride / 2;
            buffer += blocksize;
        }
    }
}

static void copy_vals_w_padding(unsigned pxl_depth, unsigned blocksize, uint8_t *src, unsigned stride, int16_t *buffer, unsigned padding_r, unsigned padding_b)
{
    unsigned y          = 0;
    int16_t *buffer_last_line = buffer;
    
    if (pxl_depth == 1) {
        for (; y < blocksize - padding_b; y++, src += stride) {
            unsigned x     = 0;
            buffer_last_line = buffer;
            for (; x < blocksize - padding_r; x++)
                *(buffer++) = (int16_t)(src[x]);
            const int16_t last = (int16_t)(src[x]);
            for (; x < blocksize; x++)
                *(buffer++) = last;
        }
        for (; y < blocksize; y++) {
            for (unsigned x = 0; x < blocksize; x++)
                *(buffer++) = (buffer_last_line[x]);
        }        
    } else {
        uint16_t *srcptr = (uint16_t*)(src);
        for (; y < blocksize - padding_b; y++) {
            unsigned x     = 0;
            buffer_last_line = buffer;

            const unsigned nr_vals_copy = blocksize - padding_r;
            memcpy(buffer, srcptr, nr_vals_copy * sizeof(uint16_t));

            const uint16_t last = srcptr[nr_vals_copy - 1];
            for (unsigned x = nr_vals_copy; x < blocksize; x++)
                buffer[x] = last;

            buffer += blocksize;
            srcptr += stride / 2;
        }
        for (; y < blocksize; y++) {
            const unsigned nr_bytes_copy = blocksize * 2;
            memcpy(buffer, buffer_last_line, blocksize * sizeof(uint16_t));
            buffer += blocksize;
        }
    }

}

static void copy_vals_buffer(unsigned pxl_depth, unsigned offset, unsigned blocksize, uint8_t *src, unsigned stride, int16_t *buffer, unsigned padding_r, unsigned padding_b)
{
    src += offset;
    if (padding_r == 0 && padding_b == 0)
        copy_vals_wo_padding(pxl_depth, blocksize, src, stride, buffer);
    else
        copy_vals_w_padding(pxl_depth, blocksize, src, stride, buffer, padding_r, padding_b);
}

static uint32_t calc_energy_32_slice(int stride, uint8_t *src, VCAPlaneInfo *plane, VCAResults *result, int enable_lowpass, int slice_start, int slice_end, uint32_t *partial_sum){
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
            copy_vals_buffer(plane->pxl_depth, offset, 32, src, stride, block_buffer, padding_r, padding_b);

            (enable_lowpass ? ff_vca_lowpass_dct32 : ff_vca_dct32)(block_buffer, out_buffer, bit_depth);

            // Calculate energy and brightness
            // result.brightnessPerBlock[blockIndex] = uint32_t(sqrt(coeffBuffer[0]));
            result->energy[block_i] = calc_weighted_coeff(32, out_buffer, enable_lowpass);
            
            sliceTexture += result->energy[block_i];
            block_i++;
        }
    }
    //av_freep(block_buffer);
    //av_freep(out_buffer);
    *partial_sum = sliceTexture;
    return  sliceTexture;
}

static uint32_t calc_energy_16_slice(int stride, uint8_t *src, VCAPlaneInfo *plane, VCAResults *result, int enable_lowpass, int slice_start, int slice_end, uint32_t *partial_sum){
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

            copy_vals_buffer(plane->pxl_depth, offset, 16, src, stride, block_buffer, padding_r, padding_b);

            (enable_lowpass ? ff_vca_lowpass_dct16 : ff_vca_dct16)(block_buffer, out_buffer, bit_depth);

            result->energy[block_i] = calc_weighted_coeff(16, out_buffer, enable_lowpass);
            
            sliceTexture += result->energy[block_i];
            block_i++;
        }
    }
    *partial_sum = sliceTexture;
    return sliceTexture;
}

static uint32_t calc_energy_8_slice(int stride, uint8_t *src, VCAPlaneInfo *plane, VCAResults *result, int enable_lowpass, int slice_start, int slice_end, uint32_t *partial_sum){
    int block_i = (slice_start / 8) * plane->w_blocks;
    uint32_t sliceTexture = 0;

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

            copy_vals_buffer(plane->pxl_depth, offset, 8, src, stride, block_buffer, padding_r, padding_b);

            (enable_lowpass ? ff_vca_lowpass_dct8 : ff_vca_dct8)(block_buffer, out_buffer, bit_depth);

            result->energy[block_i] = calc_weighted_coeff(8, out_buffer, enable_lowpass);
            
            sliceTexture += result->energy[block_i];
            block_i++;
        }
    }
    *partial_sum = sliceTexture;
    return sliceTexture;
}

static int calc_energy_filter_slice(AVFilterContext *ctx, void *arg, int job, int nb_jobs){
    ThreadData *th = arg;
    uint32_t energy = 0;


    int block_row_start = (th->plane->h_blocks * job)     / nb_jobs;
    int block_row_end   = (th->plane->h_blocks * (job+1)) / nb_jobs;

    int slice_start = block_row_start * th->blocksize;
    int slice_end   = block_row_end   * th->blocksize;


    switch (th->blocksize) {
        case 32:
            energy = calc_energy_32_slice(th->stride, th->src, th->plane, th->result, th->enable_lowpass, slice_start, slice_end, &th->partial_sums[job]);
            break;
        case 16:
            energy = calc_energy_16_slice(th->stride, th->src, th->plane, th->result, th->enable_lowpass, slice_start, slice_end, &th->partial_sums[job]);
            break;
        case 8:
            energy = calc_energy_8_slice(th->stride, th->src, th->plane, th->result, th->enable_lowpass, slice_start, slice_end, &th->partial_sums[job]);
            break;
        default:
            return AVERROR(AVERROR_INVALIDDATA);
    }

    return 0;
}


static uint32_t calc_energy(AVFilterContext *ctx, int blocksize, int linesize, uint8_t *src, VCAPlaneInfo *plane, VCAResults *result, int enable_lowpass){
    uint32_t frameTexture = 0;
    int stride = linesize / plane->pxl_depth;

    int nb_threads = ff_filter_get_nb_threads(ctx);

    ThreadData th = {
        .stride = stride,
        .blocksize = blocksize,
        .enable_lowpass = enable_lowpass,
        .src = src,
        .plane = plane,
        .result = result,
        .partial_sums = av_calloc(nb_threads, sizeof(uint32_t)),
    };

    ff_filter_execute(ctx, calc_energy_filter_slice, &th, NULL, FFMIN(plane->h_blocks, nb_threads));

    for (int i = 0; i < nb_threads; i++)
        frameTexture += th.partial_sums[i];

    av_free(th.partial_sums);

    return (uint32_t)((double)frameTexture /(plane->n_blocks * E_norm_factor));
}

static double calc_energy_diff(VCAPlaneInfo *plane, VCAResults *result){
    int blockIndex = 0u;
    double energyDifference = 0;

    for (; blockIndex < plane->n_blocks; blockIndex++) {
        result->energy_dif[blockIndex] = abs((int)result->energy[blockIndex] - (int)result->energy_prev[blockIndex]);
        energyDifference += result->energy_dif[blockIndex];
    }
    return  energyDifference /(plane->n_blocks * h_norm_factor); 
}

static void print_log(AVFilterContext *ctx, int lvl, const char *msg, ...)
{
    va_list argument_list;

    va_start(argument_list, msg);
    if (msg)
        av_vlog(ctx, lvl, msg, argument_list);
    va_end(argument_list);
}

static void print_file(AVFilterContext *ctx, int lvl, const char *msg, ...)
{
    VCAContext *v = ctx->priv;
    va_list argument_list;

    va_start(argument_list, msg);
    if (msg) {
        char buf[128];
        int ret = vsnprintf(buf, sizeof(buf), msg, argument_list);
        avio_write(v->avio_context, buf, ret);
    }
    va_end(argument_list);
}

static void perform_vca(AVFilterContext *ctx, AVFilterLink *inlink, AVFrame *in, FilterLink *inl , VCAContext *v, int plane_i, double* h, uint32_t* E){

    E[plane_i] = calc_energy(ctx, v->blocksize, in->linesize[plane_i], in->data[plane_i], v->vca_plane[plane_i], v->vca_result[plane_i], v->enable_lowpass);
    // On first frame instead of calculating difference assign difference to NaN
    if (inl->frame_count_out != 0) {
        h[plane_i] = calc_energy_diff(v->vca_plane[plane_i], v->vca_result[plane_i]);
    } else {
        h[plane_i] = 0;
    }
    // At the end copy current energy to the previous
    memcpy(v->vca_result[plane_i]->energy_prev ,v->vca_result[plane_i]->energy, v->vca_plane[plane_i]->n_blocks * sizeof(uint32_t));

    if (v->summary) {
        v->vca_result[plane_i]->min_E  = v->n_frames_processed == 0 ? E[plane_i] : FFMIN(E[plane_i], v->vca_result[plane_i]->min_E);
        v->vca_result[plane_i]->min_h  = v->n_frames_processed == 0 ? h[plane_i] : FFMIN(h[plane_i], v->vca_result[plane_i]->min_h);
        
        v->vca_result[plane_i]->max_E = FFMAX(E[plane_i], v->vca_result[plane_i]->max_E);
        v->vca_result[plane_i]->max_h = FFMAX(h[plane_i], v->vca_result[plane_i]->max_h);

        //v->vca_result[plane_i]->max_E = fmaxf(E[plane_i], v->vca_result[plane_i]->max_E);
        //v->vca_result[plane_i]->max_h = fmaxf(h[plane_i], v->vca_result[plane_i]->max_h);

        v->vca_result[plane_i]->energy_frames[inl->frame_count_out] = E[plane_i];
        v->vca_result[plane_i]->energy_dif_frames[inl->frame_count_out] = h[plane_i];
    }
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    VCAContext *v = ctx->priv;
    FilterLink *inl = ff_filter_link(inlink);
    int planes = v->enable_chroma ? 3 : 1;
 
    if (v->n_frames_processed >= v->n_frames)
        return ff_filter_frame(inlink->dst->outputs[0], in);

    uint32_t E[3];
    double h[3];
    
    for(int i = 0; i < planes; i++)
        perform_vca(ctx, inlink, in, inl, v, i, h, E);

    v->n_frames_processed++;

    // Dump info;
    if (v->verbose) {
        v->print(ctx, AV_LOG_INFO,
           "n:%4"PRId64" pts:%7s pts_time:%-7s duration:%7"PRId64
           " duration_time:%-7s ",
           inl->frame_count_out,
           av_ts2str(in->pts), av_ts2timestr(in->pts, &inlink->time_base),
           in->duration, av_ts2timestr(in->duration, &inlink->time_base));
        v->print(ctx, AV_LOG_INFO,
               "energy:%4"PRId32" energy difference:%06f",
               E[0], h[0]);
        if (v->enable_chroma) {
            v->print(ctx, AV_LOG_INFO,
                "energy U:%4"PRId32" energy difference U:%06f",
                E[1], h[1]);
            v->print(ctx, AV_LOG_INFO,
                "energy V:%4"PRId32" energy difference V:%06f",
                E[2], h[2]);
        }       
    } else {
        v->print(ctx, AV_LOG_INFO,
            "%4"PRId64,
            inl->frame_count_out);
        v->print(ctx, AV_LOG_INFO,
                ",%d,%f",
                E[0], h[0]);
        if (v->enable_chroma) {
            v->print(ctx, AV_LOG_INFO,
                ",%d,%f",
                E[1], h[1]);
            v->print(ctx, AV_LOG_INFO,
                ",%d,%f",
                E[2], h[2]);
        }       
    }

    v->print(ctx, AV_LOG_INFO, "\n");
    return ff_filter_frame(inlink->dst->outputs[0], in);
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    VCAContext *v = ctx->priv;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    int max_pixsteps[4];
    int planes;

    v->vca_plane[0]->w_pxls_src = inlink->w;
    v->vca_plane[0]->h_pxls_src = inlink->h;

    if (v->enable_chroma){
        v->vca_plane[1]->w_pxls_src = AV_CEIL_RSHIFT(inlink->w, desc->log2_chroma_w);
        v->vca_plane[1]->h_pxls_src = AV_CEIL_RSHIFT(inlink->h, desc->log2_chroma_h);

        v->vca_plane[2]->w_pxls_src = AV_CEIL_RSHIFT(inlink->w, desc->log2_chroma_w);
        v->vca_plane[2]->h_pxls_src = AV_CEIL_RSHIFT(inlink->h, desc->log2_chroma_h);

        planes = 3;
    } else 
        planes = 1;

    for(int i = 0; i < planes; i++){
        // Inference of bit depth 
        v->vca_plane[i]->bit_depth = desc->comp[i].depth;
        // Inference of pixel depth
        av_image_fill_max_pixsteps(max_pixsteps, NULL, desc);
        v->vca_plane[i]->pxl_depth = max_pixsteps[i];

        v->vca_plane[i]->w_blocks = (v->vca_plane[i]->w_pxls_src + v->blocksize - 1) / v->blocksize;
        v->vca_plane[i]->h_blocks = (v->vca_plane[i]->h_pxls_src + v->blocksize - 1) / v->blocksize;

        v->vca_plane[i]->n_blocks = v->vca_plane[i]->w_blocks * v->vca_plane[i]->h_blocks;    
                
        v->vca_plane[i]->w_pxls = v->vca_plane[i]->w_blocks * v->blocksize;
        v->vca_plane[i]->h_pxls = v->vca_plane[i]->h_blocks * v->blocksize;

        // Free previous buffers in case they are allocated already
        av_freep(&v->vca_result[i]->energy_prev);
        av_freep(&v->vca_result[i]->energy_dif);
        av_freep(&v->vca_result[i]->energy);

        v->vca_result[i]->energy = av_malloc(v->vca_plane[i]->n_blocks * sizeof(uint32_t));
        v->vca_result[i]->energy_prev = av_malloc(v->vca_plane[i]->n_blocks * sizeof(uint32_t));
        v->vca_result[i]->energy_dif = av_malloc(v->vca_plane[i]->n_blocks * sizeof(double)); 
            
        if (!v->vca_result[i]->energy || ! v->vca_result[i]->energy_prev || !v->vca_result[i]->energy_dif)
            return AVERROR(ENOMEM);
    }

    if (!v->verbose) {
        v->print(ctx, AV_LOG_INFO, "POC,E,h");
        if (v->enable_texture)
            v->print(ctx, AV_LOG_INFO, ",L");
        if (v->enable_chroma)
            v->print(ctx, AV_LOG_INFO, ",EV,hV,EU,hE");
        if (v->enable_chroma && v->enable_texture)
            v->print(ctx, AV_LOG_INFO, ",avgV,avgU");

        v->print(ctx, AV_LOG_INFO, "\n");
    }

    av_log(ctx, AV_LOG_INFO, "threads: %d\n", ff_filter_get_nb_threads(ctx));

    return 0;
}

static av_cold int init(AVFilterContext *ctx)
{
    // User options but no input data
    VCAContext *v = ctx->priv;
    int ret;
    int planes = v->enable_chroma ? 3 : 1;

    // allocate arrays of pointers
    v->vca_result = av_calloc(planes, sizeof(*v->vca_result));
    v->vca_plane  = av_calloc(planes, sizeof(*v->vca_plane));
    if (!v->vca_result || !v->vca_plane)
        return AVERROR(ENOMEM);

    // allocate each plane/result struct
    for (int i = 0; i < planes; i++) {
        v->vca_result[i] = av_mallocz(sizeof(*v->vca_result[i]));
        v->vca_plane[i]  = av_mallocz(sizeof(*v->vca_plane[i]));

        if (!v->vca_result[i] || !v->vca_plane[i]) {
            // clean allocations on error
            for (int j = 0; j <= i; j++) {
                av_freep(&v->vca_result[j]);
                av_freep(&v->vca_plane[j]);
            }
            av_freep(&v->vca_result);
            av_freep(&v->vca_plane);
            return AVERROR(ENOMEM);
        }
    }

    for(int i = 0; i < planes; i++){
        av_freep(&v->vca_result[i]->energy_frames);
        av_freep(&v->vca_result[i]->energy_dif_frames);
        
        v->vca_result[i]->energy_frames = av_malloc(v->n_frames * sizeof(uint32_t));
        v->vca_result[i]->energy_dif_frames = av_malloc(v->n_frames * sizeof(double));

        if (!v->vca_result[i]->energy_frames || !v->vca_result[i]->energy_dif_frames)
            return AVERROR(ENOMEM);


        v->vca_result[i]->max_E = 0;
        v->vca_result[i]->max_h = 0;

        v->n_frames_processed = 0;
    }

    if (v->file_str) {
        v->print = print_file;
    } else {
        v->print = print_log;
    }

    v->avio_context = NULL;
    if (v->file_str) {
        ret = avio_open(&v->avio_context, v->file_str, AVIO_FLAG_WRITE);

        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR, "Could not open %s: %s\n",
                   v->file_str, av_err2str(ret));
            return ret;
        }
    }

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    VCAContext *v = ctx->priv;
    int planes = v->enable_chroma ? 3 : 1;

    for(int plane = 0; plane < planes; plane++){
        if (v->summary) {
            const int n_frames = v->n_frames_processed;

            double sumE = 0;
            double sumh = 0;
            for (int i = 0; i < n_frames; i++) {
                sumE += v->vca_result[plane]->energy_frames[i];
                sumh += v->vca_result[plane]->energy_dif_frames[i];
            }
            double meanE = sumE / (double) n_frames;
            double meanh = sumh / (double) n_frames;

            double sumstdE = 0;
            double sumstdh = 0;
            for (int i = 0; i < n_frames; i++) {
                sumstdE += pow(v->vca_result[plane]->energy_frames[i] - meanh, 2);
                sumstdh += pow(v->vca_result[plane]->energy_dif_frames[i] - meanE, 2);
            }
            double stdevE = sqrt(sumstdE/(double) n_frames);
            double stdevh = sqrt(sumstdh/(double) n_frames);

            v->print(ctx, AV_LOG_INFO,
                "Summary -- Plane: %"PRId64" -- Total frames: %d\n", plane, n_frames);

            v->print(ctx, AV_LOG_INFO,
                "Energy -- Max: %f Min: %f Mean: %f Stdev: %f \n",
                v->vca_result[plane]->max_E, v->vca_result[plane]->min_E, meanE, stdevE);

            v->print(ctx, AV_LOG_INFO,
                "Energy dif -- Max: %f Min: %f Mean: %f Stdev: %f \n",
                v->vca_result[plane]->max_h, v->vca_result[plane]->min_h, meanh, stdevh);
        }

        av_freep(&v->vca_result[plane]->energy_dif);
        av_freep(&v->vca_result[plane]->energy);
        av_freep(&v->vca_result[plane]->energy_prev);
    }

    if (v->avio_context) {
        avio_closep(&v->avio_context);
    }
}

static const AVFilterPad avfilter_vf_vca_inputs[] = {
    {
        .name             = "default",
        .type             = AVMEDIA_TYPE_VIDEO,
        .filter_frame     = filter_frame,
        .config_props     = config_input,
    },
};

const FFFilter ff_vf_vca = {
    .p.name        = "vca",
    .p.description = NULL_IF_CONFIG_SMALL("Perform VCA analysis."),
    .p.priv_class  = &vca_class,
    .p.flags       = AVFILTER_FLAG_SLICE_THREADS,
    .priv_size     = sizeof(VCAContext),
    .init          = init,
    .uninit        = uninit,
    FILTER_PIXFMTS_ARRAY(pxl_fmts),
    FILTER_INPUTS(avfilter_vf_vca_inputs),
    FILTER_OUTPUTS(ff_video_default_filterpad),
};

/*
    for (unsigned blockY = 0; blockY < v->heightInPixels; blockY += v->blocksize){
        int paddingBottom = fmaxf(((blockY + v->blocksize) - frame->height), 0);
        for (unsigned blockX = 0; blockX < v->widthInPixels; blockX += v->blocksize){
            auto paddingRight = fmaxf((blockX +  v->blocksize) - (frame->width), 0);
            auto blockOffsetLumaBytes = blockX * v->pixel_depth + (blockY * srcStride);



            result.brightness[blockIndex] = uint32_t(sqrt(coeffBuffer[0]));
            result.energy[blockIndex]     = calculateWeightedCoeffSum(blockSize,
                                                                              coeffBuffer,
                                                                              enableLowpass);
            frameBrightness += v->brightness[blockIndex];
            frameTexture +=  v->energy[blockIndex];

            blockIndex++;
        }
    }
*/

// TODO: Check formats
/*
static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    const VCAContext *vca = ctx->priv;
    static const enum AVPixelFormat wires_pix_fmts[] = {AV_PIX_FMT_GRAY8, AV_PIX_FMT_NONE};
    static const enum AVPixelFormat canny_pix_fmts[] = {AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV422P, AV_PIX_FMT_YUV444P, AV_PIX_FMT_GBRP, AV_PIX_FMT_GRAY8, AV_PIX_FMT_NONE};
    static const enum AVPixelFormat colormix_pix_fmts[] = {AV_PIX_FMT_GBRP, AV_PIX_FMT_GRAY8, AV_PIX_FMT_NONE};
    const enum AVPixelFormat *pix_fmts = NULL;

    return ff_set_common_formats_from_list2(ctx, cfg_in, cfg_out, pix_fmts);
}


// Check frame's color range and convert to full range if needed
static uint16_t convertFullRange(int factor, uint16_t y)
{
    int shift;
    int limit_upper;
    int full_upper;
    int limit_y;

    // For 8 bits, limited range goes from 16 to 235, for 10 bits the range is multiplied by 4
    shift = 16 * factor;
    limit_upper = 235 * factor - shift;
    full_upper = 256 * factor - 1;
    limit_y = fminf(fmaxf(y - shift, 0), limit_upper);
    return (full_upper * limit_y / limit_upper);
}
*/
    // ALIGN_VAR_32(int16_t, pixelBuffer[32 * 32]); ??? 
    //for(; blockIndex < v->totalNumberBlocks; blockIndex++){
    //}
    /*
            #define CREATE_BLOCK(bts){                                              \
                unsigned y          = 0;                                            \
                uint##bts##_t *src = (uint##bts##_t*)frame->data[0];                \
                src += blockOffsetLumaBytes;                                        \
                int16_t* bufferLastLine = block_buffer;                             \
                for (; y < v->blocksize - paddingBottom; y++, src += srcStrideBytes)\
                {                                                                   \
                    unsigned x     = 0;                                             \
                    bufferLastLine = block_buffer;                                  \
                    for (; x < v->blocksize - paddingRight; x++)                    \
                        *(block_buffer++) = v->fullRange ?                          \
                             src[x] :  convertFullRange(factor, src[x]);            \
                    const auto lastValue = v->fullRange ?                           \
                             src[x] :  convertFullRange(factor, src[x]);            \
                    for (; x < v->blocksize; x++)                                   \
                        *(block_buffer++) = lastValue;                              \
                    }                                                               \
                    for (; y < v->blocksize; y++)                                   \
                    {                                                               \
                        for (unsigned x = 0; x < v->blocksize; x++)                 \
                            *(block_buffer++) = (bufferLastLine[x]);                \
                    }                                                               \
                }
            if (v->pixel_depth == 2) {
                CREATE_BLOCK(16);
            } else {
                CREATE_BLOCK(8);
            }

v->fullRange = is_full_range(in);

// Determine whether the video is in full or limited range. If not defined, assume limited.
static int is_full_range(AVFrame* frame)
{
    // If color range not specified, fallback to pixel format
    if (frame->color_range == AVCOL_RANGE_UNSPECIFIED || frame->color_range == AVCOL_RANGE_NB)
        return frame->format == AV_PIX_FMT_YUVJ420P || frame->format == AV_PIX_FMT_YUVJ422P;
    return frame->color_range == AVCOL_RANGE_JPEG;
}


void dct_init(double *coefficients, unsigned int blocksize)
{
    unsigned int i, j;

    for (j = 0; j < blocksize; ++j) {
        coefficients[j] = sqrt(0.125);
        for (i = blocksize; i < blocksize*blocksize; i += blocksize) {
            double tmp = 0.5 * cos(i * (j + 0.5) * M_PI / (blocksize*blocksize));
            coefficients[i + j] = tmp;
        }
    }
}

// This impleamentation is currently a stand-in based on reference implimentation
void fdct(uint16_t *block, uint16_t *out, double *coefficients, unsigned blocksize)
{
    // implement the equation: block = coefficients * block * coefficients' 

    unsigned int i, j, k;

    // out = av_malloc(blocksize*blocksize*sizeof(int16_t));    

    // out = coefficients * block 
    for (i = blocksize; i < blocksize*blocksize; i += blocksize) {
        for (j = 0; j < blocksize; ++j) {
            double tmp = 0;
            for (k = 0; k < blocksize; ++k) {
                tmp += coefficients[i + k] * block[k * blocksize + j];
            }
            out[i + j] = tmp * blocksize;
        }
    }

    // block = out * (coefficients') 
    for (j = 0; j < blocksize; ++j) {
        for (i = 0; i < blocksize*blocksize; i += blocksize) {
            double tmp = 0;
            for (k = 0; k < blocksize; ++k) {
                tmp += out[i + k] * coefficients[j * blocksize + k];
            }
            block[i + j] = floor(tmp + 0.499999999999);
        }
    }
}
*/

/*
void copyPixelValuesToBufferWithPaddingHighBitDepth(unsigned blockSize,
                                                    uint8_t *srcData,
                                                    unsigned srcStrideBytes,
                                                    int16_t *buffer,
                                                    unsigned paddingRight,
                                                    unsigned paddingBottom)
{

    unsigned y          = 0;
    uint16_t* bufferLastLine = buffer;

}
*/


//typedef struct VCAResult{
//    int nan;
//} VCAResult;
//int srcStride = frame->linesize[0] / v->pixel_depth;
//coef_buffer=av_malloc(v->blocksize*v->blocksize*sizeof(double)); 

//int srcStride = frame->linesize[0] / v->pixel_depth;
//int factor = v->pixel_depth == 1 ? 1 : 4;

// For 8 bits, limited range goes from 16 to 235, for 10 bits the range is multiplied by 4
//int factor = v->pixel_depth == 1 ? 1 : 4;

    //size_t energy_sz_uint_32;
    //size_t energy_sz_dbl;

    //energy_sz_uint_32 = v->n_blocks * sizeof(uint32_t);
    //energy_sz_dbl = v->n_blocks * sizeof(double);


/*


uint32_t calc_energy(int blocksize, int linesize, uint8_t* src, int h_pxls, int w_pxls,  int stride, int pxl_depth){
    int block_i = 0u;
    uint32_t frameTexture = 0;
    int stride = linesize / pxl_depth;

    int16_t* block_buffer;
    int16_t* out_buffer;

    block_buffer=av_malloc(blocksize*blocksize*sizeof(int16_t));    
    out_buffer=av_malloc(blocksize*blocksize*sizeof(int16_t)); 

    const auto widthInPixelsC             = widthInBlocksC * blockSize;
    const auto heightInPixelsC            = heightInBlockC * blockSize;

    //uint8_t* src = frame->data[0];
    // For each block on X and Y
    for (unsigned blockY = 0; blockY < h_pxls; blockY += blocksize){  
        int padding_b = fmaxf(((int)(blockY + v->blocksize) - (int)(frame->height)), 0);
        for (unsigned blockX = 0; blockX < v->w_pxls; blockX += v->blocksize){
            int offset = blockX * v->pxl_depth + (blockY * stride);
            int padding_r = fmaxf((int)(blockX +  v->blocksize) - (int)(frame->width), 0);

            // Copy values to block buffer 
            copy_vals_buffer(v->pxl_depth, offset, v->blocksize, src, stride, block_buffer, padding_r, padding_b);
            // Perform DCTs
            perform_dct(v->bit_depth, v->blocksize, block_buffer, out_buffer, v->enable_lowpass);

            // Calculate energy and brightness
            //result.brightnessPerBlock[blockIndex] = uint32_t(sqrt(coeffBuffer[0]));
            v->energy[block_i] = calc_weighted_coeff(v->blocksize, out_buffer, v->enable_lowpass);
            
            frameTexture += v->energy[block_i];
            block_i++;
        }
    }
    //av_freep(block_buffer);
    //av_freep(out_buffer);
    return  (uint32_t)((double)(frameTexture) / (v->n_blocks * E_norm_factor));
}


    int w_pxls;
    int h_pxls;
    int w_chroma;
    int h_chroma;
    
    int w_blocks;
    int h_blocks;

    int n_blocks;


    // global vars
    uint32_t *energy;
    uint32_t *energy_prev;
    double *energy_dif;

    // results
    double max_h;
    double max_E;
    
    double min_h;
    double min_E;

    uint32_t *energy_frames;
    double *energy_dif_frames;

uint32_t calc_energy(VCAContext *v, AVFrame *frame){
    int block_i = 0u;
    uint32_t frameTexture = 0;
    int stride = frame->linesize[0] / v->pxl_depth;

    int16_t *block_buffer;
    int16_t *out_buffer;

    block_buffer=av_malloc(v->blocksize*v->blocksize*sizeof(int16_t));    
    out_buffer=av_malloc(v->blocksize*v->blocksize*sizeof(int16_t)); 

    uint8_t *src = frame->data[0];
    // For each block on X and Y
    for (unsigned blockY = 0; blockY < v->h_pxls; blockY += v->blocksize) {  
        int padding_b = fmaxf(((int)(blockY + v->blocksize) - (int)(frame->height)), 0);
        for (unsigned blockX = 0; blockX < v->w_pxls; blockX += v->blocksize) {
            int offset = blockX * v->pxl_depth + (blockY * stride);
            int padding_r = fmaxf((int)(blockX +  v->blocksize) - (int)(frame->width), 0);

            // Copy values to block buffer 
            copy_vals_buffer(v->pxl_depth, offset, v->blocksize, src, stride, block_buffer, padding_r, padding_b);
            // Perform DCTs
            perform_dct(v->bit_depth, v->blocksize, block_buffer, out_buffer, v->enable_lowpass);

            // Calculate energy and brightness
            //result.brightnessPerBlock[blockIndex] = uint32_t(sqrt(coeffBuffer[0]));
            v->energy[block_i] = calc_weighted_coeff(v->blocksize, out_buffer, v->enable_lowpass);
            
            frameTexture += v->energy[block_i];
            block_i++;
        }
    }
    //av_freep(block_buffer);
    //av_freep(out_buffer);
    return  (uint32_t)((double)(frameTexture) / (v->n_blocks * E_norm_factor));
}

double calc_energy_diff(VCAContext *v, AVFrame *frame){
    int blockIndex = 0u;
    double energyDifference = 0;

    for (; blockIndex < v->n_blocks; blockIndex++) {
        v->energy_dif[blockIndex] = abs((int)v->energy[blockIndex] - (int)v->energy_prev[blockIndex]);
        energyDifference += v->energy_dif[blockIndex];
    }
    return  energyDifference /(v->n_blocks * h_norm_factor); 
}


*/

/*
static int perform_dct(const unsigned bit_depth, const unsigned blocksize, int16_t *pxl_buffer, int16_t *coeff_buffer, int enable_lowpass)
{
    if (bit_depth != 8 && bit_depth != 10 && bit_depth != 12)
        return AVERROR(AVERROR_INVALIDDATA);

    if(enable_lowpass) {
        switch (blocksize) {
            case 32:
                ff_vca_lowpass_dct32(pxl_buffer, coeff_buffer, bit_depth);
                return 0;
            case 16:
                ff_vca_lowpass_dct16(pxl_buffer, coeff_buffer, bit_depth);
                return 0;
            case 8:
                ff_vca_lowpass_dct8(pxl_buffer, coeff_buffer, bit_depth);
                return 0;
            default:
                return AVERROR(AVERROR_INVALIDDATA);
        }
    }

    switch (blocksize) {
        case 32:
            ff_vca_dct32(pxl_buffer, coeff_buffer, bit_depth);
            return 0;
        case 16:
            ff_vca_dct16(pxl_buffer, coeff_buffer, bit_depth);
            return 0;
        case 8:
            ff_vca_dct8(pxl_buffer, coeff_buffer, bit_depth);
            return 0;
        default:
            return AVERROR(AVERROR_INVALIDDATA);
    }
}
*/
    // perform_dct(plane->bit_depth, blocksize, block_buffer, out_buffer, enable_lowpass);

    //ALIGN_VAR_32(int16_t, block_buffer[32 * 32]);
    //ALIGN_VAR_32(int16_t, out_buffer[32 * 32]);
    //block_buffer=av_malloc(blocksize*blocksize*sizeof(int16_t));    
    //out_buffer=av_malloc(blocksize*blocksize*sizeof(int16_t)); 

    //uint8_t* src = frame->data[0];
    // For each block on X and Y
    /*
        for (unsigned blockY = 0; blockY < plane->h_pxls; blockY += blocksize){  
        int padding_b = fmaxf(((int)(blockY + blocksize) - (int)(plane->w_pxls_src)), 0);
        for (unsigned blockX = 0; blockX < plane->w_pxls; blockX += blocksize){
            int offset = blockX * plane->pxl_depth + (blockY * stride);
            int padding_r = fmaxf((int)(blockX + blocksize) - (int)(plane->h_pxls_src), 0);

            // Copy values to block buffer 
            copy_vals_buffer(plane->pxl_depth, offset, blocksize, src, stride, block_buffer, padding_r, padding_b);
            // Perform DCTs
            perform_dct(plane->bit_depth, blocksize, block_buffer, out_buffer, enable_lowpass);

            // Calculate energy and brightness
            //result.brightnessPerBlock[blockIndex] = uint32_t(sqrt(coeffBuffer[0]));
            result->energy[block_i] = calc_weighted_coeff(blocksize, out_buffer, enable_lowpass);
            
            frameTexture += result->energy[block_i];
            block_i++;
        }
    }
    */

    //av_freep(block_buffer);
    //av_freep(out_buffer);

            //memset(v->vca_result[i]->energy_frames, 0, v->n_frames * sizeof(uint32_t));
        //memset(v->vca_result[i]->energy_frames, 0, v->n_frames * sizeof(double));
        
        //v->vca_result[i]->min_E = 0;
        //v->vca_result[i]->min_h = 0;

            //memcpy(ptr, p, size);
    //v->vca_result[plane_i]->energy_prev = av_memdup(v->vca_result[plane_i]->energy, v->vca_plane[plane_i]->n_blocks * sizeof(uint32_t));


    
    // CSV header
    /*
        if (v->yuview) {
        v->print(ctx, AV_LOG_INFO, "%%;syntax-version;v1.22\n");
        v->print(ctx, AV_LOG_INFO, "%%;%%;Written by VCA for YUView\n");
        v->print(ctx, AV_LOG_INFO, "%%;%%;POC;X-position of the left top pixel in the block;Y-position of the left top pixel in the block;Width of the block;Height of the block; Type-ID;Type specific value\n");
        //v->print(ctx, AV_LOG_INFO, "%;seq-specs;%s;layer0;%d;%d;24\n",inlink->inputs[0]->name,inlink->w,inlink->h);
        v->print(ctx, AV_LOG_INFO, "%%;type;0;BlockBrightness;range\n");
        v->print(ctx, AV_LOG_INFO, "%%;defaultRange;0;300;heat\n");
        v->print(ctx, AV_LOG_INFO, "%%;type;1;BlockEnergy;range\n");
        v->print(ctx, AV_LOG_INFO, "%%;defaultRange;0;10000;heat\n");
        v->print(ctx, AV_LOG_INFO, "%%;type;2;SAD;range\n");
        v->print(ctx, AV_LOG_INFO, "%%;defaultRange;0;3000;heat\n");
    }



    static uint32_t calc_energy_32(int stride, uint8_t *src, VCAPlaneInfo *plane, VCAResults *result, int enable_lowpass){
    int block_i = 0u;
    uint32_t frameTexture = 0;

    ALIGN_VAR_32(int16_t, block_buffer[32 * 32]);
    ALIGN_VAR_32(int16_t, out_buffer[32 * 32]);

    const unsigned bit_depth = plane->bit_depth;
    if (bit_depth != 8 && bit_depth != 10 && bit_depth != 12)
        return AVERROR(AVERROR_INVALIDDATA);

    for (unsigned blockY = 0; blockY < plane->h_pxls; blockY += 32){  
        int padding_b = FFMAX(((int)(blockY + 32) - (int)(plane->h_pxls_src)), 0);
        for (unsigned blockX = 0; blockX < plane->w_pxls; blockX += 32){
            int offset = blockX * plane->pxl_depth + (blockY * stride);
            int padding_r = FFMAX((int)(blockX + 32) - (int)(plane->w_pxls_src), 0);

            // Copy values to block buffer 
            copy_vals_buffer(plane->pxl_depth, offset, 32, src, stride, block_buffer, padding_r, padding_b);

            (enable_lowpass ? ff_vca_lowpass_dct32 : ff_vca_dct32)(block_buffer, out_buffer, bit_depth);

            // Calculate energy and brightness
            // result.brightnessPerBlock[blockIndex] = uint32_t(sqrt(coeffBuffer[0]));
            result->energy[block_i] = calc_weighted_coeff(32, out_buffer, enable_lowpass);
            
            frameTexture += result->energy[block_i];
            block_i++;
        }
    }
    //av_freep(block_buffer);
    //av_freep(out_buffer);
    return  (uint32_t)((double)(frameTexture) / (plane->n_blocks * E_norm_factor));
}

static uint32_t calc_energy_16(int stride, uint8_t *src, VCAPlaneInfo *plane, VCAResults *result, int enable_lowpass){
    int block_i = 0u;
    uint32_t frameTexture = 0;

    ALIGN_VAR_32(int16_t, block_buffer[16 * 16]);
    ALIGN_VAR_32(int16_t, out_buffer[16 * 16]);

    const unsigned bit_depth = plane->bit_depth;
    if (bit_depth != 8 && bit_depth != 10 && bit_depth != 12)
        return AVERROR(AVERROR_INVALIDDATA);

    for (unsigned blockY = 0; blockY < plane->h_pxls; blockY += 16){  
        int padding_b = FFMAX(((int)(blockY + 16) - (int)(plane->h_pxls_src)), 0);
        for (unsigned blockX = 0; blockX < plane->w_pxls; blockX += 16){
            int offset = blockX * plane->pxl_depth + (blockY * stride);
            int padding_r = FFMAX((int)(blockX + 16) - (int)(plane->w_pxls_src), 0);

            copy_vals_buffer(plane->pxl_depth, offset, 16, src, stride, block_buffer, padding_r, padding_b);

            (enable_lowpass ? ff_vca_lowpass_dct16 : ff_vca_dct16)(block_buffer, out_buffer, bit_depth);

            result->energy[block_i] = calc_weighted_coeff(16, out_buffer, enable_lowpass);
            
            frameTexture += result->energy[block_i];
            block_i++;
        }
    }
    return  (uint32_t)((double)(frameTexture) / (plane->n_blocks * E_norm_factor));
}

static uint32_t calc_energy_8(int stride, uint8_t *src, VCAPlaneInfo *plane, VCAResults *result, int enable_lowpass){
    int block_i = 0u;
    uint32_t frameTexture = 0;

    ALIGN_VAR_32(int16_t, block_buffer[8 * 8]);
    ALIGN_VAR_32(int16_t, out_buffer[8 * 8]);

    const unsigned bit_depth = plane->bit_depth;
    if (bit_depth != 8 && bit_depth != 10 && bit_depth != 12)
        return AVERROR(AVERROR_INVALIDDATA);

    for (unsigned blockY = 0; blockY < plane->h_pxls; blockY += 8){  
        int padding_b = FFMAX(((int)(blockY + 8) - (int)(plane->h_pxls_src)), 0);
        for (unsigned blockX = 0; blockX < plane->w_pxls; blockX += 8){
            int offset = blockX * plane->pxl_depth + (blockY * stride);
            int padding_r = FFMAX((int)(blockX + 8) - (int)(plane->w_pxls_src), 0);

            copy_vals_buffer(plane->pxl_depth, offset, 8, src, stride, block_buffer, padding_r, padding_b);

            (enable_lowpass ? ff_vca_lowpass_dct8 : ff_vca_dct8)(block_buffer, out_buffer, bit_depth);

            result->energy[block_i] = calc_weighted_coeff(8, out_buffer, enable_lowpass);
            
            frameTexture += result->energy[block_i];
            block_i++;
        }
    }
    return  (uint32_t)((double)(frameTexture) / (plane->n_blocks * E_norm_factor));
}
    */