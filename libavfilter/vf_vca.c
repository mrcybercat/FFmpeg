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


#include "avfilter.h"
#include "filters.h"
#include "formats.h"
#include "video.h"

#include "vca_dct.h"

typedef struct ThreadData {
    void (*perform_dct)(const int16_t* block, int16_t* dst, int bit_depth);
    int stride;
    int blocksize;

    int enable_lowpass;

    uint8_t *src;

    VCAPlaneInfo *plane;
    VCAResults *result;
    
    uint32_t *partial_sums;
} ThreadData;

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
    { "simd", "Enable hardware acceralation with SIMD", OFFSET(enable_simd), AV_OPT_TYPE_BOOL, { .i64=0 }, 0, 1, FLAGS },
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

#define DEFINE_CALC_ENERGY_SLICE(BLOCKSIZE)                                                         \
static uint32_t calc_energy_##BLOCKSIZE##_slice(                                                    \
    int stride, uint8_t *src, VCAPlaneInfo *plane, VCAResults *result,                              \
    int enable_lowpass, int slice_start, int slice_end, uint32_t *partial_sum,                      \
    void (*perform_dct)(const int16_t* block, int16_t* dst, int bit_depth)) {                       \
    int block_i = (slice_start / BLOCKSIZE) * plane->w_blocks;                                      \
    uint32_t sliceTexture = 0;                                                                      \
    ALIGN_VAR_32(int16_t, block_buffer[BLOCKSIZE * BLOCKSIZE]);                                     \
    ALIGN_VAR_32(int16_t, out_buffer[BLOCKSIZE * BLOCKSIZE]);                                       \
    const unsigned bit_depth = plane->bit_depth;                                                    \
    if (bit_depth != 8 && bit_depth != 10 && bit_depth != 12)                                       \
        return AVERROR(AVERROR_INVALIDDATA);                                                        \
    for (unsigned blockY = slice_start; blockY < slice_end; blockY += BLOCKSIZE) {                  \
        int padding_b = FFMAX(((int)(blockY + BLOCKSIZE) - (int)(plane->h_pxls_src)), 0);           \
        for (unsigned blockX = 0; blockX < plane->w_pxls; blockX += BLOCKSIZE) {                    \
            int offset = blockX * plane->pxl_depth + (blockY * stride);                             \
            int padding_r = FFMAX((int)(blockX + BLOCKSIZE) - (int)(plane->w_pxls_src), 0);         \
            /* Copy values to block buffer */                                                       \
            copy_vals_buffer(plane->pxl_depth, offset, BLOCKSIZE, src, stride,                      \
                             block_buffer, padding_r, padding_b);                                   \
            perform_dct(block_buffer, out_buffer, bit_depth);                                       \
            /* Calculate energy */                                                                  \
            result->energy[block_i] = calc_weighted_coeff(BLOCKSIZE, out_buffer, enable_lowpass);   \
            sliceTexture += result->energy[block_i];                                                \
            block_i++;                                                                              \
        }                                                                                           \
    }                                                                                               \
    *partial_sum = sliceTexture;                                                                    \
    return sliceTexture;                                                                            \
}

DEFINE_CALC_ENERGY_SLICE(8)
DEFINE_CALC_ENERGY_SLICE(16)
DEFINE_CALC_ENERGY_SLICE(32)

static int calc_energy_filter_slice(AVFilterContext *ctx, void *arg, int job, int nb_jobs){
    ThreadData *th = arg;
    uint32_t energy = 0;

    int block_row_start = (th->plane->h_blocks * job)     / nb_jobs;
    int block_row_end   = (th->plane->h_blocks * (job+1)) / nb_jobs;

    int slice_start = block_row_start * th->blocksize;
    int slice_end   = block_row_end   * th->blocksize;

    switch (th->blocksize) {
        case 32:
            energy = calc_energy_32_slice(th->stride, th->src, th->plane, th->result, th->enable_lowpass, slice_start, slice_end, &th->partial_sums[job], th->perform_dct);
            break;
        case 16:
            energy = calc_energy_16_slice(th->stride, th->src, th->plane, th->result, th->enable_lowpass, slice_start, slice_end, &th->partial_sums[job], th->perform_dct);
            break;
        case 8:
            energy = calc_energy_8_slice(th->stride, th->src, th->plane, th->result, th->enable_lowpass, slice_start, slice_end, &th->partial_sums[job], th->perform_dct);
            break;
        default:
            return AVERROR(AVERROR_INVALIDDATA);
    }

    return 0;
}


static uint32_t calc_energy(AVFilterContext *ctx, int linesize, uint8_t *src, VCAPlaneInfo *plane, VCAResults *result, int blocksize, int enable_lowpass, void* perform_dct){
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
        .perform_dct = perform_dct
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

    E[plane_i] = calc_energy(ctx, in->linesize[plane_i], in->data[plane_i], v->vca_plane[plane_i], v->vca_result[plane_i], v->blocksize, v->enable_lowpass, v->perform_dct);
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
        
    if (v->enable_lowpass) {
        switch (v->blocksize) {
            case 32:
                v->perform_dct = ff_vca_lowpass_dct32_c;
                break;
            case 16:
                v->perform_dct = ff_vca_lowpass_dct16_c;
                break;
            case 8:
                v->perform_dct = ff_vca_lowpass_dct8_c;
                break;
            default:
                return AVERROR(AVERROR_INVALIDDATA);
        }
    }
    else {            
        switch (v->blocksize) {
            case 32:
                v->perform_dct = ff_vca_dct32_c;
                break;
            case 16:
                v->perform_dct = ff_vca_dct16_c;
                break;
            case 8:
                v->perform_dct = ff_vca_dct8_c;
                break;
            default:
                return AVERROR(AVERROR_INVALIDDATA);
        }
    }

    if (v->enable_simd) {
        #if ARCH_X86
        ret = ff_vca_dct_init_x86(v);
        if (ret != 0) {
            return ret;
        }
        #endif
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