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
#include "vca_ovca.h"
#include "vca_evca.h"
#include "vca_svca.h"

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
    // Considering depricating summary option 
    { "file", "Set file where to print analysis information", OFFSET(file_str), AV_OPT_TYPE_STRING, {.str=NULL}, 0, 0, FLAGS },
    //{ "yuview", "Ignore extension detection and force output YUView stats to file", OFFSET(yuview), AV_OPT_TYPE_BOOL, { .i64=0 }, 0, 1, FLAGS },
    // Algos 
    { "algo", "Alalysis algorithm to use", OFFSET(algo), AV_OPT_TYPE_INT,
        { .i64 = ALGO_STANDARD_VCA }, 0, INT_MAX, .flags = FLAGS, .unit = "algo" },
    { "vca", "Standard VCA is used (fastest)", 0, AV_OPT_TYPE_CONST,
        { .i64 = ALGO_STANDARD_VCA }, INT_MIN, INT_MAX, .flags = FLAGS, .unit = "algo" },
    { "evca", "Enhanced VCA is used (less perfomance, better corelation)", 0, AV_OPT_TYPE_CONST,
        { .i64 = ALGO_ENHANCED_VCA }, INT_MIN, INT_MAX, .flags = FLAGS, .unit = "algo" },
    { "svca", "Stereoscopic VCA is used (intended only for stereoscopic videos)", 0, AV_OPT_TYPE_CONST,
        { .i64 = ALGO_STEREO_VCA }, INT_MIN, INT_MAX, .flags = FLAGS, .unit = "algo" },
    { "ivca", "Inter-relation-aware VCA is used (slightly less perfomance, better corelation)", 0, AV_OPT_TYPE_CONST,
        { .i64 = ALGO_INTER_VCA }, INT_MIN, INT_MAX, .flags = FLAGS, .unit = "algo" },
    { NULL }
};

AVFILTER_DEFINE_CLASS(vca);

static const enum AVPixelFormat pxl_fmts[] = {
    AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV422P,
    AV_PIX_FMT_YUVJ420P, AV_PIX_FMT_YUVJ422P,
    AV_PIX_FMT_YUV420P10, AV_PIX_FMT_YUV422P10,
    AV_PIX_FMT_NONE
};

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

static VCAAlgoContext *algo_create_ovca(int n_blocks) {
    OVCAAlgoContext *ovca = av_mallocz(sizeof(*ovca));
    ovca->base.vtable = &ovca_vtable;
    ovca->base.vtable->init_algo((VCAAlgoContext *)ovca, n_blocks, NULL);
    return (VCAAlgoContext *)ovca;
}

static VCAAlgoContext *algo_create_evca(int n_blocks, int blocksize) {
    EVCAAlgoContext *evca = av_mallocz(sizeof(*evca));
    evca->base.vtable = &evca_vtable;
    evca->base.vtable->init_algo((VCAAlgoContext *)evca, n_blocks, blocksize);
    return (VCAAlgoContext *)evca;
}

static VCAAlgoContext *algo_create_svca(int n_blocks) {
    SVCAAlgoContext *svca = av_mallocz(sizeof(*svca));
    svca->base.vtable = &svca_vtable;
    svca->base.vtable->init_algo((VCAAlgoContext *)svca, n_blocks, NULL);
    return (VCAAlgoContext *)svca;
}


static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    VCAContext *v = ctx->priv;
    FilterLink *inl = ff_filter_link(inlink);
    int planes = v->enable_chroma ? 3 : 1;

    if (v->n_frames_processed >= v->n_frames)
        return ff_filter_frame(inlink->dst->outputs[0], in);
    
    //for (int i = 0; i < planes; i++)
    //    v->perform_xvca(ctx, inlink, in, inl, v, i);
    for (int i = 0; i < planes; i++)
        v->algoctx[i]->vtable->perform_algo(ctx, in, inl, v, i);

    v->n_frames_processed++;

    return ff_filter_frame(inlink->dst->outputs[0], in);
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    VCAContext *v = ctx->priv;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    int max_pixsteps[4];
    int planes = v->enable_chroma ? 3 : 1;

    // Allocate arrays of pointers
    v->algoctx = av_calloc(planes, sizeof(*v->algoctx));
    v->plane  = av_calloc(planes, sizeof(*v->plane));
    if (!v->algoctx || !v->plane)
        return AVERROR(ENOMEM);

    // Allocate each plane/result struct
    for (int i = 0; i < planes; i++) {
        //v->algoctx[i] = av_mallocz(sizeof(*v->algoctx[i]));
        v->plane[i]  = av_mallocz(sizeof(*v->plane[i]));

        if (!v->plane[i]) {
            // Clean allocations on error
            for (int j = 0; j <= i; j++) {
                av_freep(&v->plane[j]);
            }
            av_freep(&v->plane);
            return AVERROR(ENOMEM);
        }
    }

    v->plane[0]->w_pxls_src = inlink->w;
    v->plane[0]->h_pxls_src = inlink->h;

    if (v->enable_chroma) {
        v->plane[1]->w_pxls_src = AV_CEIL_RSHIFT(inlink->w, desc->log2_chroma_w);
        v->plane[1]->h_pxls_src = AV_CEIL_RSHIFT(inlink->h, desc->log2_chroma_h);

        v->plane[2]->w_pxls_src = AV_CEIL_RSHIFT(inlink->w, desc->log2_chroma_w);
        v->plane[2]->h_pxls_src = AV_CEIL_RSHIFT(inlink->h, desc->log2_chroma_h);

        planes = 3;
    } else 
        planes = 1;

    for (int i = 0; i < planes; i++) {
        // Inference of bit depth 
        v->plane[i]->bit_depth = desc->comp[i].depth;
        // Inference of pixel depth
        av_image_fill_max_pixsteps(max_pixsteps, NULL, desc);
        v->plane[i]->pxl_depth = max_pixsteps[i];

        v->plane[i]->w_blocks = (v->plane[i]->w_pxls_src + v->blocksize - 1) / v->blocksize;
        v->plane[i]->h_blocks = (v->plane[i]->h_pxls_src + v->blocksize - 1) / v->blocksize;

        v->plane[i]->n_blocks = v->plane[i]->w_blocks * v->plane[i]->h_blocks;    
        
        v->plane[i]->w_pxls = v->plane[i]->w_blocks * v->blocksize;
        v->plane[i]->h_pxls = v->plane[i]->h_blocks * v->blocksize;
    }

    for (int i = 0; i < planes; i++) {
        switch (v->algo) {
            case ALGO_STANDARD_VCA:
                v->algoctx[i] = algo_create_ovca(v->plane[i]->n_blocks);
                break;
            case ALGO_ENHANCED_VCA:
                v->algoctx[i] = algo_create_evca(v->plane[i]->n_blocks, v->blocksize);
                break;
            case ALGO_STEREO_VCA:
                v->algoctx[i] = algo_create_svca(v->plane[i]->n_blocks);
                break;
            case ALGO_INTER_VCA:
                v->algoctx[i] = algo_create_ovca(v->plane[i]->n_blocks);
                break;
        }
    }

    if(v->algo != ALGO_STEREO_VCA)
        v->print(ctx, AV_LOG_INFO, "POC,E,h");
    else
        v->print(ctx, AV_LOG_INFO, "POC,E_l,h_l,E_r,h_r,s");
    if (v->enable_texture)
        v->print(ctx, AV_LOG_INFO, ",L");
    if (v->enable_chroma)
        v->print(ctx, AV_LOG_INFO, ",EV,hV,EU,hE");
    if (v->enable_chroma && v->enable_texture)
        v->print(ctx, AV_LOG_INFO, ",avgV,avgU");

    v->print(ctx, AV_LOG_INFO, "\n");

    av_log(ctx, AV_LOG_INFO, "threads: %d\n", ff_filter_get_nb_threads(ctx));

    return 0;
}

static av_cold int init(AVFilterContext *ctx)
{
    // User options but no input data
    VCAContext *v = ctx->priv;
    int ret;
    int planes = v->enable_chroma ? 3 : 1;
    v->n_frames_processed = 0;
    
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
    } else {            
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
            av_log(ctx, AV_LOG_ERROR, "Error initialidzing SIMD functions");
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

    if (v->algoctx) {
        for (int i = 0; i < planes; i++) {
            if(v->algoctx[i]){
                v->algoctx[i]->vtable->uninit_algo(v->algoctx[i]);
                av_free(v->algoctx[i]);
            }   
        }
        av_free(v->algoctx);
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