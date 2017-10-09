#include <limits>
#include "keypoint/sift_param.h"

namespace open3DCV
{
    SiftParam::SiftParam()
    {
        num_octaves_ = 3;
        num_levels_ = 3;
        first_octave_ = 0;
        peak_thresh_ = 1.2f / 255.0; // 0
        edge_thresh_ = 10.0f / 255.0; // 10
        norm_thresh_ = -std::numeric_limits<float>::infinity();
        magnif_ = 3;
        window_size_ = 2;
        root_sift_ = true;
        upright_sift_ = true;
    }
    
    SiftParam::SiftParam(int num_octaves,
                         int num_levels,
                         int first_octave,
                         float edge_thresh,
                         float peak_thresh,
                         float norm_thresh,
                         float magnif,
                         float window_size) :
        num_octaves_(num_octaves), num_levels_(num_levels), first_octave_(first_octave),
        edge_thresh_(edge_thresh), peak_thresh_(peak_thresh),
        norm_thresh_(norm_thresh), magnif_(magnif), window_size_(window_size)
    {
    }
    
    SiftParam::SiftParam(int num_octaves,
                         int num_levels,
                         int first_octave,
                         float edge_thresh,
                         float peak_thresh) :
        num_octaves_(num_octaves), num_levels_(num_levels), first_octave_(first_octave),
        edge_thresh_(edge_thresh), peak_thresh_(peak_thresh)
    {
    }
    
    SiftParam::SiftParam(int num_octaves,
                         int num_levels,
                         int first_octave) :
        num_octaves_(num_octaves), num_levels_(num_levels), first_octave_(first_octave)
    {
    }
    
    SiftParam::SiftParam(const SiftParam& SiftParam) :
        num_octaves_(SiftParam.num_octaves_), num_levels_(SiftParam.num_levels_),
        first_octave_(SiftParam.first_octave_), edge_thresh_(SiftParam.edge_thresh_),
        peak_thresh_(SiftParam.peak_thresh_), norm_thresh_(SiftParam.norm_thresh_),
        magnif_(SiftParam.magnif_), window_size_(SiftParam.window_size_)
    {
    }
    
    SiftParam::~SiftParam()
    {
    }
}
