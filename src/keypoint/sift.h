#ifndef sift_h_
#define sift_h_

#include "vl/sift.h"
#include "keypoint/keypoint.h"
#include "keypoint/detector.h"
#include "keypoint/descriptor.h"

namespace open3DCV {

class Sift_Params {
public:
    Sift_Params() {};
    Sift_Params(int num_octaves,
                int num_levels,
                int first_octave,
                float edge_thresh,
                float peak_thresh,
                float norm_thresh,
                float magnif,
                float window_size) :
        num_octaves_(num_octaves), num_levels_(num_levels),
        first_octave_(first_octave), edge_thresh_(edge_thresh),
        peak_thresh_(peak_thresh), norm_thresh_(norm_thresh),
        magnif_(magnif), window_size_(window_size){ };
    
    Sift_Params(int num_octaves,
                int num_levels,
                int first_octave) :
        num_octaves_(num_octaves), num_levels_(num_levels), first_octave_(first_octave) { };
    
    Sift_Params(const Sift_Params& sift_params) :
        num_octaves_(sift_params.num_octaves_), num_levels_(sift_params.num_levels_),
        first_octave_(sift_params.first_octave_), edge_thresh_(sift_params.edge_thresh_),
        peak_thresh_(sift_params.peak_thresh_), norm_thresh_(sift_params.norm_thresh_),
        magnif_(sift_params.magnif_), window_size_(sift_params.window_size_){ };
    
    ~Sift_Params() { };
    
    int num_octaves_ = 3;
    int num_levels_ = 3;
    int first_octave_ = 0;
    float edge_thresh_ = 10.0f / 255.0;
    float peak_thresh_ = 1.2f / 255.0;
    float norm_thresh_ = -INFINITY; // re-initialize
    float magnif_ = 3; // re-initialize
    float window_size_ = 2; // re-initialize
    bool root_sift_ = true;
    bool upright_sift_ = true;
};

class Sift : public Detector, public Descriptor {
    
public:
    
    Sift();
    Sift(const Sift_Params& sift_params);
    Sift(Image& image);
    ~Sift();
    
    int convert(const Image &image);
    int detect_keypoints(const Image& image, vector<Keypoint>& keypoint, int verbose = 0);
    int detect_keypoints_unfinished(const Image& image, vector<Keypoint>& keypoints, int verbose);
    int extract_descriptor(const Image& image, const Keypoint& keypoint, Vecf& descriptor);
    int extract_descriptors(const Image& image, vector<Keypoint>& keypoints, vector<Vecf>& descriptors);
    double get_valid_first_octave(const int first_octave, const int width, const int height);
    void convert_root_sift(Vecf& descriptor);
    void set_params(const Sift_Params& sift_params);
    void clear();
    // not used
    void transpose_descriptor (vl_sift_pix* dst, vl_sift_pix* src);
    static bool ksort(const Keypoint& a, const Keypoint& b);
    vl_bool check_sorted(const vector<Keypoint> &keys, vl_size nkeys);
    
private:
    vl_sift_pix* data_;
    VlSiftFilt* sift_filter_;
    Sift_Params sift_params_;
    int width_, height_, channel_;
    KeypointType type_;
    
}; // end of class Sift

} // end of namespace open3DCV

#endif // sift_h
