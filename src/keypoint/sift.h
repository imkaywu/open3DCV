#ifndef sift_h_
#define sift_h_

#include "vl/sift.h"
#include "keypoint/keypoint.h"
#include "keypoint/detector.h"
#include "keypoint/descriptor.h"

namespace open3DCV {
  
/*
enum option {
    opt_octaves = 0,
    opt_levels,
    opt_first_octave,
    opt_frames,
    opt_edge_thresh,
    opt_peak_thresh,
    opt_norm_thresh,
    opt_magnif,
    opt_window_size,
    opt_orientations,
    opt_float_descriptors,
    opt_verbose
};
 */

class Sift_Params {
    
};

class Sift : public Detector, public Descriptor {
    
public:
    
    Sift() { type_ = SIFT; };
    ~Sift();
    
    int convert(Image &image);
    int detect_keypoints_simp(Image &image, vector<Keypoint> &keypoint, int verbose = 0);
    int detect_keypoints(Image &image, vector<Keypoint> &keypoints, int verbose = 0);
    int extract_descript();
    void transpose_descriptor (vl_sift_pix* dst, vl_sift_pix* src);
    static bool ksort(const Keypoint &a, const Keypoint &b);
    vl_bool check_sorted(const vector<Keypoint> &keys, vl_size nkeys);
    
private:
    vl_sift_pix* data_;
    int width_, height_, channel_;
    KeypointType type_;
    
}; // end of class Sift

} // end of namespace open3DCV

#endif // sift_h
