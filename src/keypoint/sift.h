#ifndef sift_h
#define sift_h

#include "vl/sift.h"
#include "image.h"
#include "numeric.h"
#include "keypoint.h"

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

class Sift : public Keypoint {
    
public:
    
    Sift(const Vec2 x, unsigned int i);
    Sift(const Vec2 x, unsigned int i, const Vec3i c);
    ~Sift();
    
    int convert(Image &img);
    int detect();
    int descript();
    void transpose_descriptor (vl_sift_pix* dst, vl_sift_pix* src);
    
private:
    vl_sift_pix* data_;
    int width_, height_, channel_;
    Mat4X keys_;
    Mat descr_;
    
}; // end of class Sift
    
} // end of namespace open3DCV

#endif // sift_h
