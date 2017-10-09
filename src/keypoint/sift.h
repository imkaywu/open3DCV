#ifndef sift_h_
#define sift_h_

#include "vl/sift.h"
#include "keypoint/keypoint.h"
#include "keypoint/detector.h"
#include "keypoint/descriptor.h"
#include "keypoint/sift_param.h"

namespace open3DCV {

    class Sift : public Detector, public Descriptor
    {
    public:
        
        Sift();
        Sift(const SiftParam& sift_param);
        Sift(Image& image);
        ~Sift();
        
        int convert(const Image &image);
        int detect_keypoints(const Image& image, vector<Keypoint>& keypoint, int verbose = 0);
        int detect_keypoints_unfinished(const Image& image, vector<Keypoint>& keypoints, int verbose);
        int extract_descriptor(const Image& image, const Keypoint& keypoint, Vecf& descriptor);
        int extract_descriptors(const Image& image, vector<Keypoint>& keypoints, vector<Vecf>& descriptors);
        double get_valid_first_octave(const int first_octave, const int width, const int height);
        void convert_root_sift(Vecf& descriptor);
        void set_params(const SiftParam& sift_param);
        void clear();
        // not used
        void transpose_descriptor (vl_sift_pix* dst, vl_sift_pix* src);
        static bool ksort(const Keypoint& a, const Keypoint& b);
        vl_bool check_sorted(const vector<Keypoint> &keys, vl_size nkeys);
        
    private:
        vl_sift_pix* data_;
        VlSiftFilt* sift_filter_;
        SiftParam sift_param_;
        int width_, height_, channel_;
        
    }; // end of class Sift

} // end of namespace open3DCV

#endif // sift_h
