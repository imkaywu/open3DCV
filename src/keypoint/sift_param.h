#ifndef sift_param_h_
#define sift_param_h_

namespace open3DCV
{
    // overview of SIFT detector parameters: http://www.vlfeat.org/api/sift.html#sift-intro-detector
    class SiftParam
    {
    public:
        SiftParam();
        SiftParam(int num_octaves,
                  int num_levels,
                  int first_octave,
                  float edge_thresh,
                  float peak_thresh,
                  float norm_thresh,
                  float magnif,
                  float window_size);
        
        SiftParam(int num_octaves,
                  int num_levels,
                  int first_octave,
                  float edge_thresh,
                  float peak_thresh);
        
        SiftParam(int num_octaves,
                  int num_levels,
                  int first_octave);
        
        SiftParam(const SiftParam& SiftParam);
        
        ~SiftParam();
        
        /********** Detector parameters **********/
        // number of octave of the DoG scale space.
        int num_octaves_;
        // number of levels per octave of the DoG scale space.
        int num_levels_;
        // index of the first octave of the DoG scale space.
        int first_octave_;
        // peak selection threshold, decrease to eliminate more keypoints
        float edge_thresh_;
        // non-edge selection threshold, increase to eliminate more keypoints
        float peak_thresh_;
        
        /********** Descriptor parameters **********/
        // Set the minimum l2-norm of the descriptors before normalization.
        // Descriptors below the threshold are set to zero.
        float norm_thresh_;
        // Set the descriptor magnification factor. The scale of the
        // keypoint is multiplied by this factor to obtain the width (in
        // pixels) of the spatial bins. For instance, if there are 
        // 4 spatial bins along each spatial direction, the
        // ``side'' of the descriptor is approximatively 4 * MAGNIF.
        float magnif_;
        // Set the variance of the Gaussian window that determines the
        // descriptor support. It is expressend in units of spatial bins.
        float window_size_;
        
        bool root_sift_;
        /* Upright sift enables only a single descriptor to be extracted at a given
         * location. This is useful for SfM for a number of reasons, especially during
         * geometric verification.
         */
        bool upright_sift_;
    };
}

#endif
