#ifndef pair_h_
#define pair_h_

#include "math/numeric.h"
#include "matching/dmatch.h"
#include "sfm/track.h"
#include "sfm/structure_point.h"

namespace open3DCV
{
    class Pair
    {
    public:
        Pair();
        Pair(const int cam1, const int cam2);
        Pair(const int cam1, const int cam2, const std::vector<std::pair<Vec2f, Vec2f> >& matches);
        Pair(const int cam1, const int cam2, const std::vector<DMatch>& matches);
        ~Pair();
        
        void init(const int ind_cam1, const int ind_cam2);
        void update_matches(const int* vote_inlier);
        void update_intrinsics(const float f, const int w, const int h);
        bool operator<(const Pair& rhs) const;
        float baseline_angle() const;
        
        std::vector<int> cams_;
        std::vector<DMatch> matches_;
        Mat3f F_;
        Mat3f E_;
        std::vector<Mat3f> intrinsics_mat_;
        std::vector<Mat34f> extrinsics_mat_;
        
    };
    
}

#endif
