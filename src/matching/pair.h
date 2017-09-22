#ifndef pair_h_
#define pair_h_

#include "math/numeric.h"
#include "sfm/track.h"
#include "sfm/structure_point.h"

namespace open3DCV
{
    class Pair
    {
    public:
        Pair(const std::vector<int>& ind_cam);
        Pair(const std::vector<int>& ind_cam, const std::vector<std::pair<Vec2f, Vec2f> >& matches);
        ~Pair();
        
        std::vector<int> ind_cam_;
        std::vector<std::pair<Vec2f, Vec2f> > matches_;
        Mat3f F_;
        Mat3f E_;
        std::vector<Mat3f> K_;
        std::vector<Mat34f> Rt_;
        
    private:
        void init(const std::vector<int>& ind_cam);
    };
    
}

#endif
