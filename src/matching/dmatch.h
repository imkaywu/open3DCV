#ifndef dmatch_h_
#define dmatch_h_

#include "keypoint/keypoint.h"

namespace open3DCV
{
    class DMatch
    {
    public:
        DMatch () {};
        DMatch (const int r_ikey1, const int r_ikey2, const float dist) :
            ind_key_(r_ikey1, r_ikey2), dist_(dist) {};
        DMatch (const int r_ikey1, const int r_ikey2, const Vec2f r_pt1, const Vec2f r_pt2, const float dist) :
            ind_key_(r_ikey1, r_ikey2), point_(r_pt1, r_pt2), dist_(dist) {};
        DMatch (const DMatch& match) :
            ind_key_(match.ind_key_), point_(match.point_), dist_(match.dist_) {};
        
        const float& dist() const;
        
        std::pair<int, int> ind_key_;
        std::pair<Vec2f, Vec2f> point_;
        float dist_;
    };
    
    inline const float& DMatch::dist() const
    {
        return dist_;
    }

} // namespace open3DCV

#endif
