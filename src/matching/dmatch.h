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
            ind_key_(r_ikey1, r_ikey2), dist_(dist)
        {
            ikey1_ = r_ikey1;
            ikey2_ = r_ikey2;
        };
        DMatch (const int r_ikey1, const int r_ikey2, const Vec2f r_pt1, const Vec2f r_pt2, const float dist) :
            ind_key_(r_ikey1, r_ikey2), point_(r_pt1, r_pt2), dist_(dist)
        {
            ikey1_ = r_ikey1;
            ikey2_ = r_ikey2;
        };
        
        void update_match_pt(const std::vector<Keypoint>& key1, const std::vector<Keypoint>& key2);
        
        int ikey1_;
        int ikey2_;
        std::pair<int, int> ind_key_;
        std::pair<Vec2f, Vec2f> point_;
        float dist_;
    };
    
    inline void DMatch::update_match_pt(const std::vector<Keypoint>& key1, const std::vector<Keypoint>& key2)
    {
        point_.first = key1[ind_key_.first].coords();
        point_.second = key2[ind_key_.second].coords();
    }

} // namespace open3DCV

#endif
