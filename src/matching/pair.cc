#include "matching/pair.h"

using std::vector;
using std::pair;

namespace open3DCV
{
    Pair::Pair(const int ind_cam1, const int ind_cam2)
    {
        init(ind_cam1, ind_cam2);
    }
    
    Pair::Pair(const int ind_cam1, const int ind_cam2, const std::vector<std::pair<Vec2f, Vec2f> >& matches) : matches_(matches)
    {
        init(ind_cam1, ind_cam2);
    }
    
    Pair::~Pair()
    {
        ind_cam_.clear();
        matches_.clear();
        intrinsics_mat_.clear();
        extrinsics_mat_.clear();
    }
    
    void Pair::update_matches(const std::vector<std::pair<Vec2f, Vec2f> >& matches, const int* vote_inlier)
    {
        matches_.clear();
        for (int i = 0; i < matches.size(); ++i)
        {
            if (vote_inlier[i])
            {
                matches_.push_back(matches[i]);
            }
        }
    }
    
    void Pair::update_intrinsics(const float f, const int w, const int h)
    {
        intrinsics_mat_[0] << f, 0, w/2.0,
                              0, f, h/2.0,
                              0, 0, 1;
        intrinsics_mat_[1] = intrinsics_mat_[0];
    }
    
    void Pair::init(const int ind_cam1, const int ind_cam2)
    {
        ind_cam_.resize(2);
        ind_cam_[0] = ind_cam1;
        ind_cam_[1] = ind_cam2;
        intrinsics_mat_.resize(2);
        for (int i = 0; i < 2; ++i)
        {
            intrinsics_mat_[i].setZero();
        }
        extrinsics_mat_.resize(2);
        for (int i = 0; i < 2; ++i)
        {
            extrinsics_mat_[i].block<3, 3>(0, 0).setIdentity();
            extrinsics_mat_[i].block<3, 1>(0, 3).setZero();
        }
    }
}
