#include "matching/pair.h"
#include "transform/rodrigues.h"

using std::vector;
using std::pair;

namespace open3DCV
{
    
    Pair::Pair() : cams_(-1, -1)
    {
        // no op
    }
    
    Pair::Pair(const int cam1, const int cam2)
    {
        init(cam1, cam2);
    }
    
    Pair::Pair(const int cam1, const int cam2, const vector<DMatch>& matches) : matches_(matches)
    {
        init(cam1, cam2);
    }
    
    Pair::~Pair()
    {
        cams_.clear();
        matches_.clear();
        intrinsics_mat_.clear();
        extrinsics_mat_.clear();
    }
    
    void Pair::update_matches(const int* vote_inlier)
    {
        for (int i = static_cast<int>(matches_.size() - 1); i >= 0; --i)
        {
            if (!vote_inlier[i])
            {
                matches_.erase(matches_.begin() + i);
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
    
    void Pair::init(const int cam1, const int cam2)
    {
        cams_.resize(2);
        cams_[0] = cam1;
        cams_[1] = cam2;
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
    
    bool Pair::operator<(const Pair& rhs) const
    {
        return matches_.size() > rhs.matches_.size();
    }
    
    float Pair::baseline_angle() const
    {
        Mat3f rotation = extrinsics_mat_[1].block<3, 3>(0, 0);
        Vec3f om;
        irodrigues(om, nullptr, rotation);
        return om.norm();
    }
}
