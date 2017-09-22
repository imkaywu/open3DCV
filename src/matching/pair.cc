#include "matching/pair.h"

using std::vector;
using std::pair;

namespace open3DCV
{
    Pair::Pair(const std::vector<int>& ind_cam)
    {
        init(ind_cam);
    }
    
    Pair::Pair(const std::vector<int>& ind_cam, const std::vector<std::pair<Vec2f, Vec2f> >& matches) : matches_(matches)
    {
        init(ind_cam);
    }
    
    Pair::~Pair()
    {
        
    }
    
    void Pair::init(const std::vector<int>& ind_cam)
    {
        ind_cam_.resize(2);
        ind_cam_[0] = ind_cam[0];
        ind_cam_[1] = ind_cam[1];
        K_.resize(2);
        Rt_.resize(2);
        Rt_[0].block<3, 3>(0, 0) = Mat3f::Identity();
        Rt_[0].block<3, 1>(0, 3).setZero();
    }
}
