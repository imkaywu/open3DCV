#include "graph.h"

namespace open3DCV
{
    Graph::Graph()
    {
        // no op
    }
    
    Graph::Graph(const Pair& pair)
    {
        init(pair);
    }
    
    Graph::~Graph()
    {
        ind_cam_.clear();
        intrinsics_mat_.clear();
        extrinsics_mat_.clear();
        tracks_.clear();
        structure_points_.clear();
    }
    
    void Graph::init(const Pair& pair)
    {
        const size_t nmatches = pair.matches_.size();
        
        ncams_ = 2;
        ind_cam_ = pair.ind_cam_;
        F_ = pair.F_;
        E_ = pair.E_;
        intrinsics_mat_ = pair.intrinsics_mat_;
        extrinsics_mat_ = pair.extrinsics_mat_;
        
        tracks_.resize(nmatches);
        for (size_t i = 0; i < nmatches; ++i)
        {
            tracks_[i].add_keypoint(Keypoint(pair.matches_[i].first, pair.ind_cam_[0]));
            tracks_[i].add_keypoint(Keypoint(pair.matches_[i].second, pair.ind_cam_[1]));
        }
        
        structure_points_.resize(nmatches);
    }
    
    int Graph::index(int icam) const
    {
        for (int i = 0; i < ncams_; ++i)
        {
            if (icam == ind_cam_[i])
                return i;
        }
        return -1;
    }
    
    int Graph::size() const
    {
        return static_cast<int>(ind_cam_.size());
    }
}
