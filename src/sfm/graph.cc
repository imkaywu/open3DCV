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
    
    void Graph::init(const Pair& pair)
    {
        ncams_ = 2;
        ind_cam_ = pair.ind_cam_;
        F_ = pair.F_;
        E_ = pair.E_;
        K_ = pair.K_;
        Rt_ = pair.Rt_;
        
        const size_t nmatches = pair.matches_.size();
        tracks_.resize(nmatches);
        for (size_t i = 0; i < nmatches; ++i)
        {
            tracks_[i].add_keypoint(Keypoint(pair.matches_[i].first, pair.ind_cam_[0]));
            tracks_[i].add_keypoint(Keypoint(pair.matches_[i].second, pair.ind_cam_[1]));
        }
        
        structure_points_.resize(nmatches);
    }
    
    Graph::~Graph()
    {
        // no op
    }
    
    int Graph::index(const int icam) const
    {
        for (int i = 0; i < ncams_; ++i)
        {
            if (icam == ind_cam_[i])
                return i;
        }
        return -1;
    }
}
