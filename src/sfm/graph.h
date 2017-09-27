#ifndef graph_h_
#define graph_h_

#include "matching/pair.h"
#include "sfm/track.h"
#include "sfm/structure_point.h"

namespace open3DCV
{
    class Graph
    {
    public:
        Graph();
        Graph(const Pair& pair);
        ~Graph();
        
        void init(const Pair& pair);
        int index(int icam) const;
        int size() const;
        static std::vector<int> intersect(const std::vector<Track>& tracks1, const std::vector<Track>& tracks2);
        
        int ncams_;
        std::vector<int> ind_cam_;
        Mat3f F_;
        Mat3f E_;
        float f_;
        std::vector<Mat3f> intrinsics_mat_;
        std::vector<Mat34f> extrinsics_mat_;
        std::vector<Track> tracks_;
        std::vector<Structure_Point> structure_points_;
    };
}

#endif
