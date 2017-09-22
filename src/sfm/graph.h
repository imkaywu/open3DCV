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
        int index(const int icam) const;
        
        int ncams_;
        std::vector<int> ind_cam_;
        Mat3f F_;
        Mat3f E_;
        float f_;
        std::vector<Mat3f> K_;
        std::vector<Mat34f> Rt_;
        std::vector<Track> tracks_;
        std::vector<Structure_Point> structure_points_;
        
    };
}

#endif
