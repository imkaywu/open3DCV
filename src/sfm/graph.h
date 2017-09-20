#ifndef graph_h_
#define graph_h_

#include "matching/match.h"
#include "track.h"
#include "structure_point.h"

namespace open3DCV
{
    class Graph
    {
        Graph();
        ~Graph();
        
        std::vector<int> iframes_;
        std::vector<Match> matches_;
        Mat3f F_;
        Mat3f E_;
        float f_;
        std::vector<Mat34f> pose_;
        std::vector<Track> tracks_;
        std::vector<Structure_Point> structure_point_;
    };
}

#endif
