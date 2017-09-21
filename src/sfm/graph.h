#ifndef graph_h_
#define graph_h_

#include "matching/match.h"
#include "track.h"
#include "structure_point.h"

namespace open3DCV
{
    struct Graph
    {
        Graph();
        Graph(const int& nframes, const std::vector<int>& iframes, const std::vector<Keypoint>& keys, const std::vector<Match>& matches);
        ~Graph();
        
        int nframes_;
        std::vector<int> iframes_;
        std::vector<Keypoint> keys_;
        std::vector<Match> matches_;
        Mat3f F_;
        Mat3f E_;
        float f_;
        std::vector<Mat3f> K_;
        std::vector<Mat34f> Rt_;
        std::vector<Track> tracks_;
        std::vector<Structure_Point> structure_point_;
    };
}

#endif
