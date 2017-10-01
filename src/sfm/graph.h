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
        Graph(const Graph& graph);
        ~Graph();
        
        void init(const Pair& pair);
        int index(int icam) const;
        int size() const; // number of 3D points
        void rm_outliers();
        void add_track(const Track& track);
        void rm_track(int index);
        void add_struct_pt(const Structure_Point& struct_pt);
        void rm_struct_pt(int index);
        static void merge_graph(Graph& graph1, Graph& graph2);
        static void merge_tracks(Track& track1, const Track& track2, std::vector<std::pair<int, int> >& ind_key);
        bool operator<(const Graph& rhs) const;
        
        int ncams_;
        std::vector<int> cams_;
        Mat3f F_;
        Mat3f E_;
        std::vector<Mat3f> intrinsics_mat_;
        std::vector<Mat34f> extrinsics_mat_;
        std::vector<Track> tracks_;
        std::vector<Structure_Point> structure_points_;
    };
}

#endif
