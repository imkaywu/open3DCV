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
        Graph& operator=(const Graph& graph);
        ~Graph();
        
        void init(const Graph& graph);
        void init(const Pair& pair);
        int index(int icam) const;
        int sz_cams() const; // number of cameras;
        int sz_tracks() const; // number of feature tracks;
        void add_track(const Track& track);
        void rm_track(int index);
        void add_struct_pt(const Structure_Point& struct_pt);
        void rm_struct_pt(int index);
        void rm_outliers(const float thresh_reproj, const float thresh_angle);
        bool operator<(const Graph& rhs) const;
        float baseline_angle() const;
        static void merge_graph(Graph& graph1, Graph& graph2);
        static void merge_tracks(Track& track1, const Track& track2, std::vector<std::pair<int, int> >& ind_key);
        static int find_next_graph(const std::vector<Graph>& graphs, const Graph& graph, std::vector<int>& merged_graph);

        int ncams_;
        std::vector<int> cams_;
        Mat3f F_; // to be deleted
        Mat3f E_; // to be deleted
        std::vector<Mat3f> intrinsics_mat_;
        std::vector<Mat34f> extrinsics_mat_;
        std::vector<Track> tracks_;
        std::vector<Structure_Point> structure_points_;
    };
}

#endif
