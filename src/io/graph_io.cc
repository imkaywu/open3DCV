#include <fstream>
#include "io/graph_io.h"

using std::string;

namespace open3DCV
{
    int read_graph(const std::string fname, Graph& graph)
    {
        std::ifstream ifstr;
        ifstr.open(fname.c_str());
        if (!ifstr.is_open())
        {
            std::cerr << "Cannot open the file." << std::endl;
            return 1;
        }
        ifstr >> graph.ncams_;
        graph.cams_.resize(graph.ncams_);
        graph.intrinsics_mat_.resize(graph.ncams_);
        graph.extrinsics_mat_.resize(graph.ncams_);
        for (int i = 0; i < graph.ncams_; ++i)
            ifstr >> graph.cams_[i];
        
        // read camera parameters
        for (int i = 0; i < graph.ncams_; ++i)
        {
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    ifstr >> graph.intrinsics_mat_[i](r, c);
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 4; ++c)
                    ifstr >> graph.extrinsics_mat_[i](r, c);
        }
        
        // read tracks
        int ntracks, nkeys;
        ifstr >> ntracks;
        graph.tracks_.resize(ntracks, Track());
        for (int i = 0; i < ntracks; ++i)
        {
            ifstr >> nkeys;
            Track& track = graph.tracks_[i];
            for (int j = 0; j < nkeys; ++j)
            {
                Keypoint key;
                ifstr >> key;
                track.add_keypoint(key);
            }
        }
        
        // read structure_point
        int npts;
        ifstr >> npts;
        graph.structure_points_.resize(npts, Structure_Point());
        for (int i = 0; i < npts; ++i)
        {
            Vec3f& pt = graph.structure_points_[i].coords();
            ifstr >> pt(0) >> pt(1) >> pt(2);
        }
        
        ifstr.close();
        return 0;
    }
    
    int write_graph(const string fname, const Graph& graph)
    {
        return 0;
    }
}
