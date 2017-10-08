#include <fstream>
#include "io/sfm_io.h"

using std::endl;
namespace open3DCV
{
    void read_sfm(const std::string fname, Graph& graph)
    {
        std::ifstream ifstr;
        ifstr.open(fname.c_str());
        if (!ifstr.is_open())
        {
            std::cerr << "Cannot open the file." << std::endl;
            return;
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
    }
    
    void write_sfm(const Graph& graph)
    {
        std::ofstream ofstr;
        char oname[20];
        for (int i = 0; i < graph.ncams_; ++i)
        {
            sprintf(oname, "%08d.txt", i);
            ofstr.open(&oname[0], std::ofstream::out);
            const Mat3f& K = graph.intrinsics_mat_[i];
            const Mat34f& Rt = graph.extrinsics_mat_[i];
            ofstr << K(0, 0) << " " << K(0, 1) << " " << K(0, 2) << endl
                  << K(1, 0) << " " << K(1, 1) << " " << K(1, 2) << endl
                  << K(2, 0) << " " << K(2, 1) << " " << K(2, 2) << endl
                  << Rt(0, 0) << " " << Rt(0, 1) << " " << Rt(0, 2) << " " << Rt(0, 3) << endl
                  << Rt(1, 0) << " " << Rt(1, 1) << " " << Rt(1, 2) << " " << Rt(1, 3) << endl
                  << Rt(2, 0) << " " << Rt(2, 1) << " " << Rt(2, 2) << " " << Rt(2, 3);
            ofstr.close();
            
            sprintf(oname, "txt/%08d.txt", i);
            ofstr.open(&oname[0], std::ofstream::out);
            const Mat34f& pose = graph.intrinsics_mat_[i] * graph.extrinsics_mat_[i];
            ofstr << "CONTOUR" << endl
                  << pose(0, 0) << " " << pose(0, 1) << " " << pose(0, 2) << " " << pose(0, 3) << endl
                  << pose(1, 0) << " " << pose(1, 1) << " " << pose(1, 2) << " " << pose(1, 3) << endl
                  << pose(2, 0) << " " << pose(2, 1) << " " << pose(2, 2) << " " << pose(2, 3);
            ofstr.close();
        }
    }
}
