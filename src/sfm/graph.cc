#include "transform/transform.h"
#include "sfm/graph.h"
#include "utils/sort_index.h"

using std::vector;
using std::pair;

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
    
    Graph::Graph(const Graph& graph)
    {
        ncams_ = graph.ncams_;
        cams_ = graph.cams_;
        F_ = graph.F_;
        E_ = graph.E_;
        intrinsics_mat_ = graph.intrinsics_mat_;
        extrinsics_mat_ = graph.extrinsics_mat_;
        tracks_.resize(graph.tracks_.size(), Track());
        structure_points_.resize(graph.tracks_.size(), Structure_Point());
        for (int i = 0; i < graph.tracks_.size(); ++i)
        {
            tracks_[i] = graph.tracks_[i];
            structure_points_[i] = graph.structure_points_[i];
        }
    }
    
    Graph::~Graph()
    {
        cams_.clear();
        intrinsics_mat_.clear();
        extrinsics_mat_.clear();
        tracks_.clear();
        structure_points_.clear();
    }
    
    void Graph::init(const Pair& pair)
    {
        const size_t nmatches = pair.matches_.size();
        
        ncams_ = 2;
        cams_ = pair.cams_;
        F_ = pair.F_;
        E_ = pair.E_;
        intrinsics_mat_ = pair.intrinsics_mat_;
        extrinsics_mat_ = pair.extrinsics_mat_;
        
        tracks_.resize(nmatches);
        for (size_t i = 0; i < nmatches; ++i)
        {
            tracks_[i].add_keypoint(Keypoint(pair.matches_[i].point_.first, pair.cams_[0]));
            tracks_[i].add_keypoint(Keypoint(pair.matches_[i].point_.second, pair.cams_[1]));
        }
        
        structure_points_.resize(nmatches);
    }
    
    int Graph::index(int icam) const
    {
        for (int i = 0; i < ncams_; ++i)
        {
            if (icam == cams_[i])
                return i;
        }
        return -1;
    }
    
    int Graph::size() const
    {
        return static_cast<int>(cams_.size());
    }
    
    void Graph::rm_outliers()
    {
        
    }
    
    void Graph::add_track(const Track& track)
    {
        tracks_.push_back(track);
    }
    
    void Graph::rm_track(int index)
    {
        tracks_.erase(tracks_.begin() + index);
    }
    
    void Graph::add_struct_pt(const Structure_Point& struct_pt)
    {
        structure_points_.push_back(struct_pt);
    }
    
    void Graph::rm_struct_pt(int index)
    {
        structure_points_.erase(structure_points_.begin() + index);
    }
    
    void Graph::merge_tracks(Track& track1, const Track& track2, vector<pair<int, int> >& ind_key)
    {
        vector<int> ind;
        for (int i = 0; i < ind_key.size(); ++i)
            ind.push_back(ind_key[i].second);
        for (int i = 0; i < track2.size(); ++i)
        {
            auto iter = std::find(ind.begin(), ind.end(), i);
            if (iter == ind.end())
                track1.add_keypoint(track2[i]);
        }
    }
    
    void Graph::merge_graph(Graph &graph1, Graph &graph2)
    {
        vector<int>::iterator iter;
        
        // find overlapping cameras
        vector<int> cams_common(std::max(graph1.ncams_, graph2.ncams_));
        iter = std::set_intersection(graph1.cams_.begin(), graph1.cams_.end(), graph2.cams_.begin(), graph2.cams_.end(), cams_common.begin());
        cams_common.resize(iter - cams_common.begin());
        
        // find distinct cameras
        vector<int> cams_diff(std::max(graph1.ncams_, graph2.ncams_));
        iter = std::set_difference(graph2.cams_.begin(), graph2.cams_.end(), graph1.cams_.begin(), graph1.cams_.end(), cams_diff.begin());
        cams_diff.resize(iter - cams_diff.begin());
        
        if (cams_common.empty())
            return;
        
        // merge the camera intrinsic and extrinsic parameters using the first common camera
        int ind_cam1 = graph1.index(cams_common[0]);
        int ind_cam2 = graph2.index(cams_common[0]);
        Mat34f Rt1 = graph1.extrinsics_mat_[ind_cam1];
        Mat34f Rt2 = graph2.extrinsics_mat_[ind_cam2];
        Mat34f Rt21 = concat_Rt(Rt1, inv_Rt(Rt2));
        Mat34f Rt21_inv = inv_Rt(Rt21);
        for (int i = 0; i < graph2.structure_points_.size(); ++i)
        {
            Vec3f& pt3d = graph2.structure_points_[i].coords();
            pt3d = Rt21_inv * pt3d.homogeneous();
        }
        for (int i = 0; i < cams_diff.size(); ++i)
        {
            graph1.cams_.push_back(cams_diff[i]);
            graph1.intrinsics_mat_.push_back(graph2.intrinsics_mat_[graph2.index(cams_diff[i])]);
            graph1.extrinsics_mat_.push_back(concat_Rt(Rt21, graph2.extrinsics_mat_[graph2.index(cams_diff[i])]));
        }
        graph1.ncams_ = static_cast<int>(graph1.cams_.size());
        vector<size_t> indexes;
        sort<int>(graph1.cams_, graph1.cams_, indexes);
        reorder<Mat3f>(graph1.intrinsics_mat_, indexes, graph1.intrinsics_mat_);
        reorder<Mat34f>(graph1.extrinsics_mat_, indexes, graph1.extrinsics_mat_);
        
        const int ntracks = static_cast<int>(graph1.tracks_.size());
        for (int j = 0; j < graph2.tracks_.size(); ++j)
        {
            Track& track2 = graph2.tracks_[j];
            bool is_track_connected = false;
            for (int i = 0; i < ntracks; ++i)
            {
                Track& track1 = graph1.tracks_[i];
                vector<pair<int, int> > feats_common;
                Track::find_overlapping_keypoints(track1, track2, feats_common);
                // find overlapping feature tracks from common cameras
                if (!feats_common.empty())
                {
                    Graph::merge_tracks(track1, track2, feats_common);
                    is_track_connected = true;
                    // check if the structur_points are close, for debug purpose
                    if (false)
                    {
                        const Vec3f& pt1 = graph1.structure_points_[i].coords();
                        const Vec3f& pt2 = graph2.structure_points_[j].coords();
                        std::cout << "graph1 structure_point: " << std::endl << pt1/pt1(2) << std::endl;
                        std::cout << "graph2 structure_point: " << std::endl << pt2/pt2(2) << std::endl;
                    }
                    break;
                }
            }
            // find non-overlapping feature tracks from common cameras
            if (!is_track_connected)
            {
                graph1.add_track(track2);
                graph1.add_struct_pt(graph2.structure_points_[j]);
            }
        }

        // find new features from non-overlapping cameras, this part is not needed for now
    }
    
    bool Graph::operator<(const Graph& rhs) const
    {
        return (cams_[0] < rhs.cams_[0]) && (cams_[1] < rhs.cams_[1]);
    }
}
