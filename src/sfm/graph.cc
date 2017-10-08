#include "transform/transform.h"
#include "transform/rodrigues.h"
#include "sfm/graph.h"
#include "utils/sort_index.h"
#include "triangulation/triangulation.h"

using std::vector;
using std::pair;

namespace open3DCV
{
    Graph::Graph()
    {
        ncams_ = 0;
        cams_ = vector<int>();
        intrinsics_mat_ = vector<Mat3f>();
        extrinsics_mat_ = vector<Mat34f>();
        tracks_ = vector<Track>();
        structure_points_ = vector<Structure_Point>();
    }
    
    Graph::Graph(const Pair& pair)
    {
        init(pair);
    }
    
    Graph::Graph(const Graph& graph)
    {
        init(graph);
    }
    
    Graph& Graph::operator=(const Graph& graph)
    {
        init(graph);
        return *this;
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
    
    void Graph::init(const Graph& graph)
    {
        ncams_ = graph.ncams_;
        cams_ = graph.cams_;
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
    
    int Graph::index(int icam) const
    {
        for (int i = 0; i < ncams_; ++i)
        {
            if (icam == cams_[i])
                return i;
        }
        return -1;
    }
    
    int Graph::sz_cams() const
    {
        return static_cast<int>(cams_.size());
    }
    
    int Graph::sz_tracks() const
    {
        return static_cast<int>(tracks_.size());
    }
    
    void Graph::rm_outliers(const float thresh_reproj, const float thresh_angle)
    {
        const int ntracks = static_cast<int>(tracks_.size());
        const float thresh_cos_angle = std::cos(thresh_angle * M_PI / 180.0);
        vector<int> rm_reproj, rm_angle;
        
        vector<Vec3f> cam_center(sz_cams());
        for (int i = 0; i < sz_cams(); ++i)
        {
            Mat34f ext_mat = extrinsics_mat_[i];
            cam_center[i] = -ext_mat.block<3, 3>(0, 0).transpose() * ext_mat.block<3, 1>(0, 3);
        }
        
        bool removed = false;
        for (int i = 0; i < ntracks; ++i)
        {
            removed = false;
            const Vec3f& pt3d = structure_points_[i].coords();
            const Track& track = tracks_[i];
            for (int j = 0; j < track.size(); ++j)
            {
                int ind_cam = index(track[j].index());
                Vec3f x = intrinsics_mat_[ind_cam] * extrinsics_mat_[ind_cam] * pt3d.homogeneous();
                Vec2f dx = x.head<2>() / x(2) - track[j].coords();
                if (dx.dot(dx) > thresh_reproj)
                {
                    rm_reproj.push_back(i);
                    removed = true;
                    break;
                }
            }
            
            if (removed)
                continue;
            
            for (int m = 0; m < track.size(); ++m)
            {
                for (int n = 0; n < track.size(); ++n)
                {
                    if (m == n)
                        continue;
                    
                    int ind_cam1 = index(track[m].index());
                    int ind_cam2 = index(track[n].index());
                    Vec3f dir_cam1 = pt3d - cam_center[ind_cam1];
                    Vec3f dir_cam2 = pt3d - cam_center[ind_cam2];
                    dir_cam1.normalize();
                    dir_cam2.normalize();
                    if (dir_cam1.dot(dir_cam2) > thresh_cos_angle)
                    {
                        rm_angle.push_back(i);
                        removed = true;
                        break;
                    }
                }
                if (removed)
                    break;
            }
        }
        std::sort(rm_reproj.begin(), rm_reproj.end());
        for (int i = static_cast<int>(rm_reproj.size()) - 1; i >= 0; --i)
        {
            rm_struct_pt(rm_reproj[i]);
            rm_track(rm_reproj[i]);
        }
        std::cout << "remove " << rm_reproj.size() << " outliers out of " << ntracks << " points with reprojection error bigger than "
                  << thresh_reproj << " pixels" << std::endl;
        rm_reproj.clear();
        
        /* for image 5, 6, the fundamental matrix estimation is not accurate enough
        std::sort(rm_angle.begin(), rm_angle.end());
        for (int i = static_cast<int>(rm_angle.size()) - 1; i >= 0; --i)
        {
            rm_struct_pt(rm_angle[i]);
            rm_track(rm_angle[i]);
        }
        std::cout << "remove " << rm_angle.size() << " outliers out of " << ntracks << " points with view angle less than "
                  << thresh_angle << " degrees" << std::endl;
        rm_angle.clear();
         */
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
        vector<int> cams_common(std::min(graph1.ncams_, graph2.ncams_));
        iter = std::set_intersection(graph1.cams_.begin(), graph1.cams_.end(), graph2.cams_.begin(), graph2.cams_.end(), cams_common.begin());
        cams_common.resize(iter - cams_common.begin());
        
        // find distinct cameras
        vector<int> cams_diff(std::min(graph1.ncams_, graph2.ncams_));
        iter = std::set_difference(graph2.cams_.begin(), graph2.cams_.end(), graph1.cams_.begin(), graph1.cams_.end(), cams_diff.begin());
        cams_diff.resize(iter - cams_diff.begin());
        
        if (cams_common.empty())
            return;
        
        // merge the camera intrinsic and extrinsic parameters using the first common camera
        int ind_cam1 = graph1.index(cams_common[0]);
        int ind_cam2 = graph2.index(cams_common[0]);
        Mat34f Rt1 = graph1.extrinsics_mat_[ind_cam1];
        Mat34f Rt2 = graph2.extrinsics_mat_[ind_cam2];
        // solution 1
//        Mat34f Rt21 = concat_Rt(inv_Rt(Rt1), Rt2);
//        Mat34f Rt21_inv = inv_Rt(Rt21);
//        for (int i = 0; i < graph2.structure_points_.size(); ++i)
//        {
//            Vec3f& pt3d = graph2.structure_points_[i].coords();
//            pt3d = Rt21.block<3, 3>(0, 0) * pt3d + Rt21.block<3, 1>(0, 3);
//        }
//        for (int i = 0; i < cams_diff.size(); ++i)
//        {
//            graph1.cams_.push_back(cams_diff[i]);
//            const int ind_cam = graph2.index(cams_diff[i]);
//            graph1.intrinsics_mat_.push_back(graph2.intrinsics_mat_[ind_cam]);
//            graph1.extrinsics_mat_.push_back(concat_Rt(graph2.extrinsics_mat_[ind_cam], Rt21_inv));
//        }
        // solution 2
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
            const int ind_cam = graph2.index(cams_diff[i]);
            graph1.intrinsics_mat_.push_back(graph2.intrinsics_mat_[ind_cam]);
            graph1.extrinsics_mat_.push_back(concat_Rt(Rt21, graph2.extrinsics_mat_[ind_cam]));
        }
        // debug
        if ((false))
        {
            Mat34f ext_mat1 = concat_Rt(graph2.extrinsics_mat_[0], Rt21_inv);
            Mat34f ext_mat2 = concat_Rt(graph2.extrinsics_mat_[1], Rt21_inv);
            Mat34f pose1 = graph2.intrinsics_mat_[0] * ext_mat1;
            Mat34f pose2 = graph2.intrinsics_mat_[1] * ext_mat2;
            for (int i = 0; i < graph2.structure_points_.size(); ++i)
            {
                const Vec3f& pt3d = graph2.structure_points_[i].coords();
                Vec3f pt1 = pose1 * pt3d.homogeneous();
                Vec3f pt2 = pose2 * pt3d.homogeneous();
                const Vec2f& key1 = graph2.tracks_[i][0].coords();
                const Vec2f& key2 = graph2.tracks_[i][1].coords();
                Vec2f dx = (pt1.head<2>() / pt1(2) - key1) + (pt2.head<2>() / pt2(2) - key2);
                std::cout << "reprojection error: " << dx.dot(dx) << std::endl;
            }
        }
        graph1.ncams_ = static_cast<int>(graph1.cams_.size());
//        vector<size_t> indexes;
//        sort<int>(graph1.cams_, graph1.cams_, indexes);
//        reorder<Mat3f>(graph1.intrinsics_mat_, indexes, graph1.intrinsics_mat_);
//        reorder<Mat34f>(graph1.extrinsics_mat_, indexes, graph1.extrinsics_mat_);
        
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
                    // check if the structur_points are close, for debug purpose
                    if ((false))
                    {
                        const Vec3f& pt1 = graph1.structure_points_[i].coords();
                        const Vec3f& pt2 = graph2.structure_points_[j].coords();
                        std::cout << "graph1 structure_point: " << std::endl << pt1/pt1(2) << std::endl;
                        std::cout << "graph2 structure_point: " << std::endl << pt2/pt2(2) << std::endl;
                        
                        if ((false))
                        {
                            std::cout << "re-triangulation" << std::endl;
                            vector<Mat34f> poses(2);
                            poses[0] = graph1.intrinsics_mat_[0] * graph1.extrinsics_mat_[0];
                            poses[1] = graph1.intrinsics_mat_[1] * graph1.extrinsics_mat_[1];
                            Structure_Point struct_pt1;
                            triangulate_nonlinear(poses, track1, struct_pt1);
                            const Vec3f& pt3 = struct_pt1.coords();
                            std::cout << "graph1 structure_point: " << std::endl << pt3/pt3(2) << std::endl;
                            
                            poses[0] = graph2.intrinsics_mat_[0] * concat_Rt(graph2.extrinsics_mat_[0], Rt21_inv);
                            poses[1] = graph2.intrinsics_mat_[1] * concat_Rt(graph2.extrinsics_mat_[1], Rt21_inv);
                            Structure_Point struct_pt2;
                            triangulate_nonlinear(poses, track2, struct_pt2);
                            const Vec3f& pt4 = struct_pt2.coords();
                            std::cout << "graph2 structure_point: " << std::endl << pt4/pt4(2) << std::endl;
                            
                            Track track3(track1);
                            Graph::merge_tracks(track3, track2, feats_common);
                            std::sort(track3.key_begin(), track3.key_end());
                            poses[0] = graph1.intrinsics_mat_[0] * graph1.extrinsics_mat_[0];
                            poses[1] = graph1.intrinsics_mat_[1] * graph1.extrinsics_mat_[1];
                            Mat34f pose3 = graph1.intrinsics_mat_[2] * graph1.extrinsics_mat_[2];
                            poses.push_back(pose3);
                            Structure_Point struct_pt3;
                            triangulate_nonlinear(poses, track3, struct_pt3);
                            const Vec3f& pt5 = struct_pt3.coords();
                            std::cout << "graph merged structure_point: " << std::endl << pt5/pt5(2) << std::endl;
                        }
                    }
                    // shoud check if track contains keypoints that are not in the camera view
                    Graph::merge_tracks(track1, track2, feats_common);
                    is_track_connected = true;
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
        
        // re-triangulation
        triangulate_nonlinear(graph1);
        
        // find new features from non-overlapping cameras, this part is not needed for sequential SfM
    }
    
    bool Graph::operator<(const Graph& rhs) const
    {
        return tracks_.size() > rhs.tracks_.size();
    }
    
    // for 2-view graphs only
    float Graph::baseline_angle() const
    {
        Mat3f rotation = extrinsics_mat_[1].block<3, 3>(0, 0);
        Vec3f om;
        irodrigues(om, nullptr, rotation);
        return om.norm();
    }
    
    int Graph::find_next_graph(const vector<Graph>& graphs, const Graph& graph, std::vector<int>& merged_graph)
    {
        int count_max = 0;
        int ind_selected = -1;
        for (int i = 0; i < graphs.size(); ++i)
        {
            if (merged_graph[i])
                continue;
            
            const Graph& graph1 = graphs[i];
            vector<int> cams_common(std::min(graph1.ncams_, graph.ncams_));
            auto iter = std::set_intersection(graph1.cams_.begin(), graph1.cams_.end(), graph.cams_.begin(), graph.cams_.end(), cams_common.begin());
            cams_common.resize(iter - cams_common.begin());
            
            if (cams_common.empty())
                continue;
            
            int count = 0;
            vector<int> tracks_common(std::min(graph1.tracks_.size(), graph.tracks_.size()));
            for (int m = 0; m < graph.tracks_.size(); ++m)
            {
                for (int n = 0; n < graph1.tracks_.size(); ++n)
                {
                    const Track& track = graph.tracks_[m];
                    const Track& track1 = graph1.tracks_[n];
                    if (Track::has_overlapping_keypoints(track, track1))
                        ++count;
                }
            }
            if (count > count_max)
            {
                count_max = count;
                ind_selected = i;
            }
        }
        if (ind_selected > 0)
            merged_graph[ind_selected] = 1;
        return ind_selected;
    }
    
    void Graph::report_graph(const Graph& graph)
    {
        for (int i = 0; i < graph.tracks_.size(); ++i)
        {
            const Track& track = graph.tracks_[i];
            std::cout << track.size() << " ";
            for (int j = 0; j < track.size(); ++j)
            {
                const Keypoint& key = track[j];
                std::cout << key << " ";
            }
            std::cout << std::endl;
        }
    }
}
