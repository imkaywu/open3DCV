#include "math/numeric.h"
#include "image/image.h"
#include "camera/camera.h"
#include "keypoint/sift.h"
#include "matching/matcher_flann.h"
#include "matching/pair.h"
#include "estimator/fundamental.h"
#include "estimator/ransac.h"
#include "estimator/est_Rt_from_E.h"
#include "triangulation/triangulation.h"
#include "sfm/bundle_adjust.h"
#include "sfm/graph.h"
#include "viz/plot.h"
#include "io/keypoint_io.h"
#include "io/match_io.h"
#include "io/sfm_io.h"

using namespace std;
using namespace open3DCV;

int main(const int argc, const char** argv)
{
    string odir = "templeRing/";
    string idir = "/Users/BlacKay/Documents/Projects/Images/test/sfm/"+odir;
    bool is_vis = true;
    bool read_from_file = true;
    const float bs_angle_thre[2] = {3.0 / 180.0 * M_PI, 70.0 / 180.0f * M_PI};
    
    // -------------------------------------------------
    // read images
    // -------------------------------------------------
    const int nimages = 10;
    char iname[100];
    vector<Image> images(nimages);
    for (int i = 0; i < nimages; ++i)
    {
//        sprintf(iname, "%s/B%02d.jpg", idir.c_str(), 21+i);
        sprintf(iname, "%s/%08d.jpg", idir.c_str(), i+1);
        images[i].read(iname);
    }
    
    // -------------------------------------------------
    // feature detection, descriptor extraction
    // -------------------------------------------------
    Sift_Params sift_param(3, 3, 0, 10.0f/255, 1, -INFINITY, 3, 2);
    Sift sift(sift_param);
    vector<vector<Keypoint> > keys(nimages, vector<Keypoint>());
    vector<vector<Vecf> > descs(nimages, vector<Vecf>());
    if (!read_from_file)
    {
        for (int i = 0; i < nimages; ++i)
        {
            sift.detect_keypoints(images[i], keys[i], 0);
            sift.extract_descriptors(images[i], keys[i], descs[i]);
            sift.clear();
            string fname = odir+"feature"+to_string(i+1)+".txt";
            write_keypoints(fname, keys[i]);
            if ((false))
            {
                draw_cross(images[i], keys[i], odir+"feature"+to_string(i+1));
            }
            cout << "Image " << i + 1 << ": " << "features number: " << keys[i].size() << endl;
        }
    }
    else
    {
        for (int i = 0; i < nimages; ++i)
        {
            string fname = odir+"feature"+to_string(i+1)+".txt";
            read_keypoints(fname, keys[i]);
        }
    }
    
    // -------------------------------------------------
    // feature matching pairwise
    // -------------------------------------------------
    vector<Pair> pairs;
    Matcher_Param matcher_param(0.6*0.6, 50);
    Matcher_Flann matcher(matcher_param);
    vector<vector<vector<DMatch> > > matches_pairwise(nimages-1, vector<vector<DMatch> >());
    if (!read_from_file)
    {
        for (int i = 0; i < nimages-1; ++i)
        {
            matches_pairwise[i].resize(nimages-(i+1));
            for (int j = i+1; j < nimages; ++j)
            {
                vector<DMatch>& matches = matches_pairwise[i][j-(i+1)];
                matcher.match(descs[i], descs[j], matches);
                matcher.matching_keys(keys[i], keys[j], matches);
                pairs.push_back(Pair(i, j, matches));
                string fname = odir+"matching"+to_string(i+1)+"_"+to_string(j+1)+".txt";
                write_matches(fname, matches);
                if ((false))
                {
                    draw_matches(images[i], keys[i], images[j], keys[j], matches, odir+"matching"+to_string(i+1)+"_"+to_string(j+1));
                }
                cout << "Image (" << i+1 << ", " << j+1 << "): matches number: " << matches.size() << endl;
            }
        }
    }
    else
    {
        for (int i = 0; i < nimages - 1; ++i)
        {
            matches_pairwise[i].resize(nimages-(i+1));
            for (int j = i+1; j < nimages; ++j)
            {
                vector<DMatch>& matches = matches_pairwise[i][j-(i+1)];
                string fname = odir+"matching"+to_string(i+1)+"_"+to_string(j+1)+".txt";
                read_matches(fname, matches);
                pairs.push_back(Pair(i, j, matches));
            }
        }
    }
    sort(pairs.begin(), pairs.end()); // sort pairs based on number of matches
    
    // -------------------------------------------------
    // 2-view SfM
    // -------------------------------------------------
    vector<Graph> graphs;
    const float reproj_error_thre = 1.0;
    const float ratio_inlier_thre = 0.6;
    for (int i = 0; i < pairs.size(); ++i)
    {
        // ------ image pair ------
        Pair& pair = pairs[i];
        int nmatches = static_cast<int>(pair.matches_.size());
        const int& ind1 = pair.cams_[0];
        const int& ind2 = pair.cams_[1];
        cout << "*******************************" << endl;
        cout << " 2-View SfM of image " << ind1+1 << " and " << ind2+1 << endl;
        cout << "*******************************" << endl;
        
        // ------ estimate Fundamental matrix ------
        vector<float> params(9);
        int *vote_inlier = new int[nmatches];
        Param_Estimator<DMatch, float>* fund_esti = new open3DCV::Fundamental_Estimator(10e-8);
        float ratio_inlier = Ransac<DMatch, float>::estimate(fund_esti, pair.matches_, params, 0.99, vote_inlier);
        std::cout << "ratio of matching inliers: " << ratio_inlier << std::endl;
        if (ratio_inlier < ratio_inlier_thre)
        {
            delete [] vote_inlier;
            continue;
        }
        pair.F_ << params[0], params[3], params[6],
                   params[1], params[4], params[7],
                   params[2], params[5], params[8];
        
        // remove outliers
        pair.update_matches(vote_inlier);
        std::cout << "number of matches: " << pair.matches_.size() << std::endl;
        
        // ------ estimate relative pose ------
//        const float f = 719.5459;
//        const int w = 480, h = 640;
        const float f = 1520.4;
        const int w = 302.32*2, h = 246.87*2;
        pair.update_intrinsics(f, w, h);
        pair.E_ = pair.intrinsics_mat_[1].transpose() * pair.F_ * pair.intrinsics_mat_[0];
        Rt_from_E(pair);
        
        // ------ filter pair using baseline angle ------
        float angle = pair.baseline_angle();
        if (angle < bs_angle_thre[0] || angle > bs_angle_thre[1])
            continue;
        
        // ------ init graph from pair ------
        Graph graph(pair);
        
        // ------ triangulate ------
        triangulate_nonlinear(graph);
        
        // compute reprojection error
        float error = reprojection_error(graph);
        std::cout << "reprojection error (before bundle adjustment): " << error << std::endl;
        
        // ------ bundle adjustment ------
        cout << "------ start bundle adjustment ------" << endl;
        Open3DCVBundleAdjustment(graph, BUNDLE_INTRINSICS);
        cout << "------ end bundle adjustment ------" << endl;
        error = reprojection_error(graph);
        std::cout << "reprojection error (after bundle adjustment): " << error << std::endl;
        
        // ------ filter pair using baseline angle ------
        angle = graph.baseline_angle();
        if (angle < bs_angle_thre[0] || angle > bs_angle_thre[1])
            continue;
        
        delete [] vote_inlier;
        
        if (error > reproj_error_thre)
            continue;
        
        if (isnan(error))
        {
            error = reprojection_error(graph);
            continue;
        }
        
        graphs.push_back(graph);
        
        // visualize matching inliers
        if ((false))
        {
            draw_matches(images[ind1], images[ind2], pair.matches_, odir+"matching_inlier"+to_string(ind1+1)+"_"+to_string(ind2+1));
        }
        // visualize epipolar geometry
        if (is_vis)
        {
            draw_epipolar_geometry(images[ind1], images[ind2], pair.F_, pair.matches_, odir+"epipolar"+to_string(ind1+1)+"_"+to_string(ind2+1));
        }
    }
    sort(graphs.begin(), graphs.end());
    
    // -------------------------------------------------
    // debug
    // -------------------------------------------------
    if ((false))
    {
        for (int i = 0; i < graphs.size(); ++i)
        {
            const Graph& graph = graphs[i];
            const Mat3f rot_mat = graph.extrinsics_mat_[1].block<3, 3>(0, 0);
            Vec3f om;
            irodrigues(om, nullptr, rot_mat);
            cout << "rotation axis:  " << endl << om / om.norm() << endl;
            cout << "rotation angle: " << endl << om.norm() << endl;
        }
    }
    
    // -------------------------------------------------
    // N-view SfM
    // -------------------------------------------------
    // find a more elegant way to deal with the first graph
    vector<int> merged_graph(graphs.size());
    fill(merged_graph.begin(), merged_graph.end(), 0);
    merged_graph[0] = 1;
    Graph global_graph(graphs[0]);
    int icam = 0;
    while ((icam = Graph::find_next_graph(graphs, global_graph, merged_graph)) > 0)
    {
        cout << "*******************************" << endl;
        cout << " N-View SfM: merging graph " << icam+1 << endl;
        cout << "*******************************" << endl;
        
        // ------ merge graphs ------
        Graph::merge_graph(global_graph, graphs[icam]);
        
        // ------ N-view triangulation ------
        triangulate_nonlinear(global_graph);
        float error = reprojection_error(global_graph);
        std::cout << "reprojection error (before bundle adjustment): " << error << std::endl;
        
        // ------ N-view bundle adjustment ------
        cout << "------ start bundle adjustment ------" << endl;
        Open3DCVBundleAdjustment(global_graph, BUNDLE_INTRINSICS);
        cout << "------ end bundle adjustment ------" << endl;
        error = reprojection_error(global_graph);
        std::cout << "reprojection error (after bundle adjustment): " << error << std::endl;
        
        if (isnan(error))
        {
            error = reprojection_error(global_graph);
        }
    }
    
    // -------------------------------------------------
    // Output
    // -------------------------------------------------
    write_sfm(global_graph);
    return 0;
}
