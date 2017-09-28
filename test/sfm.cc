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
#include "io/match_io.h"

using namespace std;
using namespace open3DCV;

int main(const int argc, const char** argv)
{
    string idir = "/Users/BlacKay/Documents/Projects/Images/test/bust/";
    bool is_vis = true;
    
    // -------------------------------------------------
    // read images
    // -------------------------------------------------
    const int nimages = 5;
    vector<string> inames(nimages);
    inames[0] = idir + "B21.jpg";
    inames[1] = idir + "B22.jpg";
    inames[2] = idir + "B23.jpg";
    inames[3] = idir + "B24.jpg";
    inames[4] = idir + "B25.jpg";
    vector<Image> images(nimages);
    for (int i = 0; i < nimages; ++i)
    {
        images[i].read(inames[i]);
    }
    
    // -------------------------------------------------
    // feature detection, descriptor extraction
    // -------------------------------------------------
    Sift_Params sift_param(3, 3, 0, 10.0f / 255.0, 0, -INFINITY, 3, 2);
    Sift sift(sift_param);
    vector<vector<Keypoint> > keys(nimages, vector<Keypoint>());
    vector<vector<Vecf> > descs(nimages, vector<Vecf>());
    for (int i = 0; i < nimages; ++i)
    {
        sift.detect_keypoints(images[i], keys[i], 0);
        sift.extract_descriptors(images[i], keys[i], descs[i]);
        sift.clear();
        if (/* DISABLES CODE */ (false))
        {
            draw_cross(images[i], keys[i], "feature"+to_string(i+1));
        }
    }
    
    // -------------------------------------------------
    // feature matching
    // -------------------------------------------------
    Matcher_Param matcher_param;
    Matcher_Flann matcher(matcher_param);
    vector<vector<DMatch> > matches(nimages - 1, vector<DMatch>());
    for (int i = 0; i < nimages - 1; ++i)
    {
        matcher.match(descs[i], descs[i+1], matches[i]);
        if (/* DISABLES CODE */ (false))
        {
            draw_matches(images[i], keys[i], images[i+1], keys[i+1], matches[i], "matching"+to_string(i+1)+"_"+to_string(i+2));
        }
    }
    
    // -------------------------------------------------
    // 2-view SfM
    // -------------------------------------------------
    vector<std::pair<Vec2f, Vec2f> > data; // variable to store the coordinates of matching features
    vector<Graph> graph(nimages - 1);
    for (int i = 0; i < nimages - 1; ++i)
    {
        cout << "*******************************" << endl;
        cout << " 2-View SfM of image " << i << " and " << i + 1 << endl;
        cout << "*******************************" << endl;
        
        // ------ image pair ------
        Pair pair(i, i+1);
        int nmatches = static_cast<int>(matches[i].size());
        for (int j = 0; j < nmatches; ++j)
        {
            Vec2f x1 = keys[i][matches[i][j].ikey1_].coords();
            Vec2f x2 = keys[i+1][matches[i][j].ikey2_].coords();
            std::pair<Vec2f, Vec2f> pair_data;
            pair_data.first = x1;
            pair_data.second = x2;
            data.push_back(pair_data);
        }
        int *vote_inlier = new int[nmatches];
        std::fill(vote_inlier, vote_inlier + nmatches, 1);
        pair.update_matches(data, vote_inlier);
        
        // ------ estimate Fundamental matrix ------
        vector<float> params(9);
        Param_Estimator<std::pair<Vec2f, Vec2f>, float>* fund_esti = new open3DCV::Fundamental_Estimator(10e-8);
        float ratio_inlier = Ransac<std::pair<Vec2f, Vec2f>, float>::estimate(fund_esti, pair.matches_, params, 0.99, vote_inlier);
        std::cout << "ratio of matching inliers: " << ratio_inlier << std::endl;
        pair.F_ << params[0], params[3], params[6],
                   params[1], params[4], params[7],
                   params[2], params[5], params[8];
        
        // remove outliers
        pair.update_matches(data, vote_inlier);
        std::cout << "number of matches: " << pair.matches_.size() << std::endl;
        // visualize matching inliers
        if (is_vis)
        {
            draw_matches(images[i], images[i+1], pair.matches_, "matching_inlier"+to_string(i+1)+"_"+to_string(i+2));
        }
        // visualize epipolar geometry
        if (is_vis)
        {
            draw_epipolar_geometry(images[i], images[i+1], pair.F_, pair.matches_, "epipolar"+to_string(i+1)+"_"+to_string(i+2));
        }
        
        // ------ estimate relative pose ------
        const float f = 719.5459;
        const int w = 480, h = 640;
        pair.update_intrinsics(f, w, h);
        pair.E_ = pair.intrinsics_mat_[1].transpose() * pair.F_ * pair.intrinsics_mat_[0];
        Rt_from_E(pair);
        
        // ------ init graph from pair ------
        graph[i].init(pair);
        
        // ------ triangulate ------
        triangulate_nonlinear(graph[i]);
        
        // compute reprojection error
        float error = reprojection_error(graph[i]);
        std::cout << "reprojection error (before bundle adjustment): " << error << std::endl;
        
        // ------ bundle adjustment ------
        cout << "------ start bundle adjustment ------" << endl;
        Open3DCVBundleAdjustment(graph[i], BUNDLE_PRINCIPAL_POINT);
        cout << "------ end bundle adjustment ------" << endl;
        error = reprojection_error(graph[i]);
        std::cout << "reprojection error (after bundle adjustment): " << error << std::endl;
        
        delete [] vote_inlier;
        data.clear();
    }
    
    // -------------------------------------------------
    // N-view SfM
    // -------------------------------------------------
    Graph global_graph(graph[0]);
    for (int i = 1; i < nimages - 1; ++i)
    {
        cout << "*******************************" << endl;
        cout << " N-View SfM: merging image " << i << endl;
        cout << "*******************************" << endl;
        // ------ merge graphs ------
        Graph::merge_graph(global_graph, graph[i]);
        
        // ------ triangulation ------
        triangulate_nonlinear(global_graph);
        float error = reprojection_error(global_graph);
        std::cout << "reprojection error (before bundle adjustment): " << error << std::endl;
        
        // ------ bundle adjustment ------
        cout << "------ start bundle adjustment ------" << endl;
        Open3DCVBundleAdjustment(global_graph, BUNDLE_PRINCIPAL_POINT);
        cout << "------ end bundle adjustment ------" << endl;
        error = reprojection_error(global_graph);
        std::cout << "reprojection error (after bundle adjustment): " << error << std::endl;
    }
    
    // -------------------------------------------------
    // Output
    // -------------------------------------------------
    
    return 0;
}
