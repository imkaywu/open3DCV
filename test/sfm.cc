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
#include "io/sfm_io.h"

using namespace std;
using namespace open3DCV;

int main(const int argc, const char** argv)
{
    string idir = "/Users/BlacKay/Documents/Projects/Images/test/templeRing";
    string odir = "temple/";
    bool is_vis = true;
    
    // -------------------------------------------------
    // read images
    // -------------------------------------------------
    const int nimages = 8;
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
    for (int i = 0; i < nimages; ++i)
    {
        sift.detect_keypoints(images[i], keys[i], 0);
        sift.extract_descriptors(images[i], keys[i], descs[i]);
        sift.clear();
        if (is_vis)
        {
            draw_cross(images[i], keys[i], odir+"feature"+to_string(i+1));
        }
    }
    
    // -------------------------------------------------
    // feature matching pairwise
    // -------------------------------------------------
    vector<Pair> pairs;
    Matcher_Param matcher_param(0.6*0.6, 30);
    Matcher_Flann matcher(matcher_param);
    vector<vector<vector<DMatch> > > matches_pairwise(nimages-1, vector<vector<DMatch> >());
    for (int i = 0; i < nimages-1; ++i)
    {
        matches_pairwise[i].resize(nimages-(i+1));
        for (int j = i+1; j < nimages; ++j)
        {
            matcher.match(descs[i], descs[j], matches_pairwise[i][j-(i+1)]);
            pairs.push_back(Pair(i, j, matches_pairwise[i][j-(i+1)]));
            if (is_vis)
            {
                draw_matches(images[i], keys[i], images[j], keys[j], matches_pairwise[i][j-(i+1)], odir+"matching"+to_string(i+1)+"_"+to_string(j+1));
            }
        }
    }
    sort(pairs.begin(), pairs.end()); // sort pairs based on number of matches
    
//    vector<Pair> pairs;
//    vector<vector<DMatch> > matches;
//    for (int i = 0; i < nimages; ++i)
//    {
//        std::pair<int, int> ind_pair;
//        float dist_min = INFINITY;
//        for (int j = 0; j < nimages; ++j)
//        {
//            if (i == j)
//                continue;
//            float dist = 0.0f;
//            vector<DMatch>& dmatches = i < j ? matches_pairwise[i][j-(i+1)] : matches_pairwise[j][i-(j+1)];
//            for (int n = 0; n < dmatches.size(); ++n)
//            {
//                dist += dmatches[n].dist_;
//            }
//            dist /= dmatches.size();
//            if (dist < dist_min)
//            {
//                dist_min = dist;
//                ind_pair.first = i;
//                ind_pair.second = j;
//            }
//        }
//        pairs.push_back(Pair(ind_pair.first, ind_pair.second));
//        const int& ind1 = ind_pair.first;
//        const int& ind2 = ind_pair.second;
//        vector<DMatch>& dmatches = ind1 < ind2 ? matches_pairwise[ind1][ind2-(ind1+1)] : matches_pairwise[ind2][ind1-(ind2+1)];
//        matches.push_back(dmatches);
//    }
    
    // -------------------------------------------------
    // 2-view SfM
    // -------------------------------------------------
    vector<DMatch> data; // variable to store the coordinates of matching features
    vector<Graph> graphs;
    const int reproj_error_thre = 4;
    for (int i = 0; i < pairs.size(); ++i)
    {
        cout << "*******************************" << endl;
        cout << " 2-View SfM of image " << i << " and " << i + 1 << endl;
        cout << "*******************************" << endl;
        
        // ------ image pair ------
        Pair& pair = pairs[i];
        const int& ind1 = pair.cams_[0];
        const int& ind2 = pair.cams_[1];
        int nmatches = static_cast<int>(pair.matches_.size());
        for (int j = 0; j < nmatches; ++j)
        {
            Vec2f x1 = keys[ind1][pair.matches_[j].ind_key_.first].coords();
            Vec2f x2 = keys[ind2][pair.matches_[j].ind_key_.second].coords();
            DMatch pair_data;
            pair_data.point_.first = x1;
            pair_data.point_.second = x2;
            data.push_back(pair_data);
        }
        int *vote_inlier = new int[nmatches];
        std::fill(vote_inlier, vote_inlier + nmatches, 1);
        pair.update_matches(data, vote_inlier);
        
        // ------ estimate Fundamental matrix ------
        vector<float> params(9);
        Param_Estimator<DMatch, float>* fund_esti = new open3DCV::Fundamental_Estimator(10e-8);
        float ratio_inlier = Ransac<DMatch, float>::estimate(fund_esti, pair.matches_, params, 0.99, vote_inlier);
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
            draw_matches(images[i], images[i+1], pair.matches_, odir+"matching_inlier"+to_string(i+1)+"_"+to_string(i+2));
        }
        // visualize epipolar geometry
        if (is_vis)
        {
            draw_epipolar_geometry(images[i], images[i+1], pair.F_, pair.matches_, odir+"epipolar"+to_string(i+1)+"_"+to_string(i+2));
        }
        
        // ------ estimate relative pose ------
//        const float f = 719.5459;
//        const int w = 480, h = 640;
        const float f = 1520.4;
        const int w = 302.32*2, h = 246.87*2;
        pair.update_intrinsics(f, w, h);
        pair.E_ = pair.intrinsics_mat_[1].transpose() * pair.F_ * pair.intrinsics_mat_[0];
        Rt_from_E(pair);
        
        // ------ init graph from pair ------
        Graph graph(pair);
        
        // ------ triangulate ------
        triangulate_nonlinear(graph);
        
        // compute reprojection error
        float error = reprojection_error(graph);
        std::cout << "reprojection error (before bundle adjustment): " << error << std::endl;
        
        if (error > reproj_error_thre)
            continue;
        
        // ------ bundle adjustment ------
        cout << "------ start bundle adjustment ------" << endl;
        Open3DCVBundleAdjustment(graph, BUNDLE_PRINCIPAL_POINT);
        cout << "------ end bundle adjustment ------" << endl;
        error = reprojection_error(graph);
        std::cout << "reprojection error (after bundle adjustment): " << error << std::endl;
        graphs.push_back(graph);
        
        delete [] vote_inlier;
        data.clear();
    }
    
    // -------------------------------------------------
    // N-view SfM
    // -------------------------------------------------
    Graph global_graph(graphs[0]);
    for (int i = 1; i < graphs.size(); ++i)
    {
        cout << "*******************************" << endl;
        cout << " N-View SfM: merging graph 0-" << i << endl;
        cout << "*******************************" << endl;
        // ------ merge graphs ------
        Graph::merge_graph(global_graph, graphs[i]);
        
        // ------ N-view triangulation ------
        triangulate_nonlinear(global_graph);
        float error = reprojection_error(global_graph);
        std::cout << "reprojection error (before bundle adjustment): " << error << std::endl;
        
        // ------ N-view bundle adjustment ------
        cout << "------ start bundle adjustment ------" << endl;
        Open3DCVBundleAdjustment(global_graph, BUNDLE_PRINCIPAL_POINT);
        cout << "------ end bundle adjustment ------" << endl;
        error = reprojection_error(global_graph);
        std::cout << "reprojection error (after bundle adjustment): " << error << std::endl;
    }
    
    // -------------------------------------------------
    // Output
    // -------------------------------------------------
    write_sfm(global_graph);
    return 0;
}
