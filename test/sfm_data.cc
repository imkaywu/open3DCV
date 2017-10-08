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
#include "sfm/sfm_bundle_adjuster.h"
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
    bool update_focal = false;
    bool update_intrinsic = false;
    const float thresh_reproj1 = 1.0f;
    const float thresh_angle = 2.0f;
    
    char fname[100];
    const int ngraphs = 4;
    vector<Graph> graphs(ngraphs);
    for (int i = 0; i < ngraphs; ++i)
    {
        sprintf(fname, "sfm_data/%08d.txt", i);
        string temp = fname;
        read_sfm(temp, graphs[i]);
    }
    
    // -------------------------------------------------
    // N-view SfM
    // -------------------------------------------------
    // TODO: find a more elegant way to deal with the first graph
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
        
        // ------ report graphs ------
//        Graph::report_graph(global_graph);
        
        // ------ N-view triangulation ------
        triangulate_nonlinear(global_graph);
        float error = reprojection_error(global_graph);
        std::cout << "reprojection error (BEFORE bundle adjustment): " << error << std::endl;
        
        // ------ N-view bundle adjustment ------
        cout << "------ start bundle adjustment ------" << endl;
        Open3DCVBundleAdjustment(global_graph, BUNDLE_NO_INTRINSICS);
        if (update_focal)
            Open3DCVBundleAdjustment(global_graph, BUNDLE_FOCAL_LENGTH);
        else if (update_intrinsic)
            Open3DCVBundleAdjustment(global_graph, BUNDLE_INTRINSICS);
        cout << "------ end bundle adjustment ------" << endl;
        error = reprojection_error(global_graph);
        std::cout << "reprojection error (AFTER bundle adjustment): " << error << std::endl;
        
        // ------ outlier rejection ------
        global_graph.rm_outliers(thresh_reproj1, thresh_angle);
        
        // ------ N-view bundle adjustment ------
        cout << "------ start bundle adjustment ------" << endl;
        Open3DCVBundleAdjustment(global_graph, BUNDLE_NO_INTRINSICS);
        if (update_focal)
            Open3DCVBundleAdjustment(global_graph, BUNDLE_FOCAL_LENGTH);
        else if (update_intrinsic)
            Open3DCVBundleAdjustment(global_graph, BUNDLE_INTRINSICS);
        cout << "------ end bundle adjustment ------" << endl;
        error = reprojection_error(global_graph);
        std::cout << "reprojection error (AFTER bundle adjustment): " << error << std::endl;
        
        if (isnan(error))
        {
            error = reprojection_error(global_graph);
        }
    }
    for (int i = 0; i < global_graph.ncams_; ++i)
    {
        const Mat34f pose = global_graph.extrinsics_mat_[i];
        Vec3f center = -pose.block<3, 3>(0, 0).transpose() * pose.block<3, 1>(0, 3);
        cout << "center: " << center.transpose() << endl;
    }
    
    // -------------------------------------------------
    // Output
    // -------------------------------------------------
    for (int i = 0; i < global_graph.ncams_; ++i)
    {
        Mat3f& intrinsic_mat = global_graph.intrinsics_mat_[i];
        intrinsic_mat(0, 2) = 240;
        intrinsic_mat(1, 2) = 320;
    }
    write_sfm(global_graph);
    return 0;
}
