#include "math/numeric.h"
#include "image/image.h"
#include "camera/camera.h"
#include "keypoint/sift.h"
#include "matching/matcher_flann.h"
#include "estimator/fundamental.h"
#include "estimator/ransac.h"
#include "viz/plot.h"

using namespace std;
using namespace open3DCV;

int main(const int argc, const char** argv)
{
    string idir = "/Users/BlacKay/Documents/Projects/Images/test/bust/";
    bool is_vis = true;
    
    // read images
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
    
    // feature detection, descriptor extraction
    Sift_Params sift_param(3, 3, 0, 10.0f / 255.0, 0, -INFINITY, 3, 2);
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
            draw_cross(images[i], keys[i], "feature"+to_string(i+1));
        }
    }
    
    // feature matching
    Matcher_Param matcher_param(0.3, 128, 10, 3);
    Matcher_Flann matcher(matcher_param);
    vector<vector<Match> > matches(nimages - 1, vector<Match>());
    for (int i = 0; i < nimages - 1; ++i)
    {
        matcher.match(descs[i], descs[i+1], matches[i]);
        if (is_vis)
        {
            draw_matches(images[i], keys[i], images[i+1], keys[i+1], matches[i], "matching"+to_string(i+1)+"_"+to_string(i+2));
        }
    }
    
    // estimate fundamental matrix
    vector<std::pair<Vec2f, Vec2f> > data;
    for (int i = 0; i < nimages - 1; ++i)
    {
        for (int j = 0; j < matches[i].size(); ++j)
        {
            Vec2f x1 = keys[i][matches[i][j].ikey1_].coords();
            Vec2f x2 = keys[i+1][matches[i][j].ikey2_].coords();
            std::pair<Vec2f, Vec2f> pair_data;
            pair_data.first = x1;
            pair_data.second = x2;
            data.push_back(pair_data);
        }
        vector<float> params(9);
        int* vote_inlier = new int[data.size()];
        Param_Estimator<std::pair<Vec2f, Vec2f>, float>* fund_esti = new open3DCV::Fundamental_Estimator(0.2f);
        float ratio_inlier = Ransac<std::pair<Vec2f, Vec2f>, float>::estimate(fund_esti, data, params, 0.99, vote_inlier);
        std::cout << "ratio of inliers: " << ratio_inlier << std::endl;
        
        vector<Match> matches_inlier;
        vector<std::pair<Vec2f, Vec2f> > data_inlier;
        for (int j = 0; j < matches[i].size(); ++j)
        {
            if (vote_inlier[j])
            {
                matches_inlier.push_back(matches[i][j]);
                data_inlier.push_back(data[j]);
            }

        }
        draw_matches(images[i], keys[i], images[i+1], keys[i+1], matches_inlier, "matching_inlier"+to_string(i+1)+"_"+to_string(i+2));
        
        Mat3f F;
        F << params[0], params[3], params[6],
             params[1], params[4], params[7],
             params[2], params[5], params[8];
        
        // visualize epipolar geometry
        if (is_vis)
        {
            draw_epipolar_geometry(images[i], images[i+1], F, data_inlier, "epipolar"+to_string(i+1)+"_"+to_string(i+2));
        }
        data.clear();
    }
    
    return 0;
}
