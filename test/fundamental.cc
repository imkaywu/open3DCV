#include <fstream>
#include <sstream>
#include "math/numeric.h"
#include "keypoint/keypoint.h"
#include "keypoint/sift.h"
#include "matching/matcher_brute_force.h"
#include "matching/matcher_flann.h"
#include "estimator/fundamental.h"
#include "estimator/param_estimator.h"
#include "estimator/ransac.h"
#include "vis/plot.h"

using std::string;
using std::ifstream;
using std::istringstream;
using namespace open3DCV;

void load_pts(const string fname, vector<Vec2f>& x1, vector<Vec2f>& x2)
{
    x1.clear();
    x2.clear();
    ifstream ifstr(fname.c_str());
    bool good = ifstr.is_open();
    string line;
    while (getline(ifstr, line))
    {
        istringstream iss(line);
        Vec2f y1, y2;
        iss >> y1(0) >> y1(1) >> y2(0) >> y2(1);
        x1.push_back(y1);
        x2.push_back(y2);
    }
    if (!good)
    {
        std::cerr << "Error loading points." << std::endl;
    }
}

int main(int argc, const char* argv[])
{
    // --------------------- test 1
//    string fname = "/Users/BlacKay/Documents/Projects/open3DCV/test/fundamental/matches.txt";
//    vector<Vec2f> x1, x2;
//    load_pts(fname, x1, x2);
//    
//    vector<std::pair<Vec2f, Vec2f> > data;
//    for (int i = 0; i < x1.size(); ++i)
//    {
//        std::pair<Vec2f, Vec2f> pair_data;
//        pair_data.first = x1[i];
//        pair_data.second = x2[i];
//        data.push_back(pair_data);
//    }
//    
//    vector<float> params(9);
//    open3DCV::Param_Estimator<std::pair<Vec2f, Vec2f>, float>* fund_esti = new open3DCV::Fundamental_Estimator(0.5f);
//    open3DCV::Ransac<std::pair<Vec2f, Vec2f>, float>::estimate(fund_esti, data, params, 0.95);
//    
//    Mat3f F;
//    F << params[0], params[1], params[2],
//         params[3], params[4], params[5],
//         params[6], params[7], params[8];
//    std::cout << F << std::endl;
    
    // --------------------- test 2 & 3
    string iname1 = "/Users/BlacKay/Documents/Projects/Images/test/Notre Dame/1.jpg";
    string iname2 = "/Users/BlacKay/Documents/Projects/Images/test/Notre Dame/2.jpg";
    Image image1(iname1), image2(iname2);
    image1.read(iname1);
    image2.read(iname2);
    
    vector<Keypoint> keys1, keys2;
    vector<Vecf> desc1, desc2;
    
    // --------------------- test 2
//    Sift_Params sift_params(3, 3, 0, 10.0f, 7.0f, -INFINITY, 3, 2);
//    Sift sift(sift_params);
//    
//    sift.detect_keypoints(image1, keys1, 0);
//    draw_cross(image1, keys1, "sift_1");
//    sift.extract_descriptors(image1, keys1, desc1);
//    Detector::write_keypoints(keys1, "key1.txt");
//    Descriptor::write_descriptors(desc1, "desc1.txt");
//    sift.clear();
//    
//    sift_params.peak_thresh_ = 7.0f;
//    sift.set_params(sift_params);
//    
//    sift.detect_keypoints(image2, keys2, 0);
//    draw_cross(image2, keys2, "sift_2");
//    sift.extract_descriptors(image2, keys2, desc2);
//    Detector::write_keypoints(keys1, "key2.txt");
//    Descriptor::write_descriptors(desc1, "desc2.txt");
//    sift.clear();
//    
//    vector<Match> matches;
//    Matcher_Param matcher_param(0.6, 128, 3, 10);
//    
//    Matcher_Brute_Force matcher(matcher_param);
//    //Matcher_Flann matcher(matcher_param);
//    matcher.match(desc1, desc2, matches);
//    draw_matches(image1, keys1, image2, keys2, matches, "matching");
//    Matcher::write_matches(matches, "matches.txt");
    
    // --------------------- test 3
    
    Detector::read_keypoints(keys1, "key1.txt");
    Detector::read_keypoints(keys2, "key2.txt");
//    Descriptor::read_descriptors(desc1, "desc1.txt");
//    Descriptor::read_descriptors(desc2, "desc2.txt");
    vector<Match> matches;
    Matcher::read_matches(matches, "matches.txt");
    
    
    
    return 0;
}
