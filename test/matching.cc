//#include "matching/matcher.h"
//
//#include "math/numeric.h"
//#include "image/image.h"
//#include "keypoint/sift.h"
//#include "matching/matcher_brute_force.h"
//#include "matching/matcher_flann.h"
//#include "viz/plot.h"
//
//using namespace open3DCV;
//using std::string;
//
//int main(int argc, const char* argv[])
//{
//    string iname1 = "/Users/BlacKay/Documents/Projects/Images/test/Notre Dame/1.jpg";
//    string iname2 = "/Users/BlacKay/Documents/Projects/Images/test/Notre Dame/2.jpg";
//    
//    Sift_Params sift_params(3, 3, 0, 10.0f / 255.0, 0, -INFINITY, 3, 2);
//    Sift sift(sift_params);
//    vector<Keypoint> keys0, keys1;
//    vector<Vecf> desc0, desc1;
//    
//    Image img0, img1;
//    img0.read(iname1);
//    sift.detect_keypoints(img0, keys0);
//    sift.extract_descriptors(img0, keys0, desc0);
//    draw_cross(img0, keys0, "sift_1");
//    sift.clear();
//
//    img1.read(iname2);
//    sift.detect_keypoints(img1, keys1);
//    sift.extract_descriptors(img1, keys1, desc1);
//    draw_cross(img1, keys1, "sift_2");
//    sift.clear();
//    
//    Matcher_Param matcher_param(0.5);
//    Matcher* matcher = new Matcher_Flann(matcher_param);
//    vector<Match> matches;
//    matcher->match(desc0, desc1, matches);
//    draw_matches(img0, keys0, img1, keys1, matches, "matching");
//    
//    return 0;
//}
