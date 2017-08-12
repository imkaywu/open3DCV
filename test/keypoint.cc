//
//  keypoint.cpp
//  open3DCV_test
//
//  Created by KaiWu on Jul/4/17.
//  Copyright © 2017 KaiWu. All rights reserved.
//

#include <stdio.h>
#include "keypoint/sift.h"
#include "vis/plot.h"

using namespace std;
using namespace open3DCV;


//int main(int argc, const char * argv[]) {
//    Image image;
//    string img_pbm = "/Users/BlacKay/Documents/Projects/Images/test/marbles.pbm";
//    string img_pgm = "/Users/BlacKay/Documents/Projects/Images/test/mandrill.pgm";
//    string img_ppm = "/Users/BlacKay/Documents/Projects/Images/test/hara_fumina.ppm";
//    string img_jpg = "/Users/BlacKay/Documents/Projects/Images/test/1.jpg";
//    string img_jpeg = "/Users/BlacKay/Documents/Projects/Images/test/1.jpeg";
//    
//    vector<Keypoint> keypoints;
//    Sift_Params sift_params(3, 3, 0, 10, 0, -INFINITY, 3, 2);
//    Sift sift(sift_params);
//
//    // pgm test
////    image.read(img_pgm);
////    sift_detector.detect_keypoints_simp(image, keypoints);
////    draw_cross(image, keypoints);
////    keypoints.clear();
////    sift.clear();
//
//    // ppm test
//    image.read(img_ppm);
//    sift.detect_keypoints_simp(image, keypoints);
//    draw_cross(image, keypoints);
//    keypoints.clear();
//    sift.clear();
//
//    // jpg test
//    image.read(img_jpg);
//    sift.detect_keypoints_simp(image, keypoints);
//    draw_cross(image, keypoints);
//    keypoints.clear();
////    sift.clear();
//
//    return 0;
//}
//
