//
//  keypoint.cpp
//  open3DCV_test
//
//  Created by KaiWu on Jul/4/17.
//  Copyright © 2017 KaiWu. All rights reserved.
//

#include <stdio.h>
#include "sift.h"

using namespace std;
using namespace open3DCV;


int main(int argc, const char * argv[]) {
    Image image;
    string img_pbm = "/Users/BlacKay/Documents/Projects/Images/test/marbles.pbm";
    string img_pgm = "/Users/BlacKay/Documents/Projects/Images/test/mandrill.pgm";
    string img_ppm = "/Users/BlacKay/Documents/Projects/Images/test/hara_fumina.ppm";
    string img_jpg = "/Users/BlacKay/Documents/Projects/Images/test/1.jpg";
    string img_jpeg = "/Users/BlacKay/Documents/Projects/Images/test/1.jpeg";
    
    vector<Keypoint> keypoints;
    Sift sift_detector;
    
    // pbm test
    image.read(img_pbm);
    sift_detector.detect_keypoints_simp(image, keypoints);
    for (int i = 0; i < static_cast<int>(keypoints.size()); ++i)
    {
        image.draw_cross(keypoints[i].coords());
    }
    image.write("testpbm.pbm");

    // pgm test
    image.read(img_pgm);
    sift_detector.detect_keypoints_simp(image, keypoints);
    for (int i = 0; i < static_cast<int>(keypoints.size()); ++i)
    {
        image.draw_cross(keypoints[i].coords().cast<int>());
    }
    image.write("testpgm.pgm");

    // ppm test
    image.read(img_ppm);
    sift_detector.detect_keypoints_simp(image, keypoints);
    for (int i = 0; i < static_cast<int>(keypoints.size()); ++i)
    {
        image.draw_cross(keypoints[i].coords().cast<int>());
    }
    image.write("testppm.ppm");

    // jpg test
    image.read(img_jpg);
    sift_detector.detect_keypoints_simp(image, keypoints);
    for (int i = 0; i < static_cast<int>(keypoints.size()); ++i)
    {
        Vec2i coords(keypoints[i].coords().cast<int>());
        image.draw_cross(coords);
    }
    image.write("testjpg.jpg");

    return 0;
}

