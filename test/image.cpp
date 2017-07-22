//
//  image.cpp
//  open3DCV_test
//
//  Created by KaiWu on Jul/4/17.
//  Copyright © 2017 KaiWu. All rights reserved.
//

#include <stdio.h>
#include "image.hpp"

using namespace std;
using namespace open3DCV;

int main(int argc, const char * argv[]) {
    Image image;
    string img_pbm = "/Users/BlacKay/Documents/Projects/Images/test/marbles.pbm";
    string img_pgm = "/Users/BlacKay/Documents/Projects/Images/test/mandrill.pgm";
    string img_ppm = "/Users/BlacKay/Documents/Projects/Images/test/hara_fumina.ppm";
    string img_jpeg = "/Users/BlacKay/Documents/Projects/Images/matching/1.jpg";
    vector<unsigned char> img;
    int width, height, channel;
    
    // pbm test
    image.readPBMImage(img_pbm, img, width, height);
    image.writePBMImage("testpbm.pbm", img, width, height);
    cout << "pbm: (" << width << ", " << height << ")" << endl;
    
    // pgm test
    image.readPGMImage(img_pgm, img, width, height);
    image.writePGMImage("testpgm.pgm", img, width, height);
    cout << "pgm: (" << width << ", " << height << ")" << endl;
    
    // ppm test
    image.readPPMImage(img_ppm, img, width, height);
    image.writePPMImage("testppm.jpg", img, width, height);
    cout << "ppm: (" << width << ", " << height << ")" << endl;
    
    // jpeg test
    image.readJpegImage(img_jpeg, img, width, height, channel);
    image.writeJpegImage("testjpeg.jpg", img, width, height);
    cout << "jpeg: (" << width << ", " << height << ", " << channel << ")" << endl;
    
    return 0;
}
