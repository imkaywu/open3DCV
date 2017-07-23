//
//  image.cpp
//  open3DCV_test
//
//  Created by KaiWu on Jul/4/17.
//  Copyright © 2017 KaiWu. All rights reserved.
//

#include <stdio.h>
#include "image.h"

using namespace std;
using namespace open3DCV;


int main(int argc, const char * argv[]) {
    Image image;
    string img_pbm = "/Users/BlacKay/Documents/Projects/Images/test/marbles.pbm";
    string img_pgm = "/Users/BlacKay/Documents/Projects/Images/test/mandrill.pgm";
    string img_ppm = "/Users/BlacKay/Documents/Projects/Images/test/hara_fumina.ppm";
    string img_jpg = "/Users/BlacKay/Documents/Projects/Images/test/1.jpg";
    string img_jpeg = "/Users/BlacKay/Documents/Projects/Images/test/1.jpeg";
    
    // pbm test
    image.read(img_pbm);
    image.write("testpbm.pbm");
    cout << "pbm: (" << image.width() << ", " << image.height() << ", " << image.channel() << ")" << endl;
    
    // pgm test
    image.read(img_pgm);
    image.write("testpgm.pgm");
    cout << "pgm: (" << image.width() << ", " << image.height() << ", " << image.channel() << ")" << endl;
    
    // ppm test
    image.read(img_ppm);
    image.write("testppm.ppm");
    cout << "ppm: (" << image.width() << ", " << image.height() << ", " << image.channel() << ")" << endl;
    
    // jpg test
    image.read(img_jpg);
    image.write("testjpg.jpg");
    cout << "jpg: (" << image.width() << ", " << image.height() << ", " << image.channel() << ")" << endl;
    
    // jpeg test
    image.read(img_jpeg);
    image.write("testjpeg.jpeg");
    cout << "jpeg: (" << image.width() << ", " << image.height() << ", " << image.channel() << ")" << endl;
    
    // rgb2grey test
    image.rgb2grey();
    image.write("testrgb2grey.pgm");
    
    return 0;
}

