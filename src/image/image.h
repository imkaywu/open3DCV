#ifndef open3dcv_image_h
#define open3dcv_image_h

#include <iostream>
#include <vector>
#include <cmath>
#include "numeric.h"

using std::cerr;
using std::endl;
using std::string;
using std::vector;
using Eigen::Vector3f;
using Eigen::Vector3i;

namespace open3DCV
{
class Image
{
public:
    Image();
    Image(const string r_name);
    virtual ~Image();
    int init(const string r_name);
    int check_format(const string r_fmt);
    void free();
    
    int read(const string r_name);
    int write(const string r_name);
    static int read_any_image(const string r_name, vector<unsigned char>& r_image, int& r_width, int& r_height, int& r_channel);
    static int read_pbm(const string r_name, vector<unsigned char>& r_image, int& r_width, int& r_height);
    static int write_pbm(const string r_name, vector<unsigned char>& r_image, int& r_width, int& r_height);
    static int read_pgm(const string r_name, vector<unsigned char>& r_image, int& r_width, int& r_height);
    static int write_pgm(const string r_name, vector<unsigned char>& r_image, int& r_width, int& r_height);
    static int read_ppm(const string r_name, vector<unsigned char>& r_image, int& r_width, int& r_height, int& r_channel);
    static int write_ppm(const string r_name, vector<unsigned char>& r_image, int& r_width, int& r_height, int& r_channel);
    static int read_jpeg(const string r_name, vector<unsigned char>& r_image, int& r_width, int& r_height, int& r_channel);
    static void write_jpeg(const string r_name, vector<unsigned char>& r_image, int& r_width, int& r_height, int& r_channel, const int flip = 0);
    
    void rgb2grey();
    int width() const;
    int height() const;
    int channel() const;
    
    //void combine_images(Image& img1, Image& img2);
    void draw_line(Vec2i r_p1, Vec2i r_p2);
    
    Vector3f color(const float fx, const float fy) const;
    Vector3f color(const int ix, const int iy) const;
    
    // try to make this private
    vector<unsigned char> m_image;
    vector<unsigned char> m_gimage;
    
private:
    // 0: nothing allocated, 1: width/height allocated, 2: memory allocated;
    int alloc_;
    string name_;
    int width_;
    int height_;
    int channel_;
};
    
} // end of namespace open3DCV

#endif // end of image_hpp
    
