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
    Image(const Image& r_img);
    Image(const int h, const int w, const int c);
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
    
    void rgb2grey(const Image& img);
    void combine_images(const Image& img1, const Image& img2);
    inline int in_image(const int ind) { return ind < width_ * height_; }
    int width() const;
    int height() const;
    int channel() const;
    string name() const;
    
    Vector3f color(const float fx, const float fy) const;
    Vector3f color(const int ix, const int iy) const;
    
    unsigned char& operator[](int ind);
    unsigned char operator[](int ind) const;
    
    // try to make this private
    vector<unsigned char> m_image;
    
private:
    // 0: nothing allocated, 1: width/height allocated, 2: memory allocated;
    int alloc_;
    string name_;
    int width_;
    int height_;
    int channel_;
};
    
inline unsigned char& Image::operator[](int ind)
{
    if (ind > 0 && ind < width_ * height_ * channel_)
        return m_image[ind];
    
    return m_image[0];
}

inline unsigned char Image::operator[](int ind) const
{
    if (ind > 0 && ind < width_ * height_ * channel_)
        return m_image[ind];

    return m_image[0];
}
    
inline string Image::name() const
{
    return name_;
}
} // end of namespace open3DCV

#endif // end of image_hpp
    
