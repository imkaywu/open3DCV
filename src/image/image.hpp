#ifndef open3dcv_image_hpp
#define open3dcv_image_hpp

#include <iostream>
#include <vector>
#include "Eigen/Dense"

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
    virtual ~Image();
    
    virtual void init(const string name);
    void free();
    void completeName(const string& lhs, string& rhs, const int isColor);
//    void alloc(const int fast = 0, const int filter = 0);
    
    static int readAnyImage(const string file, vector<unsigned char>& image, int& width, int& height, int& channel);
    static int readPBMImage(const string file, vector<unsigned char>& image, int& width, int& height);
    static int writePBMImage(const string file, vector<unsigned char>& image, const int width, const int height);
    static int readPGMImage(const string file, vector<unsigned char>& image, int& width, int& height);
    static int writePGMImage(const string file, const vector<unsigned char>& image, const int width, const int height);
    static int readPPMImage(const string file, vector<unsigned char>& image, int& width, int& height);
    static int writePPMImage(const string file, const vector<unsigned char>& image, const int width, const int height);
    static int readJpegImage(const string file, vector<unsigned char>& image, int& width, int& height, int& channel);
    static void writeJpegImage(const string file, vector<unsigned char>& image, const int width, const int height, const int flip = 0);
    
    int getWidth() const;
    int getHeight() const;
    
    Vector3f getColor(const float fx, const float fy) const;
    Vector3f getColor(const int ix, const int iy) const;
    
    // 0: nothing allocated, 1: width/height allocated, 2: memory allocated;
    int m_alloc;
    string m_iname;
    vector<unsigned char> m_image;
    int m_width;
    int m_height;
    int m_nchanl;
};
    
} // end of namespace open3DCV

#endif // end of image_hpp
    
