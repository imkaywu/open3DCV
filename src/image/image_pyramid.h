#ifndef image_hpp
#define image_hpp

#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "CImg.h"

using namespace cimg_library;

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
    
    virtual void init(const string name, const string mname, const int maxLevel = 1);
    virtual void init(const string iname, const string mname, const int nillums, const int maxLevel);
    void completeName(const string& lhs, string& rhs, const int colour);
    
    // allocate and free memories
    void alloc(const int fast = 0, const int filter = 0);
    void free();
    void free(const int freeLevel);
    
    void buildPyramid(const int filter);
    void buildImagePyramid(const int filter);
    void buildImagePyramid2(const int filter);
    void buildMaskPyramid(const int filter);
    
    static int readImage(const string file, vector<unsigned char>& image, int& width, int& height, int& channel, const int fast);
    static int readJpeg(const string file, vector<unsigned char>& image, int& width, int& height, int& channel, const int fast);
    static void writeJpeg(const string file, vector<unsigned char>& image, const int width, const int height, const int flip = 0);
    static int readPBMImage(const string file, vector<unsigned char>& image, int& width, int& height, const int fast); // Portable BitMap
    static int writePGMImage(const std::string file, const std::vector<unsigned char>& image, const int width, const int height);
    static int readPGMImage(const string file, vector<unsigned char>& image, int& width, int& height, const int fast); // Portable GreyMap
    
    // single illumination
    Vector3f getColor(const float fx, const float fy, const int level) const;
    Vector3f getColor(const int ix, const int iy, const int level) const;
    // multiple illuminations
    Vector3f getColor(const float fx, const float fy, const int level, const int illum) const;
    Vector3f getColor(const int ix, const int iy, const int level, const int illum) const;
    
    int getMask(const float fx, const float fy, const int level) const;
    int getMask(const int ix, const int iy, const int level) const;
    
    int getWidth(const int level) const;
    int getHeight(const int level) const;
    
    // allocate/free images
    // 0: nothing allocated, 1: width/height allocated, 2: memory allocated;
    int m_alloc;
    
    // a set of pyramids of images from the single viewpoint, under different illumination
    //m_illums x m_level x (m_width * m_height)
    vector<vector<vector<unsigned char> > > m_imageSets;
    // a pyramic of images from the single viewpoint, under the same illumination
    // m_level x (m_width * m_height)
    vector<vector<unsigned char> > m_images;
    // a pyramic of masks
    vector<vector<unsigned char> > m_masks;
    // image widths at each level
    vector<int> m_widths;
    // image heights at each level
    vector<int> m_heights;
    // image channel number
    int m_nchanls;
    
    // image name
    string m_iname;
    // image names
    vector<string> m_inames;
    // mask name
    string m_mname;
    // number of illuminations
    int m_nillums;
    // number of levels
    int m_maxLevel;

#ifdef PMMVPS_IMAGE_GAMMA
    // gamma decoded images
    vector<vector<float> > m_dimages;
#endif
};
    
} // end of namespace open3DCV

#endif // end of image_hpp
    
