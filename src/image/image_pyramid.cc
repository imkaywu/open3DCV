//
//  image.cpp
//  PMMVPS
//
//  Created by KaiWu on Oct/20/16.
//  Copyright © 2016 KaiWu. All rights reserved.
//

#include "image.hpp"
#include <fstream>
#include <setjmp.h>
//#include "jpeglib.h" // used in writeJpeg

using std::ifstream;
using Eigen::Matrix4f;

Image::Image() {
    m_alloc = 0;
}

Image::~Image() {
}

void Image::init(const string name, const string mname, const int maxLevel) {
    m_alloc = 0;
    
    if (!name.empty()) {
        completeName(name, m_iname, 1);
    }
    if (!mname.empty()) {
        completeName(mname, m_mname, 0);
    }
    m_inames.push_back(m_iname);
    
    m_maxLevel = maxLevel;
    if (m_maxLevel == 0) {
        cerr << "Number of level 0, set it to 1." << endl;
        m_maxLevel = 1;
    }
}

void Image::init(const string iname, const string mname, const int nillums, const int maxLevel) {
    m_nillums = nillums;
    
    for (int n = 0; n < m_nillums; ++n) {
        char imageName[1024];
        sprintf(imageName, "%s%04d", iname.c_str(), n);
        init(imageName, mname, maxLevel);
    }
}

void Image::completeName(const string& lhs, string& rhs, const int isColor) {
    // what's the point of doing this
    if (5 <= lhs.length() && lhs[lhs.length() - 4] == '.') {
        rhs = lhs;
        return;
    }
    
    // ppm jpg
    if (isColor) {
        string stmp0 = lhs + ".ppm";    string stmp1 = lhs + ".jpg";
        string stmp2 = lhs + ".png";    string stmp3 = lhs + ".tiff";
        
        if (ifstream(stmp0.c_str()))
            rhs = stmp0;
        else if (ifstream(stmp1.c_str()))
            rhs = stmp1;
#if defined(PMMVPS_HAVE_PNG)
        else if (ifstream(stmp2.c_str()))
            rhs = stmp2;
#endif
#if defined(PMMVPS_HAVE_TIFF)
        else if (ifstream(stmp3.c_str()))
            rhs = stmp3;
#endif
        else
            rhs = lhs;
    }
    // pgm pbm
    else {
        string stmp0 = lhs + ".pgm";    string stmp1 = lhs + ".pbm";
        
        if (ifstream(stmp0.c_str()))
            rhs = stmp0;
        else if (ifstream(stmp1.c_str()))
            rhs = stmp1;
        else
            rhs = lhs;
    }
}

void Image::alloc(const int fast, const int filter) {
    if (m_alloc == 1 && fast == 1) // fast probably means fast allocation
        return;
    if (m_alloc == 2)
        return;
    
    if(m_iname.length() < 3) {
        cerr << "Image file name has less than 3 characters." << endl
             << "Cannot allocate: " << m_iname << endl;
        exit(1);
    }
    
    m_masks.resize(m_maxLevel);
    m_widths.resize(m_maxLevel);
    m_heights.resize(m_maxLevel);
    
    // allocate the first level for images
    if (m_nillums == 1) {
        m_images.resize(m_maxLevel);
        if (readJpeg(m_iname, m_images[0], m_widths[0], m_heights[0], m_nchanls, fast) == -1) {
            cerr << "Unsupported image format found. Stop allocation" << endl
            << m_iname << endl;
            return;
        }
    }
    else {
        m_imageSets.resize(m_nillums);
        for (int i = 0; i < m_nillums; ++i) {
            m_imageSets[i].resize(m_maxLevel);
            
            if (readJpeg(m_inames[i], m_imageSets[i][0], m_widths[0], m_heights[0], m_nchanls, fast) == -1) {
                cerr << "Unsupported image format found. Stop allocation" << endl
                << m_iname[i] << endl;
                return;
            }
        }
    }
    
#ifdef PMMVPS_IMAGE_GAMMA
    m_dimages.resize(m_maxLevel);
    // decodeGamma();
#endif
    
    for (int level = 1; level < m_maxLevel; ++level) {
        m_widths[level] = m_widths[level - 1] / 2;
        m_heights[level] = m_heights[level - 1] / 2;
    }
    m_alloc = 1;
    
    if (fast) {
        return;
    }
    // allocate the first level of the mask
    if (!m_mname.empty()) {
        if (readPGMImage(m_mname, m_masks[0], m_widths[0], m_heights[0], 0) == 0 ||
            readPBMImage(m_mname, m_masks[0], m_widths[0], m_heights[0], 0) == 0) {
            cerr << "Read mask: " << m_mname << endl;
            for (int i = 0; i < static_cast<int>(m_masks[0].size()); ++i) {
                if (127 < static_cast<int>(m_masks[0][i])) {
                    m_masks[0][i] = (unsigned char) 255;
                }
                else {
                    m_masks[0][i] = (unsigned char) 0;
                }
            }
        }
        else {
            m_mname = "";
        }
    }
    
    buildPyramid(filter);
    
    // test of CImage
    /*
    int view = 0;
    if (m_nillums == 1) {
        for (int l = 0; l < m_maxLevel; ++l) {
            char name[1024];
            sprintf(name, "%04d%04d.jpg", view, l);
            writeJpeg(name, m_images[l], m_widths[l], m_heights[l]);
        }
    }
    else {
        for (int i = 0; i < m_nillums; ++i) {
            char name[1024];
            for (int l = 0; l < m_maxLevel; ++l) {
                sprintf(name, "000%02d%02d%02d.jpg", view, i, l);
                writeJpeg(name, m_imageSets[i][l], m_widths[l], m_heights[l]);
            }
        }
    }
    for (int l = 0; l < m_maxLevel; ++l) {
        char name[1024];
        sprintf(name, "%04d%04d.pgm", view, l);
        writePGMImage(name, m_masks[l], m_widths[l], m_heights[l]);
    }
     */
    
    m_alloc = 2;
}

void Image::free(const int freeLevel) {
    for (int l = 0; l < freeLevel; ++l) {
#ifdef PMMVPS_IMAGE_GAMMA
        vector<float>().swap(m_dimages[l]);
#else
        if (m_nillums == 1) {
            vector<unsigned char>().swap(m_images[l]);
        }
        else {
            for (int i = 0; i < m_nillums; ++i) {
                vector<unsigned char>().swap(m_imageSets[i][l]);
            }
        }
#endif
        if (!m_masks.empty()) {
            vector<unsigned char>().swap(m_masks[l]);
        }
    }
}

void Image::free() {
    if(m_alloc != 0)
        m_alloc = 1;
    else
        m_alloc = 0;
    if (m_nillums == 1) {
        vector<vector<unsigned char> >().swap(m_images);
    }
    else {
        vector<vector<vector<unsigned char> > >().swap(m_imageSets);
    }
    vector<vector<unsigned char> >().swap(m_masks);
}

// currently not used
void Image::buildPyramid(const int filter) {
    if (m_nillums == 1) {
        buildImagePyramid(filter);
    }
    else {
        buildImagePyramid2(filter);
    }
    
    if (!m_mname.empty()) {
        buildMaskPyramid(filter);
    }
}

//-------------------------------------------------------
// single illumination
//-------------------------------------------------------
void Image::buildImagePyramid(const int filter) {
    Matrix4f mask; // ??? why isn't this mask odd
    mask << 1.0f, 3.0f, 3.0f, 1.0f,
    3.0f, 9.0f, 9.0f, 3.0f,
    3.0f, 9.0f, 9.0f, 3.0f,
    1.0f, 3.0f, 3.0f, 1.0f;
    mask /= mask.sum();
    
    for (int level = 1; level < m_maxLevel; ++level) {
        const int sz = m_widths[level] * m_heights[level] * 3;
#ifdef PMMVPS_IMAGE_GAMMA
        m_dimages[level].resize(sz);
#else
        m_images[level].resize(sz);
#endif
        for (int y = 0; y < m_heights[level]; ++y) {
            for (int x = 0; x < m_widths[level]; ++x) {
                Vector3f color = Vector3f::Zero();
                if (filter == 2) {
                    color << 255.0f, 255.0f, 255.0f;
                }
                
                // why [-1, 3): has to do with the 4x4 mask
                for (int i = -1; i < 3; ++i) {
                    const int ytmp = 2 * y + i;
                    if (ytmp < 0 || m_heights[level - 1] - 1 < ytmp) {
                        continue;
                    }
                    
                    for (int j = -1; j < 3; ++j) {
                        const int xtmp = 2 * x + j;
                        if (xtmp < 0 || m_widths[level - 1] - 1 < xtmp) {
                            continue;
                        }
                        
                        const int index = (ytmp * m_widths[level - 1] + xtmp) * 3;
#ifdef PMMVPS_IMAGE_GAMMA
                        // to be written
#else
                        if (filter == 0) {
                            color(0) += mask(i + 1, j + 1) * (float)m_images[level - 1][index];
                            color(1) += mask(i + 1, j + 1) * (float)m_images[level - 1][index + 1];
                            color(2) += mask(i + 1, j + 1) * (float)m_images[level - 1][index + 2];
                        }
                        else if (filter == 1) {
                            color(0) = std::max(color(0), (float)m_images[level - 1][index]);
                            color(1) = std::max(color(1), (float)m_images[level - 1][index + 1]);
                            color(2) = std::max(color(2), (float)m_images[level - 1][index + 2]);
                        }
                        else if (filter == 2) {
                            color(0) = std::min(color(0), (float)m_images[level - 1][index]);
                            color(1) = std::min(color(1), (float)m_images[level - 1][index + 1]);
                            color(2) = std::min(color(2), (float)m_images[level - 1][index + 2]);
                        }
#endif
                    }
                }
                if (filter == 0)
                    color /= mask.sum();
                const int index = (y * m_widths[level] + x) * 3;
#ifdef PMMVPS_IMAGE_GAMMA
                // to be written
#else
                m_images[level][index] = (unsigned char)((int)floorf(color(0) + 0.5f));
                m_images[level][index + 1] = (unsigned char)((int)floorf(color(1) + 0.5f));
                m_images[level][index + 2] = (unsigned char)((int)floorf(color(2) + 0.5f));
#endif
            }
        }
    }
}

Vector3f Image::getColor(const int ix, const int iy, const int level) const {
#ifdef PMMVPS_DEBUG
    if (m_alloc != 2) {
        cerr << "First allocate" << std::endl;
        exit (1);
    }
#endif
    const int index = (iy * m_widths[level] + ix) * 3;
    
#ifdef PMMVPS_IMAGE_GAMMA
    return Vector3f(m_dimages[level][index],
                    m_dimages[level][index+1],
                    m_dimages[level][index+2]);
#else
    return Vector3f(m_images[level][index],
                    m_images[level][index+1],
                    m_images[level][index+2]);
#endif
}

Vector3f Image::getColor(const float x, const float y, const int level) const {
#ifdef PMMVPS_DEBUG
    if (m_alloc != 2) {
        std::cerr << "First allocate" << std::endl;
        exit (1);
    }
#endif
    
#ifdef PMMVPS_IMAGE_BICUBIC  // Bicubic case
    const int x1 = (int)floor(x);      const int y1 = (int)floor(y);
    const float p = x - x1;      const float q = y - y1;
    
    float f = 1+p;
    const float wx0 = (((-1) * f + 5) * f - 8) * f + 4;
    f = 2-p;
    const float wx3 = (((-1) * f + 5) * f - 8) * f + 4;
    f = p;
    const float wx1 = (((1) * f - 2) * f) * f + 1;
    f = 1 - p;
    const float wx2 = (((1) * f - 2) * f) * f + 1;
    
    f = 1+q;
    const float wy0 = (((-1) * f + 5) * f - 8) * f + 4;
    f = 2-q;
    const float wy3 = (((-1) * f + 5) * f - 8) * f + 4;
    f = q;
    const float wy1 = (((1) * f - 2) * f) * f + 1;
    f = 1 - q;
    const float wy2 = (((1) * f - 2) * f) * f + 1;
    
    const int offset = m_widths[level] * 3;
    const int index0 = ((y1 - 1) * m_widths[level] + x1 - 1) * 3;
    const int index1 = index0 + offset;
    const int index2 = index1 + offset;
    const int index3 = index2 + offset;
    
#ifdef PMMVPS_IMAGE_GAMMA
    // to be written
#else
    const unsigned char& r00 = m_images[level][index0];
    const unsigned char& g00 = m_images[level][index0+1];
    const unsigned char& b00 = m_images[level][index0+2];
    const unsigned char& r01 = m_images[level][index0+3];
    const unsigned char& g01 = m_images[level][index0+4];
    const unsigned char& b01 = m_images[level][index0+5];
    const unsigned char& r02 = m_images[level][index0+6];
    const unsigned char& g02 = m_images[level][index0+7];
    const unsigned char& b02 = m_images[level][index0+8];
    const unsigned char& r03 = m_images[level][index0+9];
    const unsigned char& g03 = m_images[level][index0+10];
    const unsigned char& b03 = m_images[level][index0+11];
    
    const unsigned char& r10 = m_images[level][index1];
    const unsigned char& g10 = m_images[level][index1+1];
    const unsigned char& b10 = m_images[level][index1+2];
    const unsigned char& r11 = m_images[level][index1+3];
    const unsigned char& g11 = m_images[level][index1+4];
    const unsigned char& b11 = m_images[level][index1+5];
    const unsigned char& r12 = m_images[level][index1+6];
    const unsigned char& g12 = m_images[level][index1+7];
    const unsigned char& b12 = m_images[level][index1+8];
    const unsigned char& r13 = m_images[level][index1+9];
    const unsigned char& g13 = m_images[level][index1+10];
    const unsigned char& b13 = m_images[level][index1+11];
    
    const unsigned char& r20 = m_images[level][index2];
    const unsigned char& g20 = m_images[level][index2+1];
    const unsigned char& b20 = m_images[level][index2+2];
    const unsigned char& r21 = m_images[level][index2+3];
    const unsigned char& g21 = m_images[level][index2+4];
    const unsigned char& b21 = m_images[level][index2+5];
    const unsigned char& r22 = m_images[level][index2+6];
    const unsigned char& g22 = m_images[level][index2+7];
    const unsigned char& b22 = m_images[level][index2+8];
    const unsigned char& r23 = m_images[level][index2+9];
    const unsigned char& g23 = m_images[level][index2+10];
    const unsigned char& b23 = m_images[level][index2+11];
    
    const unsigned char& r30 = m_images[level][index3];
    const unsigned char& g30 = m_images[level][index3+1];
    const unsigned char& b30 = m_images[level][index3+2];
    const unsigned char& r31 = m_images[level][index3+3];
    const unsigned char& g31 = m_images[level][index3+4];
    const unsigned char& b31 = m_images[level][index3+5];
    const unsigned char& r32 = m_images[level][index3+6];
    const unsigned char& g32 = m_images[level][index3+7];
    const unsigned char& b32 = m_images[level][index3+8];
    const unsigned char& r33 = m_images[level][index3+9];
    const unsigned char& g33 = m_images[level][index3+10];
    const unsigned char& b33 = m_images[level][index3+11];
#endif // PMMVPS_IMAGE_GAMMA
    // separate x and y
    const float row0[3] = {wx0 * r00 + wx1 * r01 + wx2 * r02 + wx3 * r03,
        wx0 * g00 + wx1 * g01 + wx2 * g02 + wx3 * g03,
        wx0 * b00 + wx1 * b01 + wx2 * b02 + wx3 * b03};
    const float row1[3] = {wx0 * r10 + wx1 * r11 + wx2 * r12 + wx3 * r13,
        wx0 * g10 + wx1 * g11 + wx2 * g12 + wx3 * g13,
        wx0 * b10 + wx1 * b11 + wx2 * b12 + wx3 * b13};
    const float row2[3] = {wx0 * r20 + wx1 * r21 + wx2 * r22 + wx3 * r23,
        wx0 * g20 + wx1 * g21 + wx2 * g22 + wx3 * g23,
        wx0 * b20 + wx1 * b21 + wx2 * b22 + wx3 * b23};
    const float row3[3] = {wx0 * r30 + wx1 * r31 + wx2 * r32 + wx3 * r33,
        wx0 * g30 + wx1 * g31 + wx2 * g32 + wx3 * g33,
        wx0 * b30 + wx1 * b31 + wx2 * b32 + wx3 * b33};
    
    float r = wy0 * row0[0] + wy1 * row1[0] + wy2 * row2[0] + wy3 * row3[0];
    float g = wy0 * row0[1] + wy1 * row1[1] + wy2 * row2[1] + wy3 * row3[1];
    float b = wy0 * row0[2] + wy1 * row1[2] + wy2 * row2[2] + wy3 * row3[2];
    
    return Vector3f(r, g, b);
#else // Bilinear case
    const int lx = static_cast<int>(x);
    const int ly = static_cast<int>(y);
    const int index = 3 * (ly * m_widths[level] + lx);
    
    const float dx1 = x - lx;  const float dx0 = 1.0f - dx1;
    const float dy1 = y - ly;  const float dy0 = 1.0f - dy1;
    
    const float f00 = dx0 * dy0;  const float f01 = dx0 * dy1;
    const float f10 = dx1 * dy0;  const float f11 = dx1 * dy1;
    const int index2 = index + 3 * m_widths[level];
    
#ifdef PMMVPS_IMAGE_GAMMA
    // to be written
#else
    const unsigned char* ucp0 = &m_images[level][index] - 1;
    const unsigned char* ucp1 = &m_images[level][index2] - 1;
    float r = 0.0f;  float g = 0.0f;  float b = 0.0f;
    r += *(++ucp0) * f00 + *(++ucp1) * f01;
    g += *(++ucp0) * f00 + *(++ucp1) * f01;
    b += *(++ucp0) * f00 + *(++ucp1) * f01;
    r += *(++ucp0) * f10 + *(++ucp1) * f11;
    g += *(++ucp0) * f10 + *(++ucp1) * f11;
    b += *(++ucp0) * f10 + *(++ucp1) * f11;
    return Vector3f(r, g, b);
#endif // PMMVPS_IMAGE_GAMMA
    
#endif // PMMVPS_IMAGE_BICUBIC
}

//-------------------------------------------------------
// multiple illuminations
//-------------------------------------------------------
void Image::buildImagePyramid2(const int filter) {
    Matrix4f mask; // ??? why this mask isn't odd
    mask << 1.0f, 3.0f, 3.0f, 1.0f,
    3.0f, 9.0f, 9.0f, 3.0f,
    3.0f, 9.0f, 9.0f, 3.0f,
    1.0f, 3.0f, 3.0f, 1.0f;
    mask /= mask.sum();
    
    for (int illum = 0; illum < m_nillums; ++illum) {
        for (int level = 1; level < m_maxLevel; ++level) {
            const int sz = m_widths[level] * m_heights[level] * 3;
#ifdef PMMVPS_IMAGE_GAMMA
            m_dimages[level].resize(sz);
#else
            m_imageSets[illum][level].resize(sz);
#endif
            for (int y = 0; y < m_heights[level]; ++y) {
                for (int x = 0; x < m_widths[level]; ++x) {
                    Vector3f color = Vector3f::Zero();
                    if (filter == 2) {
                        color << 255.0f, 255.0f, 255.0f;
                    }
                    
                    // why [-1, 3): has to do with the 4x4 mask
                    for (int i = -1; i < 3; ++i) {
                        const int ytmp = 2 * y + i;
                        if (ytmp < 0 || m_heights[level - 1] - 1 < ytmp) {
                            continue;
                        }
                        
                        for (int j = -1; j < 3; ++j) {
                            const int xtmp = 2 * x + j;
                            if (xtmp < 0 || m_widths[level - 1] - 1 < xtmp) {
                                continue;
                            }
                            
                            const int index = (ytmp * m_widths[level - 1] + xtmp) * 3;
#ifdef PMMVPS_IMAGE_GAMMA
                            // to be written
#else
                            if (filter == 0) {
                                color(0) += mask(i + 1, j + 1) * (float)m_imageSets[illum][level - 1][index];
                                color(1) += mask(i + 1, j + 1) * (float)m_imageSets[illum][level - 1][index + 1];
                                color(2) += mask(i + 1, j + 1) * (float)m_imageSets[illum][level - 1][index + 2];
                            }
                            else if (filter == 1) {
                                color(0) = std::max(color(0), (float)m_imageSets[illum][level - 1][index]);
                                color(1) = std::max(color(1), (float)m_imageSets[illum][level - 1][index + 1]);
                                color(2) = std::max(color(2), (float)m_imageSets[illum][level - 1][index + 2]);
                            }
                            else if (filter == 2) {
                                color(0) = std::min(color(0), (float)m_imageSets[illum][level - 1][index]);
                                color(1) = std::min(color(1), (float)m_imageSets[illum][level - 1][index + 1]);
                                color(2) = std::min(color(2), (float)m_imageSets[illum][level - 1][index + 2]);
                            }
#endif
                        }
                    }
                    if (filter == 0)
                        color /= mask.sum();
                    const int index = (y * m_widths[level] + x) * 3;
#ifdef PMMVPS_IMAGE_GAMMA
                    // to be written
#else
                    m_imageSets[illum][level][index] = (unsigned char)((int)floorf(color(0) + 0.5f));
                    m_imageSets[illum][level][index + 1] = (unsigned char)((int)floorf(color(1) + 0.5f));
                    m_imageSets[illum][level][index + 2] = (unsigned char)((int)floorf(color(2) + 0.5f));
#endif
                }
            }
        }
    }
}

Vector3f Image::getColor(const int ix, const int iy, const int level, const int illum) const {
#ifdef PMMVPS_DEBUG
    if (m_alloc != 2) {
        cerr << "First allocate" << std::endl;
        exit (1);
    }
#endif
    const int index = (iy * m_widths[level] + ix) * 3;
    
#ifdef PMMVPS_IMAGE_GAMMA
    return Vector3f(m_dimages[level][index],
                    m_dimages[level][index+1],
                    m_dimages[level][index+2]);
#else
    return Vector3f(m_imageSets[level][illum][index],
                    m_imageSets[level][illum][index+1],
                    m_imageSets[level][illum][index+2]);
#endif
}

Vector3f Image::getColor(const float x, const float y, const int level, const int illum) const {
#ifdef PMMVPS_DEBUG
    if (m_alloc != 2) {
        std::cerr << "First allocate" << std::endl;
        exit (1);
    }
#endif
    
#ifdef PMMVPS_IMAGE_BICUBIC  // Bicubic case
    const int x1 = (int)floor(x);      const int y1 = (int)floor(y);
    const float p = x - x1;      const float q = y - y1;
    
    float f = 1+p;
    const float wx0 = (((-1) * f + 5) * f - 8) * f + 4;
    f = 2-p;
    const float wx3 = (((-1) * f + 5) * f - 8) * f + 4;
    f = p;
    const float wx1 = (((1) * f - 2) * f) * f + 1;
    f = 1 - p;
    const float wx2 = (((1) * f - 2) * f) * f + 1;
    
    f = 1+q;
    const float wy0 = (((-1) * f + 5) * f - 8) * f + 4;
    f = 2-q;
    const float wy3 = (((-1) * f + 5) * f - 8) * f + 4;
    f = q;
    const float wy1 = (((1) * f - 2) * f) * f + 1;
    f = 1 - q;
    const float wy2 = (((1) * f - 2) * f) * f + 1;
    
    const int offset = m_widths[level] * 3;
    const int index0 = ((y1 - 1) * m_widths[level] + x1 - 1) * 3;
    const int index1 = index0 + offset;
    const int index2 = index1 + offset;
    const int index3 = index2 + offset;
    
#ifdef PMMVPS_IMAGE_GAMMA
    // to be written
#else
    const unsigned char& r00 = m_imageSets[illum][level][index0];
    const unsigned char& g00 = m_imageSets[illum][level][index0+1];
    const unsigned char& b00 = m_imageSets[illum][level][index0+2];
    const unsigned char& r01 = m_imageSets[illum][level][index0+3];
    const unsigned char& g01 = m_imageSets[illum][level][index0+4];
    const unsigned char& b01 = m_imageSets[illum][level][index0+5];
    const unsigned char& r02 = m_imageSets[illum][level][index0+6];
    const unsigned char& g02 = m_imageSets[illum][level][index0+7];
    const unsigned char& b02 = m_imageSets[illum][level][index0+8];
    const unsigned char& r03 = m_imageSets[illum][level][index0+9];
    const unsigned char& g03 = m_imageSets[illum][level][index0+10];
    const unsigned char& b03 = m_imageSets[illum][level][index0+11];
    
    const unsigned char& r10 = m_imageSets[illum][level][index1];
    const unsigned char& g10 = m_imageSets[illum][level][index1+1];
    const unsigned char& b10 = m_imageSets[illum][level][index1+2];
    const unsigned char& r11 = m_imageSets[illum][level][index1+3];
    const unsigned char& g11 = m_imageSets[illum][level][index1+4];
    const unsigned char& b11 = m_imageSets[illum][level][index1+5];
    const unsigned char& r12 = m_imageSets[illum][level][index1+6];
    const unsigned char& g12 = m_imageSets[illum][level][index1+7];
    const unsigned char& b12 = m_imageSets[illum][level][index1+8];
    const unsigned char& r13 = m_imageSets[illum][level][index1+9];
    const unsigned char& g13 = m_imageSets[illum][level][index1+10];
    const unsigned char& b13 = m_imageSets[illum][level][index1+11];
    
    const unsigned char& r20 = m_imageSets[illum][level][index2];
    const unsigned char& g20 = m_imageSets[illum][level][index2+1];
    const unsigned char& b20 = m_imageSets[illum][level][index2+2];
    const unsigned char& r21 = m_imageSets[illum][level][index2+3];
    const unsigned char& g21 = m_imageSets[illum][level][index2+4];
    const unsigned char& b21 = m_imageSets[illum][level][index2+5];
    const unsigned char& r22 = m_imageSets[illum][level][index2+6];
    const unsigned char& g22 = m_imageSets[illum][level][index2+7];
    const unsigned char& b22 = m_imageSets[illum][level][index2+8];
    const unsigned char& r23 = m_imageSets[illum][level][index2+9];
    const unsigned char& g23 = m_imageSets[illum][level][index2+10];
    const unsigned char& b23 = m_imageSets[illum][level][index2+11];
    
    const unsigned char& r30 = m_imageSets[illum][level][index3];
    const unsigned char& g30 = m_imageSets[illum][level][index3+1];
    const unsigned char& b30 = m_imageSets[illum][level][index3+2];
    const unsigned char& r31 = m_imageSets[illum][level][index3+3];
    const unsigned char& g31 = m_imageSets[illum][level][index3+4];
    const unsigned char& b31 = m_imageSets[illum][level][index3+5];
    const unsigned char& r32 = m_imageSets[illum][level][index3+6];
    const unsigned char& g32 = m_imageSets[illum][level][index3+7];
    const unsigned char& b32 = m_imageSets[illum][level][index3+8];
    const unsigned char& r33 = m_imageSets[illum][level][index3+9];
    const unsigned char& g33 = m_imageSets[illum][level][index3+10];
    const unsigned char& b33 = m_imageSets[illum][level][index3+11];
#endif // PMMVPS_IMAGE_GAMMA
    // separate x and y
    const float row0[3] = {wx0 * r00 + wx1 * r01 + wx2 * r02 + wx3 * r03,
        wx0 * g00 + wx1 * g01 + wx2 * g02 + wx3 * g03,
        wx0 * b00 + wx1 * b01 + wx2 * b02 + wx3 * b03};
    const float row1[3] = {wx0 * r10 + wx1 * r11 + wx2 * r12 + wx3 * r13,
        wx0 * g10 + wx1 * g11 + wx2 * g12 + wx3 * g13,
        wx0 * b10 + wx1 * b11 + wx2 * b12 + wx3 * b13};
    const float row2[3] = {wx0 * r20 + wx1 * r21 + wx2 * r22 + wx3 * r23,
        wx0 * g20 + wx1 * g21 + wx2 * g22 + wx3 * g23,
        wx0 * b20 + wx1 * b21 + wx2 * b22 + wx3 * b23};
    const float row3[3] = {wx0 * r30 + wx1 * r31 + wx2 * r32 + wx3 * r33,
        wx0 * g30 + wx1 * g31 + wx2 * g32 + wx3 * g33,
        wx0 * b30 + wx1 * b31 + wx2 * b32 + wx3 * b33};
    
    float r = wy0 * row0[0] + wy1 * row1[0] + wy2 * row2[0] + wy3 * row3[0];
    float g = wy0 * row0[1] + wy1 * row1[1] + wy2 * row2[1] + wy3 * row3[1];
    float b = wy0 * row0[2] + wy1 * row1[2] + wy2 * row2[2] + wy3 * row3[2];
    
    return Vector3f(r, g, b);
#else // Bilinear case
    const int lx = static_cast<int>(x);
    const int ly = static_cast<int>(y);
    const int index = 3 * (ly * m_widths[level] + lx);
    
    const float dx1 = x - lx;  const float dx0 = 1.0f - dx1;
    const float dy1 = y - ly;  const float dy0 = 1.0f - dy1;
    
    const float f00 = dx0 * dy0;  const float f01 = dx0 * dy1;
    const float f10 = dx1 * dy0;  const float f11 = dx1 * dy1;
    const int index2 = index + 3 * m_widths[level];
    
#ifdef PMMVPS_IMAGE_GAMMA
    // to be written
#else
    const unsigned char* ucp0 = &m_imageSets[illum][level][index] - 1;
    const unsigned char* ucp1 = &m_imageSets[illum][level][index2] - 1;
    float r = 0.0f;  float g = 0.0f;  float b = 0.0f;
    r += *(++ucp0) * f00 + *(++ucp1) * f01;
    g += *(++ucp0) * f00 + *(++ucp1) * f01;
    b += *(++ucp0) * f00 + *(++ucp1) * f01;
    r += *(++ucp0) * f10 + *(++ucp1) * f11;
    g += *(++ucp0) * f10 + *(++ucp1) * f11;
    b += *(++ucp0) * f10 + *(++ucp1) * f11;
    return Vector3f(r, g, b);
#endif // PMMVPS_IMAGE_GAMMA
    
#endif // PMMVPS_IMAGE_BICUBIC
}

//-------------------------------------------------------
// universal illumination
//-------------------------------------------------------
void Image::buildMaskPyramid(const int filter) {
    for (int level = 1; level < m_maxLevel; ++level) {
        const int sz = m_widths[level] * m_heights[level];
        
        m_masks[level].resize(sz);
        for (int y = 0; y < m_heights[level]; ++y) {
            const int ys[2] = {2 * y, std::min(m_heights[level - 1], 2 * y + 1)};
            for (int x = 0; x < m_widths[level];  ++x) {
                const int xs[2] = {2 * x, std::min(m_widths[level - 1], 2 * x + 1)};
                int inside = 0, outside = 0;
                for (int j = 0; j < 2; ++j) {
                    for (int i = 0; i < 2; ++i) {
                        const int index = ys[j] * m_widths[level - 1] + xs[i];
                        if (m_masks[level - 1][index]) {
                            ++inside;
                        }
                        else {
                            ++outside;
                        }
                    }
                }
                const int index = y * m_widths[level] + x;
                if (0 < inside) {
                    m_masks[level][index] = (unsigned char)255;
                }
                else
                    m_masks[level][index] = (unsigned char)0;
            }
        }
    }
}

int Image::getMask(const float fx, const float fy, const int level) const {
    if (m_alloc != 2) {
        cerr << "Image data not allocated" << endl;
        exit(1);
    }
    
    if (m_masks[level].empty()) {
        return -1;
    }
    
    const int ix = (int)floorf(fx + 0.5f);
    const int iy = (int)floorf(fy + 0.5f);
    return getMask(ix, iy, level);
}

int Image::getMask(const int ix, const int iy, const int level) const {
    if (m_alloc != 2) {
        cerr << "Image data not allocated" << endl;
        exit(1);
    }
    
    if (m_masks[level].empty()) {
        return -1;
    }
    
    if (ix < 0 || m_widths[level] <= ix ||
        iy < 0 || m_heights[level] <= iy) {
        return -1;
    }
    
    const int index = iy * m_widths[level] + ix;
    return m_masks[level][index];
}

int Image::getWidth(const int level) const {
    return m_widths[level];
};

int Image::getHeight(const int level) const {
    return m_heights[level];
};

//-----------------------------------------------
// Read image
//-----------------------------------------------
int Image::readImage(const string file, vector<unsigned char>& image, int& width, int& height, int& channel, const int fast) {
    CImg<unsigned char> cimage;
    try {
        cimage.load(file.c_str());
        if (!cimage.is_empty()) {
            if (cimage.spectrum() != 1 && cimage.spectrum() != 3) {
                cerr << "Cannot handle this component. Component num is " << cimage.spectrum() << endl;
                return -1;
            }
            width = cimage.width();
            height = cimage.height();
            channel = cimage.spectrum();
            
            // CImg holds the image internally in a different order, so we need to reorder it here
            image.resize(cimage.size());
            int i = 0;
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    for (int c = 0; c < 3; ++c) {
                        image[i] = cimage(x, y, 0, c);
                        i++;
                    }
                }
            }
        }
    }
    catch (cimg_library::CImgException& e) {
        cerr << "Cannot read image " << file << endl;
        return -1;
    }
    return 0;
}

int Image::readJpeg(const std::string file, vector<unsigned char>& image, int& width, int& height, int& channel, const int fast) {
    if (file.substr(file.length() - 3, file.length()) != "jpg") {
        cerr << file.substr(file.length() - 3, file.length()) << endl;
        return -1;
    }
    
    // Use CImg for image loading
    cimg::imagemagick_path("/opt/local/bin/convert");
    CImg<unsigned char> cimage;
    try {
        cimage.load_jpeg(file.c_str());
        if(!cimage.is_empty()) {
            if(cimage.spectrum() != 1 && cimage.spectrum() != 3) {
                cerr << "Cannot handle this component. Component num is " << cimage.spectrum() << endl;
                return 1;
            }
            width = cimage.width();
            height = cimage.height();
            channel = cimage.spectrum();
            
            // CImg holds the image internally in a different order, so we need to reorder it here
            int i = 0;
            if (channel == 1) {
                image.resize(3 * cimage.size());
                
                for(int y = 0; y < cimage.height(); y++) {
                    for(int x = 0; x < cimage.width(); x++) {
                        image[i] = image[i + 1] = image[i + 2] = cimage(x, y, 0, 0);
                        i += 3;
                    }
                }
            }
            else if (channel == 3) {
                image.resize(cimage.size());
                
                for(int y = 0; y < cimage.height(); y++) {
                    for(int x = 0; x < cimage.width(); x++) {
                        for(int c = 0; c < cimage.spectrum(); c++) {
                            image[i] = cimage(x, y, 0, c);
                            i++;
                        }
                    }
                }
            }
        }
    }
    catch(cimg_library::CImgException& e) {
        std::cerr << "Couldn't read image " << file.c_str() << std::endl;
        return -1;
    }
    
    return 0;
}

int Image::readPBMImage(const string file, vector<unsigned char>& image, int& width, int& height, const int fast) {
    
    if (file.substr(file.length() - 3, file.length()) != "pbm") {
        return -1;
    }
    
    ifstream ifstr;
    ifstr.open(file.c_str());
    if (!ifstr.is_open()) {
        return -1;
    }
    string header;
    unsigned char byte;
    
    ifstr >> header;
    ifstr.read((char*)&byte, sizeof(unsigned char));
    
    if (header != "P4") {
        cerr << "Only accept binary pbm format: " << file << endl;
        return -1;
    }
    
    while (1) {
        ifstr.read((char*)&byte, sizeof(unsigned char));
        ifstr.putback(byte);
        if (byte == '#') {
            char buffer[1024];
            ifstr.getline(buffer, 1024);
        }
        else {
            break;
        }
    }
    ifstr >> width >> height;
    ifstr.read((char*)&byte, sizeof(unsigned char));
    
    image.clear();
    if (fast) {
        ifstr.close();
        return 0;
    }
    int reso = width * height;
    if (reso % 8 != 0) {
        reso++;
    }
    
    int count = 0;
    for (int i = 0; i < reso; ++i) {
        ifstr.read((char*)&byte, sizeof(unsigned char));
        for (int j = 0; j < 8; ++j) {
            if (byte >> 7) {
                image.push_back((unsigned char)0);
            }
            else {
                image.push_back((unsigned char)255);
            }
            count++;
            byte <<= 1;
            if (count == width * height) {
                break;
            }
        }
    }
    ifstr.close();
    return 0;
}

int Image::readPGMImage(const string file, vector<unsigned char>& image, int& width, int& height, const int fast) {
    if (file.substr(file.length() - 3, file.length()) != "pgm") {
        return -1;
    }
    
    ifstream ifstr;
    ifstr.open(file.c_str());
    if (!ifstr.is_open()) {
        cerr << "Cannot open a file: " << file << endl;
        return -1;
    }
    
    string header;
    unsigned char byte;
    
    ifstr >> header;
    ifstr.read((char*)&byte, sizeof(unsigned char));
    if (header != "P5") {
        cerr << "Only accept binary pgm format" << file << " " << header << endl;
        return -1;
    }
    
    while (1) {
        ifstr.read((char*)&byte, sizeof(unsigned char));
        ifstr.putback(byte);
        if (byte == '#') {
            char buffer[1024];
            ifstr.getline(buffer, 1024);
        }
        else {
            break;
        }
    }
    int itmp;
    ifstr >> width >> height >> itmp;
    ifstr.read((char*)&byte, sizeof(unsigned char));
    
    image.clear();
    if (fast) {
        ifstr.close();
        return 0;
    }
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            ifstr.read((char*)&byte, sizeof(unsigned char));
            image.push_back(byte);
        }
    }
    ifstr.close();
    
    return 0;
}

int Image::writePGMImage(const std::string file, const std::vector<unsigned char>& image, const int width, const int height) {
    std::ofstream ofstr;
    ofstr.open(file.c_str());
    if (!ofstr.is_open()) {
        cerr << "Cannot write to a file: " << file << endl;
        return -1;
    }
    
    ofstr << "P5" << endl
    << width << ' ' << height << endl
    << 255 << endl;
    
    for (int i = 0; i < width * height; ++i) {
        unsigned char uctmp = image[i];
        ofstr.write((char*)&uctmp, sizeof(unsigned char));
    }
    
    ofstr.close();
    return 1;
    
    return 0;
}

//struct my_error_mgr {
//    struct jpeg_error_mgr pub;	/* "public" fields */
//    
//    jmp_buf setjmp_buffer;	/* for return to caller */
//};
//typedef struct my_error_mgr * my_error_ptr;
//
//METHODDEF(void)
//my_error_exit (j_common_ptr cinfo)
//{
//    /* cinfo->err really points to a my_error_mgr struct, so coerce pointer */
//    my_error_ptr myerr = (my_error_ptr) cinfo->err;
//    
//    /* Always display the message. */
//    /* We could postpone this until after returning, if we chose. */
//    (*cinfo->err->output_message) (cinfo);
//    
//    /* Return control to the setjmp point */
//    longjmp(myerr->setjmp_buffer, 1);
//}
//
//void Image::writeJpeg(const string filename, vector<unsigned char>& buffer, const int width, const int height, const int flip) {
//    const int quality = 100;
//    
//    struct jpeg_compress_struct cinfo;
//    struct jpeg_error_mgr jerr;
//    /* More stuff */
//    FILE * outfile;		/* target file */
//    JSAMPROW row_pointer[1];	/* pointer to JSAMPLE row[s] */
//    int row_stride;		/* physical row width in image buffer */
//    
//    cinfo.err = jpeg_std_error(&jerr);
//    jpeg_create_compress(&cinfo);
//    
//    if ((outfile = fopen(filename.c_str(), "wb")) == NULL) {
//        fprintf(stderr, "can't open %s\n", filename.c_str());
//        exit(1);
//    }
//    jpeg_stdio_dest(&cinfo, outfile);
//    
//    cinfo.image_width = width;
//    cinfo.image_height = height;
//    cinfo.input_components = 3;
//    cinfo.in_color_space = JCS_RGB;
//    jpeg_set_defaults(&cinfo);
//    jpeg_set_quality(&cinfo, quality, TRUE /* limit to baseline-JPEG values */);
//    
//    jpeg_start_compress(&cinfo, TRUE);
//    
//    row_stride = width * 3;	/* JSAMPLEs per row in image_buffer */
//    
//    while (cinfo.next_scanline < cinfo.image_height) {
//        if (flip)
//            row_pointer[0] = (JSAMPROW)& buffer[(cinfo.image_height - 1 - cinfo.next_scanline) * row_stride];
//        else
//            row_pointer[0] = (JSAMPROW)& buffer[cinfo.next_scanline * row_stride];
//        (void) jpeg_write_scanlines(&cinfo, row_pointer, 1);
//    }
//    jpeg_finish_compress(&cinfo);
//    fclose(outfile);
//    
//    jpeg_destroy_compress(&cinfo);
//}