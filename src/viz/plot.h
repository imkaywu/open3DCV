#ifndef plot_h_
#define plot_h_

#include "image/image.h"
#include "math/numeric.h"
#include "matching/dmatch.h"
#include "keypoint/keypoint.h"

namespace open3DCV
{

inline void draw_line(Image& img, Vec2i r_p1, Vec2i r_p2)
{
    int dx, dy, temp;
    int ch = img.channel();
    if (r_p1(0) == r_p2(0) && r_p1(1) == r_p2(1))
    { return; }
    
    // more horizontal than vertical
    if (abs(r_p1(1) - r_p2(1)) < abs(r_p1(0) - r_p2(0)))
    {
        // put points in increasing order by column (x)
        if (r_p1(0) > r_p2(0))
        {
            temp = r_p1(0); r_p1(0) = r_p2(0); r_p2(0) = temp;
            temp = r_p1(1); r_p1(1) = r_p2(1); r_p2(1) = temp;
        }
        dx = r_p2(0) - r_p1(0);
        dy = r_p2(1) - r_p1(1);
        for (int i = r_p1(0); i < r_p2(0); ++i)
        {
            int ind = img.width() * (r_p1(1) + (i - r_p1(0)) * dy / dx) + i;
            for (int c = 0; c < ch; ++c)
                { img.m_image[ch * ind + c] = 255; }
        }
    }
    else
    {
        if (r_p1(1) > r_p2(1))
        {
            temp = r_p1(0); r_p1(0) = r_p2(0); r_p2(0) = temp;
            temp = r_p1(1); r_p1(1) = r_p2(1); r_p2(1) = temp;
        }
        dx = r_p2(0) - r_p1(0);
        dy = r_p2(1) - r_p1(1);
        for (int i = r_p1(1); i < r_p2(1); ++i)
        {
            int ind = img.width() * i + (r_p1(0) + (i - r_p1(1)) * dx / dy);
            for (int c = 0; c < ch; ++c)
                { img.m_image[ch * ind + c] = 255; }
        }
    }
}

inline void draw_line(Image& img, Vec3f slope)
{
    Vec2i x1, x2, x3, x4;
    x1(0) = 0;
    x1(1) = -slope(2) / slope(1);
    x2(0) = img.width() - 1;
    x2(1) = -(slope(2) + x2(0) * slope(0)) / slope(1);
    x3(1) = 0;
    x3(0) = -slope(2) / slope(0);
    x4(1) = img.height() - 1;
    x4(0) = -(slope(2) + x4(1) * slope(1)) / slope(0);
    
    if (x1(1) > 0 && x1(1) < img.height() && x2(1) > 0 && x2(1) < img.height())
        draw_line(img, x1, x2);
    else if (x3(0) > 0 && x3(0) < img.width() && x4(0) > 0 && x4(0) < img.width())
        draw_line(img, x3, x4);
}

inline void draw_plus(Image& img, const Vec2i r_p, const int scale = 3)
{
    int ind;
    if (img.channel() == 1)
    {
        for (int i = 1-scale; i < scale; ++i)
        {
            ind = r_p(1) * img.width() + r_p(0) + i;
            img.m_image[ind] = 255;
            
            ind = (r_p(1) + i) * img.width() + r_p(0);
            img.m_image[ind] = 255;
        }
    }
    else
    {
        for (int i = 1-scale; i < scale; ++i)
        {
            ind = r_p(1) * img.width() + r_p(0) + i;
            img.m_image[3 * ind + 0] = img.m_image[3 * ind + 1] = img.m_image[3 * ind + 2] = 255;
            
            ind = (r_p(1) + i) * img.width() + r_p(0);
            img.m_image[3 * ind + 0] = img.m_image[3 * ind + 1] = img.m_image[3 * ind + 2] = 255;
        }
    }
}

inline void draw_plus(Image img, const vector<Keypoint>& keys, const string oname, const int scale = 3)
{
    for (int i = 0; i < keys.size(); ++i)
    {
        draw_plus(img, keys[i].coords().cast<int>());
    }
    if (img.channel() == 1)
        img.write(oname + ".pgm");
    else
        img.write(oname + ".jpg");
}

inline void draw_cross(Image& img, const Vec2i r_p, const int scale = 3)
{
    int ind;
    if (img.channel() == 1)
    {
        for (int i = 1-scale; i < scale; ++i)
        {
            ind = (r_p(1) + i) * img.width() + r_p(0) + i;
            if (img.in_image(ind))
                img.m_image[ind] = 255;
            
            ind = (r_p(1) + i) * img.width() + r_p(0) - i;
            if (img.in_image(ind))
                img.m_image[ind] = 255;
        }
    }
    else
    {
        for (int i = 1-scale; i < scale; ++i)
        {
            ind = (r_p(1) + i) * img.width() + r_p(0) + i;
            if (img.in_image(ind))
            {
                img.m_image[3 * ind + 0] = 255;
                img.m_image[3 * ind + 1] = img.m_image[3 * ind + 2] = 0;
            }
            
            ind = (r_p(1) + i) * img.width() + r_p(0) - i;
            if (img.in_image(ind))
            {
                img.m_image[3 * ind + 0] = img.m_image[3 * ind + 1] = 255;
                img.m_image[3 * ind + 1] = img.m_image[3 * ind + 2] = 0;
            }
        }
    }
}

inline void draw_cross(Image img, const vector<Keypoint>& keys, const string oname, const int scale = 3)
{
    for (int i = 0; i < keys.size(); ++i)
    {
        draw_cross(img, keys[i].coords().cast<int>());
    }
    if (img.channel() == 1)
        img.write(oname + ".pgm");
    else
        img.write(oname + ".jpg");
}

inline void draw_matches(const Image& img0, const vector<Keypoint> keys0, const Image& img1, const vector<Keypoint> keys1, vector<DMatch>& matches, const string oname)
{
    Image img;
    img.combine_images(img0, img1);
    
    for (int i = 0; i < matches.size(); ++i)
    {
        Vec2i pos0, pos1;
        pos0 = keys0[matches[i].ind_key_.first].coords().cast<int>();
        pos1 = keys1[matches[i].ind_key_.second].coords().cast<int>();
        pos1(0) += img0.width();
        draw_cross(img, pos0);
        draw_cross(img, pos1);
        draw_line(img, pos0, pos1);
    }
    
    if (img.channel() == 1)
        img.write(oname + ".pgm");
    else
        img.write(oname + ".jpg");
}
    
inline void draw_matches(const Image& img0, const Image& img1, vector<DMatch>& matches, const string oname)
{
    Image img;
    img.combine_images(img0, img1);
    
    for (int i = 0; i < matches.size(); ++i)
    {
        Vec2i pos0, pos1;
        pos0 = matches[i].point_.first.cast<int>();
        pos1 = matches[i].point_.second.cast<int>();
        pos1(0) += img0.width();
        draw_cross(img, pos0);
        draw_cross(img, pos1);
        draw_line(img, pos0, pos1);
    }
    
    if (img.channel() == 1)
        img.write(oname + ".pgm");
    else
        img.write(oname + ".jpg");
}

template<typename T>
inline Vec3f pt2slope(const T x1, const T x2)
{
    if (x1(0) == x2(0))
        return Vec3f::Zero(3);
    
    Vec3f slope;
    slope(0) = 1;
    slope(1) = -(x1(0) - x2(0))/(x1(1) - x2(1));
    slope(2) = -(x1(0) + slope(1) * x1(1));
    
    return slope;
}
    
inline void draw_epipolar_geometry(Image img1, Image img2, const Mat3f& F, const vector<DMatch>& matches, const string& oname)
{
    Mat3f Ftmp = F;
    Mat3f Ft = F.transpose().eval();
    Vec3f e1, e2, slope;
    nullspace<Mat3f, Vec3f>(&Ftmp, &e1);
    nullspace<Mat3f, Vec3f>(&Ft, &e2);
    e1 = e1.array() / e1(2);
    e2 = e2.array() / e2(2);
    
    for (int i = 0; i < std::min(30, (int)matches.size()); ++i)
    {
        draw_cross(img1, matches[i].point_.first.cast<int>());
//        slope = pt2slope<Vec2f>(matches[i].first, e1.block<2, 1>(0, 0));
        slope = Ft * matches[i].point_.second.homogeneous();
        draw_line(img1, slope);
        draw_cross(img2, matches[i].point_.second.cast<int>());
        slope = F * matches[i].point_.first.homogeneous();
        draw_line(img2, slope);
    }
    Image img;
    img.combine_images(img1, img2);
    if (img.channel() == 1)
        img.write(oname + ".pgm");
    else
        img.write(oname + ".jpg");
}

}

#endif
