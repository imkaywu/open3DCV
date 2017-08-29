#ifndef plot_h_
#define plot_h_

#include "image/image.h"
#include "matching/match.h"

namespace open3DCV
{
    
// r_p: (x, y)
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
    
inline void draw_plus(Image img, const vector<Keypoint>& keys, const int scale = 3)
{
    for (int i = 0; i < keys.size(); ++i)
    {
        draw_plus(img, keys[i].coords().cast<int>());
    }
    if (img.channel() == 1)
        img.write("sift.pgm");
    else
        img.write("sift.jpg");
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
                img.m_image[3 * ind + 0] = img.m_image[3 * ind + 1] = img.m_image[3 * ind + 2] = 255;
            
            ind = (r_p(1) + i) * img.width() + r_p(0) - i;
            if (img.in_image(ind))
                img.m_image[3 * ind + 0] = img.m_image[3 * ind + 1] = img.m_image[3 * ind + 2] = 255;
        }
    }
}

inline void draw_cross(Image img, const vector<Keypoint>& keys, const int scale = 3)
{
    for (int i = 0; i < keys.size(); ++i)
    {
        draw_cross(img, keys[i].coords().cast<int>());
    }
    if (img.channel() == 1)
        img.write("sift.pgm");
    else
        img.write("sift.jpg");
}

inline void draw_matches(const Image& img0, const vector<Keypoint> keys0, const Image& img1, const vector<Keypoint> keys1, vector<Match>& matches)
{
    Image img;
    img.combine_images(img0, img1);
    
    for (int i = 0; i < matches.size(); ++i)
    {
        Vec2i pos0, pos1;
        pos0 = keys0[matches[i].key_ind1_].coords().cast<int>();
        pos1 = keys1[matches[i].key_ind2_].coords().cast<int>();
        pos1(0) += img0.width();
        draw_cross(img, pos0);
        draw_cross(img, pos1);
        draw_line(img, pos0, pos1);
    }
    
    if (img.channel() == 1)
        img.write("sift.pgm");
    else
        img.write("sift.jpg");
}

}

#endif
