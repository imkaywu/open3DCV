#ifndef plot_h_
#define plot_h_

#include "CImg.h"
#include "image/image.h"
//#include "matching/match.h"

using namespace cimg_library;

namespace open3DCV
{
    
inline void convert_cimg(const Image& image, cimg_library::CImg<unsigned char> &img)
{
    for (int x = 0; x < image.width(); ++x)
        for (int y = 0; y < image.height(); ++y)
            for (int c = 0; c < image.channel(); ++c)
            {
                int ind = y * image.width() + x + c;
                img(x, y, 0, c) = image[ind];
            }
}
    
inline void draw_line(Image &image, const Vec2i x1, const Vec2i x2)
{
    CImg<unsigned char> img(image.width(), image.height(), 1, image.channel());
    convert_cimg(image, img);
    CImgDisplay disp_img(img, "draw line");
    const unsigned char color[] = { 255,128,64 };
    img.draw_line(x1(0), x1(1), x2(0), x2(1), color).display(disp_img);
}

inline void draw_line(Image &image, const float slope, const Vec2i x)
{
    Vec2i x1, x2;
    if (x(0) > 0 && x(0) < image.width() - 1 &&
        x(1) > 0 && x(1) < image.height() - 1)
    {
        x1 = x;
        x2(0) = image.width() - x(0) > x(0)? image.width() - 1 : 0;
        x2(1) = x(1) + round(slope * (x2(0) - x(0)));
    }
    else
    {
        x1(0) = 0;
        x1(1) = x(1) + round(slope * (x1(0) - x(0)));
        x2(0) = image.width() - 1;
        x2(1) = x(1) + round(slope * (x2(0) - x(0)));
    }
    draw_line(image, x1, x2);
}
    
inline void draw_point(Image &image, const Vec2i x)
{
    CImg<unsigned char> img(image.width(), image.height(), 1, image.channel());
    convert_cimg(image, img);
    CImgDisplay disp_img(img, "draw point");
    const unsigned char color[] = {255, 128, 64};
    img.draw_point(x(0), x(1), color).display(disp_img);
}
    
inline void disp_image(Image &image)
{
    CImg<unsigned char> img;
//    convert_cimg(image, img);
    cimg::imagemagick_path("/opt/local/bin/convert");
    img.load_jpeg(image.name().c_str());
    CImgDisplay disp_img(img, "disp image");
    img.display(disp_img);
}

}

#endif // end of namespace open3DCV
