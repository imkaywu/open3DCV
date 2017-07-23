#include "sift.h"

namespace open3DCV {
    
    int Sift::convert(Image& img, vl_sift_pix* fdata)
    {
        const int w = img.width();
        const int h = img.height();
        const int c = img.channel();
        
        // the inner storage ORDER of VlPgmImage is exactly the same as that of our Image class
        fdata = (vl_sift_pix*)malloc(w * h * c * sizeof(vl_sift_pix));
        
        if (img.channel() != 1)
        {
            for (int i = 0; i < img.width() * img.height(); ++i)
                { fdata[i] = img.m_gimage[i]; }
        }
        else
        {
            for (int i = 0; i < img.width() * img.height(); ++i)
                { fdata[i] = img.m_image[i]; }
        }
        
        return 0;
    }
    
    int Sift::detect()
    {
        
        return 0;
    }
    
    int Sift::descript()
    {
        
        return 0;
    }
}
