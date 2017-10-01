#ifndef transformation_h_
#define transformation_h_

#include "math/numeric.h"

namespace open3DCV
{
    Mat34f concat_Rt(const Mat34f& outer_Rt, const Mat34f& inner_Rt);
    Mat34f inv_Rt(const Mat34f& r_Rt);
    Mat3f rotation_x(float angle);
    Mat3f rotation_y(float angle);
    Mat3f rotation_z(float angle);
    
    template<typename T>
    inline T degree2radian(T degree)
    {
        return degree * M_PI / 180.0;
    }
    
    template<typename T>
    inline T radian2degree(T radian)
    {
        return radian / M_PI * 180;
    }
}

#endif
