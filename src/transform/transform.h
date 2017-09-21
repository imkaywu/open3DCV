#ifndef transformation_h_
#define transformation_h_

#include "math/numeric.h"

namespace open3DCV
{
    Mat3Xf concat_Rt(const Mat3Xf& outer_Rt, const Mat3Xf& inner_Rt);
    Mat3Xf inverse_Rt(const Mat3Xf& r_Rt);

}

#endif
