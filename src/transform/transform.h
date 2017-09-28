#ifndef transformation_h_
#define transformation_h_

#include "math/numeric.h"

namespace open3DCV
{
    Mat34f concat_Rt(const Mat34f& outer_Rt, const Mat34f& inner_Rt);
    Mat34f inv_Rt(const Mat34f& r_Rt);
    
}

#endif
