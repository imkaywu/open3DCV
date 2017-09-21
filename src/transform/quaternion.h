#ifndef quaternion_h_
#define quaternion_h_

#include "math/numeric.h"

namespace open3DCV
{
    void proj2quat(Mat4f& proj, float q[6]);
    void quat2proj(const float q[6], Mat4f& proj);
}

#endif
