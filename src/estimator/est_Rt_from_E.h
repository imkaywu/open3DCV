#ifndef estimate_rt_from_e_
#define estimate_rt_from_e_

#include "math/numeric.h"

namespace open3DCV
{
    void rt_from_e(const Mat3f& E, std::vector<Mat3f>& R, std::vector<Vec3f>& t);
    void rt_from_e(const Mat3f& E, Mat3f& R, Vec3f& t);
}

#endif
