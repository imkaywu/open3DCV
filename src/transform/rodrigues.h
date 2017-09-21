#ifndef rodrigues_h_
#define rodrigues_h_

#include "math/numeric.h"

namespace open3DCV
{
    // modified from 'rodrigues.h' and 'rodrigues.c' from VLFEAT
    void rodrigues(Mat3f& R, Matf* dR_pt, const Vec3f& om);
    void irodrigues(Vec3f& om, Matf* dom_pt, const Mat3f& R);
}

#endif
