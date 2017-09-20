#ifndef transformation_h_
#define transformation_h_

#include "math/numeric.h"

namespace open3DCV
{
    Mat3Xf concat_Rt(const Mat3Xf& outer_Rt, const Mat3Xf& inner_Rt);
    Mat3Xf inverse_Rt(const Mat3Xf& r_Rt);
    
    // modified from 'rodrigues.h' and 'rodrigues.c' from VLFEAT
    void rodrigues(Mat3f& R, Matf* dR_pt, const Vec3f& om);
    void irodrigues (double* om_pt, double* dom_pt, const double* R_pt) ;
}

#endif
