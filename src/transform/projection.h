#ifndef projection_h_
#define projection_h_

#include "math/numeric.h"

namespace open3DCV
{
    void P_from_KRt(const Mat3f& K, const Mat3f& R, const Vec3f& t, Mat34f& P);
    void KRt_from_P(const Mat34f& P, Mat3f& K, Mat3f& R, Vec3f& t);
    
    void project(const Mat34f& P, const Vec4f& X, Vec3f& x);
    void project(const Mat34f& P, const Vec4f& X, Vec2f& x);
    void project(const Mat34f& P, const Vec3f& X, Vec3f& x);
    void project(const Mat34f& P, const Vec3f& X, Vec2f& x);
    
    bool is_in_front_of_camera(const Mat34f& P, const Vec4f& X);
    bool is_in_front_of_camera(const Mat34f& P, const Vec3f& X);
}

#endif
