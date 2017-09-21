#include "transform/projection.h"

namespace open3DCV
{
    void P_from_KRt(const Mat3f& K, const Mat3f& R, const Vec3f& t, Mat34f& P)
    {
        P.block<3, 3>(0, 0) = K * R;
        P.block<3, 1>(0, 3) = K * t;
    }
    
    void KRt_from_P(const Mat34f& P, Mat3f& K, Mat3f& R, Vec3f& t)
    {
        
    }
    
    void project(const Mat34f& P, const Vec4f& X, Vec3f& x)
    {
        x = P * X;
    }
    
    void project(const Mat34f& P, const Vec4f& X, Vec2f& x)
    {
        Vec3f x_homog;
        x_homog = P * X;
        x = x_homog.block<2, 1>(0, 0) / x_homog(2);
    }
    
    void project(const Mat34f& P, const Vec3f& X, Vec3f& x)
    {
        x = P * X.homogeneous();
    }
    
    void project(const Mat34f& P, const Vec3f& X, Vec2f& x)
    {
        Vec3f x_homog;
        x_homog = P * X.homogeneous();
        x = x_homog.block<2, 1>(0, 0) / x_homog(2);
    }
    
    bool is_in_front_of_camera(const Mat34f& P, const Vec4f& X)
    {
        float cond_1 = P.row(2).dot(X) * X(3);
        float cond_2 = X(2) * X(3);
        if (cond_1 > 0 && cond_2 > 0)
            return true;
        else
            return false;
    }
    
    bool is_in_front_of_camera(const Mat34f& P, const Vec3f& X)
    {
        Vec4f X_homog;
        X_homog << X, 1;
        return is_in_front_of_camera(P, X_homog);
    }
}
