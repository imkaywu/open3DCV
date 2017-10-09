#include "transform/transform.h"

namespace open3DCV
{
    
    // Rt * X = outer_Rt * inner_Rt * X
    Mat34f concat_Rt(const Mat34f& outer_Rt, const Mat34f& inner_Rt)
    {
        Mat34f Rt;
        Rt.block<3, 3>(0, 0) = outer_Rt.block<3, 3>(0, 0) * inner_Rt.block<3, 3>(0, 0);
        Rt.block<3, 1>(0, 3) = outer_Rt.block<3, 3>(0, 0) * inner_Rt.block<3, 1>(0, 3) + outer_Rt.block<3, 1>(0, 3);
        
        return Rt;
    }
    
    Mat34f inv_Rt(const Mat34f& r_Rt)
    {
        Mat34f Rt_inv;
        Rt_inv.block<3, 3>(0, 0) = r_Rt.block<3, 3>(0, 0).transpose();
        Rt_inv.block<3, 1>(0, 3) = -r_Rt.block<3, 3>(0, 0).transpose() * r_Rt.block<3, 1>(0, 3);
        
        return Rt_inv;
    }
    
    Mat3f rotation_around_x(float angle)
    {
        return Eigen::AngleAxisf(angle, Vec3f::UnitX()).toRotationMatrix();
    }
    
    Mat3f rotation_around_y(float angle)
    {
        return Eigen::AngleAxisf(angle, Vec3f::UnitY()).toRotationMatrix();
    }
    
    Mat3f rotation_around_z(float angle)
    {
        return Eigen::AngleAxisf(angle, Vec3f::UnitZ()).toRotationMatrix();
    }
    
}
