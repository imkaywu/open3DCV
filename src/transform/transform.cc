#include "transform/transform.h"

namespace open3DCV
{
    
    // Rt * X = outer_Rt * inner_Rt * X
    Mat3Xf concat_Rt(const Mat3Xf& outer_Rt, const Mat3Xf& inner_Rt)
    {
        Mat3Xf Rt;
        Rt.block<3, 3>(0, 0) = outer_Rt.block<3, 3>(0, 0) * inner_Rt.block<3, 3>(0, 0);
        Rt.block<3, 1>(0, 3) = outer_Rt.block<3, 3>(0, 0) * inner_Rt.block<3, 1>(0, 3) + outer_Rt.block<3, 1>(0, 3);
        
        return Rt;
    }
    
    Mat3Xf inverse_Rt(const Mat3Xf& r_Rt)
    {
        Mat3Xf Rt_inv;
        Rt_inv.block<3, 3>(0, 0) = r_Rt.block<3, 3>(0, 0).transpose();
        Rt_inv.block<3, 1>(0, 3) = -r_Rt.block<3, 3>(0, 0).transpose() * r_Rt.block<3, 1>(0, 3);
        
        return Rt_inv;
    }
    
    
}
