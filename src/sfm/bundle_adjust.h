#ifndef bundle_adjust_h_
#define bundle_adjust_h_

#include "ceres/ceres.h"
#include "math/numeric.h"
#include "transform/rodrigues.h"

namespace open3DCV
{
    struct ReprojectionError
    {
        ReprojectionError(float observed_x, float observed_y)
            : observed_x_(observed_x), observed_y_(observed_y)
        {}
        
        template<typename T>
        bool operator()(const T* const camera,
                        const T* const point,
                        T* residules) const
        {
            // camera[0, 1, 2] are the angle-axis rotation
            T om[3];
            om[0] = camera[0]; om[1] = camera[1]; om[2] = camera[2];
            T R[9];
            rodrigues<T>(R, nullptr, om);
            T p[3];
            // might be a problem
            p[0] = R[0] * point[0] + R[1] * point[1] + R[2] * point[2] + camera[3];
            p[1] = R[3] * point[0] + R[4] * point[1] + R[5] * point[2] + camera[4];
            p[2] = R[6] * point[0] + R[7] * point[1] + R[8] * point[2] + camera[5];
            
            T xp = p[0] / p[2];
            T yp = p[1] / p[2];
            
            const T& focal = camera[6];
            const T w = T(480);
            const T h = T(640);
            const T cx = w / 2.0;
            const T cy = h / 2.0;
            
            T predicted_x = focal * xp + cx;
            T predicted_y = focal * yp + cy;
            
            residules[0] = predicted_x - T(observed_x_);
            residules[1] = predicted_y - T(observed_y_);
            return true;
        }
        
        // Factory to hide the construction of the CostFunction object from the client code
        static ceres::CostFunction* create(const float observed_x,
                                           const float observed_y)
        {
            return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 9, 3>(
                        new ReprojectionError(observed_x, observed_y)));
        }
        
        float observed_x_;
        float observed_y_;
    };
}

#endif
