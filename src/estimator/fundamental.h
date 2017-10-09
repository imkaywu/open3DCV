#ifndef fundamental_h_
#define fundamental_h_

#include "math/numeric.h"
#include "matching/dmatch.h"
#include "robust_estimator/param_estimator.h"

using Eigen::JacobiSVD;

namespace open3DCV
{
    
    class Fundamental_Estimator : public Param_Estimator<DMatch, float>
    {
    public:
        Fundamental_Estimator();
        Fundamental_Estimator(const float thresh);
        
        void estimate(std::vector<DMatch>& data, std::vector<float>& params);
        
        void ls_estimate(std::vector<DMatch>& data, std::vector<float>& params);
        
        int check_inliers(DMatch& data, std::vector<float>& params);

    private:
        
        void fund_seven_pts(const std::vector<Vec2f>& x1, const std::vector<Vec2f>& x2, std::vector<Mat3f>& F);
        
        void fund_eight_pts(const std::vector<Vec2f>& x1, const std::vector<Vec2f>& x2, Mat3f& F);
        
        float error_thresh_;
    };
    
}

#endif
