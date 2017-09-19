#include "Eigen/SVD"
#include "est_Rt_from_E.h"
#include "triangulation/triangulation.h"

using std::vector;
using Eigen::JacobiSVD;

namespace open3DCV
{
    void rt_from_e(const Mat3f& E, vector<Mat3f>& R, vector<Vec3f>& t)
    {
        JacobiSVD<Eigen::Matrix<float, 3, 3> > ematrix_svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Mat3f U, V;
        U = ematrix_svd.matrixU();
        V = ematrix_svd.matrixV();
        
        Mat3f W, Z;
        W << 0, -1, 0,
             1,  0, 0,
             0,  0, 1;
        Z << 0,  1, 0,
            -1,  0, 0,
             0,  0, 0;
        
        // translation vector (skew-symmetric)
        Mat3f S;
        S = U * Z * U.transpose();
        
        // two possible rotation matrices
        R.resize(2);
        R[0] = U * W * V.transpose();
        R[1] = U * W.transpose() * V.transpose();
        
        // two possible translation vectors
        t.resize(2);
        t[0] = U.block<3, 1>(0, 2);
        t[1] = -U.block<3, 1>(0, 2);
        
        // check determinant
        if(R[0].determinant() < 0)
        {
            R[0] = -R[0].eval();
        }
        if(R[1].determinant() < 0)
        {
            R[1] = -R[1].eval();
        }
    }
    
    void rt_from_e(const Mat3f& E, Mat3f& R, Vec3f& t)
    {
        vector<Mat3f> Rs(2);
        vector<Vec3f> ts(2);
        rt_from_e(E, Rs, ts);
        
        // four possible solutions
        vector<Mat34f> Rts(4);
        for (int i = 0; i < Rs.size(); ++i)
            for (int j = 0; j < ts.size(); ++j)
            {
                Rts[i * ts.size() + j].block<3, 3>(0, 0) = Rs[i];
                Rts[i * ts.size() + j].block<3, 1>(0, 3) = ts[j];
            }
        
        // ???
        Mat3f K;
        vector<Vec2f> pts;
        vector<Vec3f> pts_triangulated;
        
        // triangulation
        vector<Mat34f> poses(2);
        poses[0].block<3, 3>(0, 0) = K;
        poses[0].block<3, 1>(0, 3) = Vec3f::Zero();
        
        vector<int> good_count(4);
        for (int i = 0; i < 4; ++i)
        {
            poses[1] = K * Rts[i];
            triangulate_nonlinear(poses, pts, pts_triangulated);
            
            good_count[i] = 0;
            for (int j = 0; j < pts.size(); ++j)
            {
                float d = Rts[i].block<1, 3>(2, 0) * (pts_triangulated[i] - ts[i]);
                if (pts_triangulated[i](2) > 0 && d > 0)
                {
                    ++good_count[i];
                }
            }
        }
        
        vector<size_t> idx;
        idx = sort_indexes(good_count);
        R = Rts[idx[0]].block<3, 3>(0, 0);
        t = Rts[idx[0]].block<3, 1>(0, 3);
    }
}
