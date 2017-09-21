#include "math/numeric.h"
#include "estimator/est_Rt_from_E.h"
#include "transform/projection.h"
#include "triangulation/triangulation.h"

using std::vector;
using Eigen::JacobiSVD;

namespace open3DCV
{
    void Rt_from_E(const Mat3f& E, vector<Mat3f>& R, vector<Vec3f>& t)
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
        
        // skew-symmetric matrix (translation vector)
        Mat3f S;
        S = U * Z * U.transpose();
        
        // two possible translation vectors
        t.resize(2);
        t[0] = U.block<3, 1>(0, 2);
        t[1] = -U.block<3, 1>(0, 2);
        
        // two possible rotation matrices
        R.resize(2);
        R[0] = U * W * V.transpose();
        R[1] = U * W.transpose() * V.transpose();
        
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
    
    void Rt_from_E(Graph& graph)
    {
        Mat3f E = graph.E_;
        vector<Mat3f> Rs(2);
        vector<Vec3f> ts(2);
        Rt_from_E(E, Rs, ts);
        
        vector<Mat34f> Rts(4);
        for (int i = 0; i < Rs.size(); ++i)
            for (int j = 0; j < ts.size(); ++j)
            {
                Rts[i * ts.size() + j].block<3, 3>(0, 0) = Rs[i];
                Rts[i * ts.size() + j].block<3, 1>(0, 3) = ts[j];
            }
        
        vector<Mat34f> poses(2);
        poses[0].block<3, 3>(0, 0) = Mat3f::Identity();
        poses[0].block<3, 1>(0, 3).setZero();
        
        const int nmatches = static_cast<int>(graph.matches_.size());
        vector<vector<Vec2f> > pts_matching(nmatches, vector<Vec2f>());
        vector<Vec3f> pts3d(nmatches);
        for (int i = 0; i < nmatches; ++i)
        {
            pts_matching[i][0] = graph.keys_[graph.matches_[i].ikey1_].coords();
            pts_matching[i][1] = graph.keys_[graph.matches_[i].ikey2_].coords();
        }
        
        vector<int> count(4);
        for (int i = 0; i < 4; ++i)
        {
            poses[1] = graph.K_[1] * Rts[i];
            triangulate_nonlinear(poses, pts_matching, pts3d);
            
            count[i] = 0;
            for (int j = 0; j < nmatches; ++j)
            {
                float d = Rts[i].block<1, 3>(2, 0) * (pts3d[i] - Rts[i].block<3, 1>(0, 3));
                if (pts3d[i](2) > 0 && d > 0)
                    ++count[i];
            }
        }
        
        vector<size_t> idx;
        idx = sort_indexes(count);
        graph.Rt_[1].block<3, 3>(0, 0) = Rts[idx[0]].block<3, 3>(0, 0);
        graph.Rt_[1].block<3, 1>(0, 3) = Rts[idx[0]].block<3, 1>(0, 3);
    }
    
}
