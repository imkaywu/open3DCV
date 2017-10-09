#include "estimator/utils.h"
#include "estimator/fundamental.h"
#include "RpolyPlusPlus/find_polynomial_roots_jenkins_traub.h"

using std::vector;
using std::pair;
using Eigen::JacobiSVD;

namespace open3DCV
{
Fundamental_Estimator::Fundamental_Estimator() : Param_Estimator<DMatch, float>(7, 9), error_thresh_(1e-6)
{
    // no-opt
}

Fundamental_Estimator::Fundamental_Estimator(const float thresh) : Param_Estimator<DMatch, float>(7, 9), error_thresh_(thresh)
{
    // no-opt
}

void Fundamental_Estimator::estimate(vector<DMatch>& data, vector<float>& params)
{
    vector<Vec2f> x1(data.size()), x2(data.size());
    for (int i = 0; i < data.size(); ++i)
    {
        x1[i] = data[i].point_.first;
        x2[i] = data[i].point_.second;
    }
    
    if (ndata() == 7)
    {
        vector<Mat3f> F;
        fund_seven_pts(x1, x2, F);
        params.resize(F.size() * nparam());
        for (int i = 0; i < F.size(); ++i)
            for (int j = 0; j < nparam(); ++j)
            {
                params[i * nparam() + j] = F[i](j);
            }
    }
    else if (ndata() == 8)
    {
        Mat3f F;
        fund_eight_pts(x1, x2, F);
        params.resize(nparam());
        for (int i = 0; i < nparam(); ++i)
        {
            params[i] = F(i);
        }
    }
}
    
void Fundamental_Estimator::ls_estimate(vector<DMatch>& data, vector<float>& params)
{
    vector<Vec2f> x1(data.size()), x2(data.size());
    for (int i = 0; i < data.size(); ++i)
    {
        x1[i] = data[i].point_.first;
        x2[i] = data[i].point_.second;
    }
    Mat3f F;
    fund_eight_pts(x1, x2, F);
    params.resize(9);
    for (int i = 0; i < F.rows() * F.cols(); ++i)
    {
        params[i] = F(i); // column first
    }
}

int Fundamental_Estimator::check_inliers(DMatch& data, vector<float>& params)
{
    Mat3f F;
    F << params[0], params[3], params[6],
         params[1], params[4], params[7],
         params[2], params[5], params[8];

    float resi = data.point_.second.homogeneous().dot(F * data.point_.first.homogeneous());
    
    if(resi < error_thresh_)
        return 1;
    
    return 0;
}

// didn't normalize points first
// the implementation is adapted frmo vgg_F_from_7pts_2img.m
void Fundamental_Estimator::fund_seven_pts (const std::vector<Vec2f>& x1, const std::vector<Vec2f>& x2, vector<Mat3f>& F)
{
    if (x1.size() != 7 || x2.size() != 7)
    {
        std::cerr << "Wrong size of input points." << std::endl;
        return;
    }
    // point normalization, not working?
    vector<Vec2f> nx1, nx2;
    Mat3f T1 = normalize_pts<Vec2f>(x1, nx1);
    Mat3f T2 = normalize_pts<Vec2f>(x2, nx2);
    
    Mat2Xf matx1, matx2;
    vector2mat<Vec2f, Mat2Xf>(nx1, matx1);
    vector2mat<Vec2f, Mat2Xf>(nx2, matx2);

    // construct the A matrix in Af = 0
    Matf A(9, 7);
    A << matx2.row(0).array() * matx1.row(0).array(),
         matx2.row(0).array() * matx1.row(1).array(),
         matx2.row(0).array(),
         matx2.row(1).array() * matx1.row(0).array(),
         matx2.row(1).array() * matx1.row(1).array(),
         matx2.row(1).array(),
         matx1.row(0).array(),
         matx1.row(1).array(),
         Matf::Ones(1, 7);
    A = A.transpose().eval();
//    std::cout << "A: " << std::endl << A << std::endl;
    
    // solving for nullspace of A to get two F
    JacobiSVD<Eigen::Matrix<float, Eigen::Dynamic, 9> > amatrix_svd(A, Eigen::ComputeFullV);
    Vec9f fvec1 = amatrix_svd.matrixV().col(7);
    Vec9f fvec2 = amatrix_svd.matrixV().col(8);
    
    vector<Mat3f> Fmat(2);
    Fmat[0] << fvec1(0), fvec1(3), fvec1(6),
               fvec1(1), fvec1(4), fvec1(7),
               fvec1(2), fvec1(5), fvec1(8);
    
    Fmat[1] << fvec2(0), fvec2(3), fvec2(6),
               fvec2(1), fvec2(4), fvec2(7),
               fvec2(2), fvec2(5), fvec2(8);
//    std::cout << "F1: " << std::endl << Fmat[0] << std::endl;
//    std::cout << "F2: " << std::endl << Fmat[1] << std::endl;
    
    // find F that meets the singularity constraint: det(a * F1 + (1 - a) * F2) = 0
    float D[2][2][2];
    for (int i1 = 0; i1 < 2; ++i1)
        for (int i2 = 0; i2 < 2; ++i2)
            for (int i3 = 0; i3 < 2; ++i3)
            {
                Mat3f Dtmp;
                Dtmp.col(0) = Fmat[i1].col(0);
                Dtmp.col(1) = Fmat[i2].col(1);
                Dtmp.col(2) = Fmat[i3].col(2);
                D[i1][i2][i3] = Dtmp.determinant();
            }
    
    // solving cubic equation and getting 1 or 3 solutions for F
    Vec coefficients(4);
    coefficients(0) = -D[1][0][0]+D[0][1][1]+D[0][0][0]+D[1][1][0]+D[1][0][1]-D[0][1][0]-D[0][0][1]-D[1][1][1];
    coefficients(1) = D[0][0][1]-2*D[0][1][1]-2*D[1][0][1]+D[1][0][0]-2*D[1][1][0]+D[0][1][0]+3*D[1][1][1];
    coefficients(2) = D[1][1][0]+D[0][1][1]+D[1][0][1]-3*D[1][1][1];
    coefficients(3) = D[1][1][1];
    
    Vec roots;
    rpoly_plus_plus::FindPolynomialRootsJenkinsTraub(coefficients, &roots, NULL);
//    std::cout << "roots: " << std::endl << roots << std::endl;
    
    // check sign consistency
    for (int i = 0; i < roots.size(); ++i)
    {
        Mat3f Ftmp = (float)roots(i) * Fmat[0] + (float)(1 - roots(i)) * Fmat[1];
        JacobiSVD<Mat3f> fmatrix_svd(Ftmp.transpose(), Eigen::ComputeFullV);
        Vec3f e1 = fmatrix_svd.matrixV().col(2);
        Mat3Xf l1_ex = cross_product_matrix<Mat3f, Vec3f>(e1) * matx1.colwise().homogeneous(); // lines connecting of x1 and e1
        Mat3Xf l1_Fx = Ftmp * matx2.colwise().homogeneous(); // lines determined by F and x2
        Vecf s = (l1_Fx.array() * l1_ex.array()).colwise().sum();
        if ((s.array() > 0).all() || (s.array() < 0).all())
        {
            F.push_back(T2.transpose() * Ftmp * T1);
        }
    }
}

void Fundamental_Estimator::fund_eight_pts(const vector<Vec2f>& x1, const vector<Vec2f>& x2, Mat3f& F)
{
    vector<Vec2f> nx1, nx2;
    Mat3f T1, T2;
    T1 = normalize_pts<Vec2f>(x1, nx1);
    T2 = normalize_pts<Vec2f>(x2, nx2);
//    std::cout << "T1: " << T1 << std::endl;
//    std::cout << "T2: " << T2 << std::endl;
    
    Mat2Xf matx1, matx2;
    vector2mat<Vec2f, Mat2Xf>(nx1, matx1);
    vector2mat<Vec2f, Mat2Xf>(nx2, matx2);
    
    Matf A(9, x1.size());
    A << matx2.row(0).array() * matx1.row(0).array(),
         matx2.row(0).array() * matx1.row(1).array(),
         matx2.row(0).array(),
         matx2.row(1).array() * matx1.row(0).array(),
         matx2.row(1).array() * matx1.row(1).array(),
         matx2.row(1).array(),
         matx1.row(0),
         matx1.row(1),
         Matf::Ones(1, x1.size());
    A = A.transpose().eval(); // better version of transpose().eval()
//    std::cout << "after transpose: " << std::endl << A << std::endl;
    
    // Solve the constraint equation for F from nullspace extraction.
    // An LU decomposition is efficient for the minimally constrained case.
    // Otherwise, use an SVD.
    Vec9f fvector;
    if (0) // x1.size() == 8
    {
        const auto lu_decomp = A.fullPivLu();
        if(lu_decomp.dimensionOfKernel() == 1)
        {
            fvector = lu_decomp.kernel();
        }
    }
    else
    {
        // Q: A = U∑V' = (-U)∑(-V').
        // A: F is estimated up to a scale, the sign of F is irrelevant
        JacobiSVD<Eigen::Matrix<float, Eigen::Dynamic, 9> > amatrix_svd(A, Eigen::ComputeFullV);
        fvector = amatrix_svd.matrixV().col(8);
    }
    
    F << fvector(0), fvector(1), fvector(2),
         fvector(3), fvector(4), fvector(5),
         fvector(6), fvector(7), fvector(8);
    
    // enforce the constraint that F is of rank 2
    // Find the closest singular matrix to F under frobenius norm.
    // We can compute this matrix with SVD.
    JacobiSVD<Mat3f> fmatrix_svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Vec3f singular_values = fmatrix_svd.singularValues();
    singular_values(2) = 0.0f;
    F = fmatrix_svd.matrixU() * singular_values.asDiagonal() * fmatrix_svd.matrixV().transpose();
    F = T2.transpose() * F * T1;
}

}
