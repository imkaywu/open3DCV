// Copyright (c) 2007, 2008 libmv authors.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
//
// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef open3DCV_numeric_h
#define open3DCV_numeric_h

#include "Eigen/Core"
#include "Eigen/Eigenvalues"
#include "Eigen/Geometry"
#include "Eigen/LU"
#include "Eigen/QR"
#include "Eigen/SparseCore"
#include "Eigen/SVD"
#include "Eigen/StdVector"

#include <cmath>
#include <numeric>
#include <string>
#include <iostream>
#include <vector>

namespace open3DCV
{

// Check MSVC
#if _WIN32 || _WIN64
  #if _WIN64
    #define ENV64BIT
  #else
    #define ENV32BIT
  #endif
#endif

// Check GCC
#if __GNUC__
  #if __x86_64__ || __ppc64__ || _LP64
    #define ENV64BIT
  #else
    #define ENV32BIT
  #endif
#endif

typedef Eigen::Vector2i Vec2i;
typedef Eigen::Vector2f Vec2f;
typedef Eigen::Vector3i Vec3i;
typedef Eigen::Vector3f Vec3f;
typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector4i Vec4i;
typedef Eigen::Matrix<float, 6, 1> Vec6f;
typedef Eigen::Matrix<float, 9, 1> Vec9f;
typedef Eigen::Matrix<double, 9, 1> Vec9;

#if defined(ENV32BIT)
    typedef Eigen::Matrix<double, 2, 1, Eigen::DontAlign> Vec2;
    typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> Vec4f;
    typedef Eigen::Matrix<double, 4, 1, Eigen::DontAlign> Vec4;
    typedef Eigen::Matrix<double, 6, 1, Eigen::DontAlign> Vec6;
    typedef Eigen::Matrix<float, 8, 1, Eigen::DontAlign> Vec8f;
    typedef Eigen::Matrix<double, 8, 1, Eigen::DontAlign> Vec8;
#else
    typedef Eigen::Vector2d Vec2;
    typedef Eigen::Vector4f Vec4f;
    typedef Eigen::Vector4d Vec4;
    typedef Eigen::Matrix<double, 6, 1> Vec6;
    typedef Eigen::Matrix<float, 8, 1> Vec8f;
    typedef Eigen::Matrix<double, 8, 1> Vec8;
#endif

typedef Eigen::Matrix2i Mat2i;
typedef Eigen::Matrix3i Mat3i;
typedef Eigen::Matrix3f Mat3f;
typedef Eigen::Matrix3d Mat3;

#if defined(ENV32BIT)
    typedef Eigen::Matrix<float, 2, 2, Eigen::DontAlign> Mat2f;
    typedef Eigen::Matrix<double, 2, 2, Eigen::DontAlign> Mat2;
    typedef Eigen::Matrix<int, 4, 4, Eigen::DontAlign> Mat4i;
    typedef Eigen::Matrix<float, 4, 4, Eigen::DontAlign> Mat4f;
    typedef Eigen::Matrix<double, 4, 4, Eigen::DontAlign> Mat4;
    typedef Eigen::Matrix<float, 3, 4, Eigen::DontAlign> Mat34f;
    typedef Eigen::Matrix<double, 3, 4, Eigen::DontAlign> Mat34;
#else
    typedef Eigen::Matrix2f Mat2f;
    typedef Eigen::Matrix2d Mat2;
    typedef Eigen::Matrix4i Mat4i;
    typedef Eigen::Matrix4f Mat4f;
    typedef Eigen::Matrix4d Mat4;
    typedef Eigen::Matrix<float, 3, 4> Mat34f;
    typedef Eigen::Matrix<double, 3, 4> Mat34;
#endif

//-- General purpose Matrix and Vector
typedef Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> Vecu;
typedef Eigen::VectorXi Veci;
typedef Eigen::VectorXf Vecf;
typedef Eigen::VectorXd Vec;
typedef Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic> Matu;
typedef Eigen::MatrixXi Mati;
typedef Eigen::MatrixXf Matf;
typedef Eigen::MatrixXd Mat;
typedef Eigen::Matrix<float, 2, Eigen::Dynamic> Mat2Xf;
typedef Eigen::Matrix<float, Eigen::Dynamic, 2> MatX2f;
typedef Eigen::Matrix<float, 3, Eigen::Dynamic> Mat3Xf;
typedef Eigen::Matrix<float, 4, Eigen::Dynamic> Mat4Xf;
typedef Eigen::Matrix<float, 9, Eigen::Dynamic> Mat9Xf;
typedef Eigen::Matrix<double, 2, Eigen::Dynamic> Mat2X;
typedef Eigen::Matrix<double, 3, Eigen::Dynamic> Mat3X;
typedef Eigen::Matrix<double, 4, Eigen::Dynamic> Mat4X;
typedef Eigen::Matrix<double, 9, Eigen::Dynamic> Mat9X;

/// Quaternion type
typedef Eigen::Quaternion<double> Quaternion;
typedef Eigen::Quaternion<float> Quaternionf;

using Eigen::Map;

/// Trait used for double type
typedef Eigen::NumTraits<double> EigenDoubleTraits;

/// 3x3 matrix using double internal format with RowMajor storage
typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> RMat3;

//-- Sparse Matrix (Column major, and row major)
/// Sparse unconstrained matrix using double internal format
typedef Eigen::SparseMatrix<double> sMat;

/// Sparse unconstrained matrix using double internal format and Row Major storage
typedef Eigen::SparseMatrix<double, Eigen::RowMajor> sRMat;


//--------------
//-- Function --
//--------------
template<typename TMat, typename TVec>
TMat cross_product_matrix(const TVec& x)
{
    TMat X;
    X << 0,     -x(2),  x(1),
         x(2),   0,    -x(0),
        -x(1),   x(0),  0;
    return X;
}

/**
 * @brief Compute R(diagonal), Q(orthogonal)
 * @tparam T Type of the number to decompose
 * @param M Input matrix
 * @return void
 * current implementation might be wrong
 */
template<typename T>
inline int rq(T M, T& R, T& Q)
{
    Eigen::FullPivLU<T> lu_decomp(M);
    if(lu_decomp.rank() != 3)
    {
        std::cerr << "Rank of M is not 3, cannot proceed to decompose" << std::endl;
        return 1;
    }
    Eigen::HouseholderQR<T> qr(M.rowwise().reverse().transpose());
    Q = qr.householderQ();
    R = M * Q.inverse();
    Eigen::FullPivLU<T> lu_decompR(R);
    R = R.rowwise().reverse().transpose().eval();
    Q = Q.transpose().eval();
    
    T W;
//    W = R.unaryExpr(std::ptr_fun(sign_func));compute the sign
    R = R * W;
    Q = W * R;
    
    return 0;
}
    
template <typename TMat, typename TVec>
float nullspace(TMat *A, TVec *x)
{
    if ( A->rows() >= A->cols() )
    {
        Eigen::JacobiSVD<TMat> svd( *A, Eigen::ComputeFullV );
        ( *x ) = svd.matrixV().col( A->cols() - 1 );
        return svd.singularValues()( A->cols() - 1 );
    }
    // More columns than rows, extend A with rows of zeros to make it square.
    TMat A_extended( A->cols(), A->cols() );
    A_extended.block( A->rows(), 0, A->cols() - A->rows(), A->cols() ).setZero();
    A_extended.block( 0, 0, A->rows(), A->cols() ) = ( *A );
    return nullspace(&A_extended, x);
}
    
template <typename T>
T nullspace(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>* A,
            Eigen::Matrix<T, Eigen::Dynamic, 1>* x)
{
    typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> TMat;
    typedef Eigen::Matrix<T, Eigen::Dynamic, 1> TVec;
    if (A->rows() >= A->cols())
    {
        Eigen::JacobiSVD<TMat> svd(*A, Eigen::ComputeFullV);
        (*x) = svd.matrixV().col(A->cols() - 1);
        return (T)svd.singularValues()(A->cols() - 1);
    }
    // Extend A with rows of zeros to make it square. It's a hack, but is
    // necessary until Eigen supports SVD with more columns than rows.
    TMat A_extended(A->cols(), A->cols());
    A_extended.block<A->cols() - A->rows(), A->cols()>(A->rows(), 0).setZero();
    A_extended.block<A->rows(), A->cols()>(0, 0) = (*A);
    return nullspace(&A_extended, x);
}

template <typename T>
inline double Nullspace2(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> *A,
                         Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> *x1,
                         Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> *x2 )
{
    typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> TMat;
    typedef Eigen::Matrix<T, Eigen::Dynamic, 1> TVec;
    if ( A->rows() >= A->cols() )
    {
        Eigen::JacobiSVD<TMat> svd(*A, Eigen::ComputeFullV);
        TMat V = svd.matrixV();
        *x1 = V.col(A->cols() - 1);
        *x2 = V.col(A->cols() - 2);
        return svd.singularValues()(A->cols() - 1);
    }
    // Extend A with rows of zeros to make it square. It's a hack, but is
    // necessary until Eigen supports SVD with more columns than rows.
    TMat A_extended(A->cols(), A->cols());
    A_extended.block<A->cols() - A->rows(), A->cols()>(A->rows(), 0).setZero();
    A_extended.block<A->rows(), A->cols()>(0, 0) = (*A);
    return Nullspace2(&A_extended, x1, x2);
}
    
template <typename T>
void svd(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>* A,
         Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>* U,
         Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>* S,
         Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>* V)
{
    
}

template <typename Mat0, typename Mat1, typename Mat2>
void svd(Mat0 *A, Mat1 *U, Mat2 *S, Mat2 *V)
{
    if (A->rows() >= A->cols())
    {
        Eigen::JacobiSVD<Mat0> asvd(*A, Eigen::ComputeFullU | Eigen::ComputeFullV);
        if (U)
        {
            (*U) = asvd.matrixU();
        }
        if (S)
        {
            (*S) = asvd.singularValues().diagonal();
        }
        if (V)
        {
            (*V) = asvd.matrixV();
        }
    }
    else
    {
        Mat1 A_extended(A->cols(), A->cols());
        A_extended.block<A->rows(), A->cols()>(0, 0) = (*A);
        A_extended.block<A->cols() - A->rows(), A->cols()>(A->rows(), 0).setZero();
        svd(&A_extended, U, S, V);
    }
}

template<typename T>
inline T square( T x )
{
  return x * x;
}

} // namespace open3DCV


#endif  // OPENMVG_NUMERIC_NUMERIC_H
