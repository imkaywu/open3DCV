#ifndef preprocess_h_
#define preprocess_h_

#include "math/numeric.h"

namespace open3DCV
{

template<typename T>
inline Mat3f normalize_pts(const std::vector<T>& pts, std::vector<T>& newpts)
{
    newpts.resize(pts.size());
    T c(0, 0);
    for (int i = 0; i < pts.size(); ++i)
    {
        c += pts[i];
    }
    c /= pts.size();
    
    float dist = 0.0f;
    for (int i = 0; i < pts.size(); ++i)
    {
        newpts[i] = pts[i] - c;
        dist += newpts[i].norm();
    }
    dist /= pts.size();
    
    float scale = sqrt(2) / dist;
    for (int i = 0; i < pts.size(); ++i)
    {
        newpts[i] *= scale;
    }
    
    Mat3f M;
    M << scale, 0,     -scale * c(0),
         0,     scale, -scale * c(1),
         0,     0,      1;
    
    return M;
}
  
// not used
inline Mat3f normalize_pts(const Mat2f& pts, Mat2f& newpts)
{
    newpts.resize(pts.rows(), pts.cols());
    Vec2f c(0, 0);
    c << pts.row(0).mean(),
    pts.row(1).mean();
    
    Vecf w = Vecf::Ones(pts.size());
    Mat2Xf C = c * w.transpose();
    newpts = pts - C;
    
    float dist = newpts.colwise().norm().mean();
    
    float scale = std::sqrt(2) / dist;
    Mat3f T;
    T << scale, 0,     -scale * c(0),
         0,     scale, -scale * c(1),
         0,     0,      1;
    
    Mat3Xf tmpts = Mat3Xf::Ones(3, pts.cols());
    tmpts.block(0, 0, 2, pts.cols()) = pts;
    newpts = (T * tmpts).block(0, 0, 2, pts.cols());
    
    return T;
}
   
template<typename inT, typename outT>
inline void vector2mat(const std::vector<inT>& idata, outT& odata)
{
    if (idata[0].rows() != odata.rows())
    {
        std::cerr << "Inputs don't have consistent type or dimension!" << std::endl;
        return;
    }
    
    odata.resize(idata[0].rows(), idata.size());
    for (int i = 0; i < idata.size(); ++i)
    {
        odata.col(i) = idata[i];
    }
}

template<typename inT, typename outT>
inline void mat2vector(const inT& idata, std::vector<outT>& odata)
{
    if(idata.rows() != odata[0].rows()) // check underlying type
    {
        std::cerr << "Inputs don't have consistent type or dimension!" << std::endl;
        return;
    }
    
    odata.resize(idata.cols());
    for (int i = 0; i < idata.cols(); ++i)
    {
        odata[i] = idata.col(i);
    }
}

}

#endif preprocess_h_
