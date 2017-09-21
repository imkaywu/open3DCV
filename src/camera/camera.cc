//
//  camera.cpp
//  PMMVPS
//
//  Created by KaiWu on Oct/20/16.
//  Copyright © 2016 KaiWu. All rights reserved.
//

#include "camera/camera.h"
#include "math/numeric.h"
#include "transform/rodrigues.h"
#include <fstream>
#include <climits>

using std::vector;
using std::string;
using std::cerr;
using std::endl;
using std::ifstream;
using std::ofstream;

namespace open3DCV
{
    
    Camera::Camera()
    {
//        axis_scale_ = 1.0f;
    }
    
    Camera::Camera(const vector<float>& r_intrinsics, const vector<float>& r_extrinsics) : intrinsics_(r_intrinsics), extrinsics_(r_extrinsics)
    {
        // order does matter
        update_matrices(0);
        update_projection();
        update_center();
        update_axes();
    }
    
    Camera::Camera(const Mat34f& r_projection) : projection_(r_projection)
    {
        // order does matter
        update_matrices(); // decompose projection_ to K_, R_, and t_
        update_parameters();
        update_center();
        update_axes();
    }
    
    Camera::Camera(const Mat3f& K, const Mat3f& R, const Vec3f& t) : K_(K), R_(R), t_(t)
    {
        // order doesn't matter
        update_projection();
        update_parameters();
        update_center();
        update_axes();
    }
    
    Camera::~Camera()
    {
        // no op
    }
    
    void Camera::update_projection()
    {
        projection_.block<3, 3>(0, 0) = K_ * R_;
        projection_.block<3, 1>(0, 3) = K_ * t_;
    }
    
    void Camera::update_parameters()
    {
        intrinsics_[0] = K_(0, 0);
        intrinsics_[1] = K_(1, 1);
        intrinsics_[2] = K_(0, 2);
        intrinsics_[3] = K_(1, 2);
        intrinsics_[4] = K_(0, 1);
        
        irodrigues(om_, nullptr, R_);
        
        extrinsics_[0] = om_(0);
        extrinsics_[1] = om_(1);
        extrinsics_[2] = om_(2);
        extrinsics_[3] = t_(0);
        extrinsics_[4] = t_(1);
        extrinsics_[5] = t_(2);
    }
    
    void Camera::update_matrices(const int is_proj)
    {
        // from projection to parameters
        if (is_proj)
        {
            KRt_from_P();
        }
        else
        {
            K_ << intrinsics_[0], intrinsics_[4], intrinsics_[2],
                  0,              intrinsics_[1], intrinsics_[3],
                  0,              0,               1;
            
            om_ << extrinsics_[0], extrinsics_[1], extrinsics_[2];
            rodrigues(R_, nullptr, om_);
            
            t_ << extrinsics_[3], extrinsics_[4], extrinsics_[5];
        }
    }
    
    // always after update_projection
    void Camera::update_center()
    {
        /*
        Mat4f P_homog;
        P_homog.block<3, 4>(0, 0) = projection_;
        P_homog.block<1, 4>(3, 0).setZero();
        Vec4f c_homog;
        nullspace<Mat4f, Vec4f>(&P_homog, &c_homog);
        center_ = c_homog.block<3, 1>(0, 0) / c_homog(3);
         */
        
        Mat3f M;
        M = projection_.block<3, 3>(0, 0);
        
        center_ = -M.inverse() * projection_.col(3);
    }
    
    // always after update_projection
    void Camera::update_axes()
    {
        Vec4f oaxis = projection_.row(2);
        oaxis_ = oaxis.block<3, 1>(0, 0);
        oaxis_ /= oaxis.head(3).norm();
        
        zaxis_ = Vec3f(oaxis_);
        xaxis_ = Vec3f(projection_.row(0).head(3));
        yaxis_ = zaxis_.cross(xaxis_);
        yaxis_ /= yaxis_.norm();
        xaxis_ = yaxis_.cross(zaxis_);
    }
    
    void Camera::P_from_KRt()
    {
        
    }
    
    void Camera::KRt_from_P()
    {
        
    }
    
    const Mat34f& Camera::projection() const
    {
        return projection_;
    }
    
    Mat34f& Camera::projection()
    {
        return projection_;
    }
    
    const Vec3f& Camera::direction() const
    {
        return oaxis_;
    }
    
    Vec3f& Camera::direction()
    {
        return oaxis_;
    }
    
    const Vec3f& Camera::center() const
    {
        return center_;
    }
    
    Vec3f& Camera::center()
    {
        return center_;
    }

    int Camera::decompose(Mat3f& K, Mat3f& R) const
    {
        Mat3f M;
        M = projection_.block(0, 0, 3, 3);
        int ret = rq(M, K, R);
        if(ret)
        {
            cerr << "decomposition not successful" << endl;
            return 1;
        }
        
        return 0;
    }
    
    Vec3f Camera::project(const Vec4f& coord) const {
        Vec3f icoord = projection_ * coord;
        
        if(icoord(2) <= 0.0) {
            icoord << -0xffff, -0xffff, -1.0f; // probably need more bytes?
            return icoord;
        }
        else {
            icoord = icoord / icoord(2); // be careful of the aliasing issue
        }
        
        // copied diretly from PMVS camera.h, not sure if we need it
        icoord(0) = std::max((float)(INT_MIN + 3.0f), std::min((float)(INT_MAX - 3.0f), icoord(0)));
        icoord(1) = std::max((float)(INT_MIN + 3.0f), std::min((float)(INT_MAX - 3.0f), icoord(1)));
        
        return icoord;
    }
    
    // seems not used
    Vec4f Camera::unproject(const Vec3f& icoord) const {
        Vec3f b(icoord); // the third element should store the depth info
        Mat3f M = projection_.block(0, 0, 3, 3);
        Vec3f p = projection_.col(3);
        b = b - p;
        Vec4f coord = Vec4f::Ones();
        coord.head(3) = M.inverse() * b;
        return coord;
    }
    
    float Camera::compute_depth(const Vec4f& coord) const {
        if (projection_(2, 0) == 0.0f && projection_(2, 1) == 0.0f && projection_(2, 2) == 0.0f) {
            return -center_.homogeneous().dot(coord);
        }
        else {
            return oaxis_.homogeneous().dot(coord);
        }
    }
    
    // Note: the last coordinate of coord should be 1
    //float Camera::getScale(const Vec4f& coord, const int level) const {
    //    if (m_maxLevel <= level) {
    //        cerr << "Level is not within a range: " << level << " " << m_maxLevel << endl;
    //        exit(1);
    //    }
    //    
    //    float scale;
    //    if (projection_s[0](2, 0) == 0.0f && projection_s[0](2, 1) == 0.0f && projection_s[0](2, 2) == 0.0f) {
    //        // for orthographic case
    //    }
    //    else {
    //        Vec4f ray = coord - center_;
    //        scale = ray.norm() * (0x0001 << level) / m_ipscale;
    //    }
    //    return scale;
    //}
    
    //void Camera::getPAxes(const Vec4f& coord, const Vec4f& normal, Vec4f& pxaxis, Vec4f& pyaxis, const int level) const {
    //    const float pscale = getScale(coord, level); // projection scale?
    //    
    //    Vec3f normal3(normal(0), normal(1), normal(2));
    //    Vec3f yaxis3 = normal3.cross(xaxis_);
    //    yaxis3 /= yaxis3.norm();
    //    Vec3f xaxis3 = yaxis3.cross(normal3);
    //    
    //    pxaxis << xaxis3(0), xaxis3(1), xaxis3(2), 0.0f;
    //    pyaxis << yaxis3(0), yaxis3(1), yaxis3(2), 0.0f;
    //    pxaxis *= pscale;
    //    pyaxis *= pscale;
    //    
    //    const float xdis = (project(coord + pxaxis, level) - project(coord, level)).norm();
    //    const float ydis = (project(coord + pyaxis, level) - project(coord, level)).norm();
    //    pxaxis *= m_axisScale / xdis;
    //    pyaxis *= m_axisScale / ydis;
    //}
    //
    //void Camera::setAxisScale(const float axisScale) {
    //    m_axisScale = axisScale;
    //}

}
