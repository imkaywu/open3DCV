//
//  camera.cpp
//  PMMVPS
//
//  Created by KaiWu on Oct/20/16.
//  Copyright © 2016 KaiWu. All rights reserved.
//

#include "camera.h"
#include "numeric.h"
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
        axisScale_ = 1.0f;
    }
    
    Camera::Camera(const vector<float>& r_intrinsics, const vector<float>& r_extrinsics) : intrinsics_(r_intrinsics), extrinsics_(r_extrinsics)
    {
        K_ << r_intrinsics[0], r_intrinsics[4], r_intrinsics[2],
              0,               r_intrinsics[1], r_intrinsics[3],
              0,               0,               1;
        // rogrigues
        
        t_ << r_extrinsics[3], r_extrinsics[4], r_extrinsics[5];
    }
    
    Camera::Camera(const Mat34f& r_projection) : projection_(r_projection)
    {
        // decompose projection_ to K_, R_, and t_
    }
    
    Camera::Camera(const Mat3f& K, const Mat3f& R, const Vec3f& t) : K_(K), R_(R), t_(t)
    {
        projection_.block<3, 3>(0, 0) = K * R;
        projection_.block<3, 1>(0, 3) = K * t;
        
        // from K to intrinsics_
        
        // from R, t to extrinsics_
    }
    
    Camera::~Camera()
    {
        // no op
    }
    
    void Camera::readCamera(const string cname)
    {
        cname_ = cname;
        
        intrinsics_.resize(6);
        extrinsics_.resize(6);
        projection_.resize(3, 4);
        
        ifstream ifstr;
        ifstr.open(cname.c_str());
        string header;
        ifstr >> header;
        if (header == "CONTOUR")
            param_type_ = 0;
        else if (header == "CONTOUR2")
            param_type_ = 2;
        else if (header == "CONTOUR3")
            param_type_ = 3;
        else
        {
            cerr << "Unrecognizable text format" << endl;
            exit(1);
        }
        switch (param_type_)
        {
            case 0:
                for (int i = 0; i < 6; ++i)
                    ifstr >> intrinsics_[i];
                for (int i = 0; i < 6; ++i)
                    ifstr >> extrinsics_[i];
                break;
            case 2:
// read file
//            projection_.block(0, 0, 3, 3) = getK() * getR();
//            projection_.block(0, 3, 1, 3) = -getR() * getC();
                break;
            default:
                break;
        }
        ifstr.close();
        
        updateProjection();
    }
    
    void Camera::writeCamera(const string file)
    {
        ofstream ofstr;
        ofstr.open(cname_.c_str());
        
        switch(param_type_)
        {
            case 0:
                ofstr << "CONTOUR\n"
                << projection_(0, 0) << " " << projection_(0, 1) << " " << projection_(0, 2) << " " << projection_(0, 3) << endl
                << projection_(1, 0) << " " << projection_(1, 1) << " " << projection_(1, 2) << " " << projection_(1, 3) << endl
                << projection_(2, 0) << " " << projection_(2, 1) << " " << projection_(2, 2) << " " << projection_(2, 3);
                break;
            case 1:
                break;
            default:
                break;
        }
        ofstr.close();
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
    
    void Camera::updateProjection()
    {
        float params[12];
        for (int i = 0; i < 6; ++i)
        {
            params[i] = intrinsics_[i];
            params[i + 6] = extrinsics_[i];
        }
        
        if (param_type_ == 0)
        {
            for (int y = 0; y < 3; ++y)
            {
                for (int x = 0; x < 4; ++x)
                {
                    projection_(y, x) = params[4 * y + x];
                    //                decompose();
                }
            }
        }
        else if (param_type_ == 2) {
            Mat4f K;
            K << params[0], params[2], params[3], 0.0f,
            0.0f, params[1], params[4], 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f;
            
            Mat4f Rttmp;
            quat2proj(&params[6], Rttmp);
            Rttmp = K * Rttmp;
            
            for (int y = 0; y < 3; ++y) {
                for (int x = 0; x < 4; ++x) {
                    projection_(y, x) = Rttmp(y, x);
                }
            }
        }
        else if (param_type_ == 3) {
            // to be written
        }
        else {
            cerr << "Impossible setProjection" << endl;
            exit(1);
        }
    }
    
    void Camera::getAxes()
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
    
    void Camera::setC(Vec3f& c) const
    {
        Mat3f M;
        M = projection_.block(0, 0, 3, 3);
        
        c = M.inverse() * projection_.col(3);
    }
    
    void Camera::setK(Mat3f& K) const {
        if (param_type_ != 2) {
            cerr << "gerK not supported for txtType" << param_type_ << endl;
            exit(1);
        }
        
        K = Mat3f::Zero();
        K(0, 0) = intrinsics_[0];
        K(1, 1) = intrinsics_[1];
        K(0, 1) = intrinsics_[2];
        K(0, 2) = intrinsics_[3];
        K(1, 2) = intrinsics_[4];
        K(2, 2) = 1.0f;
    }
    
    void Camera::setRt(Mat4f& Rt) const {
        if (param_type_ != 2) {
            cerr << "gerRT not supported for txtType: " << param_type_ << endl;
            exit(1);
        }
        
        float params[6];
        for (int i = 0; i < 6; ++i) {
            params[i] = extrinsics_[i];
        }
        
        Mat4f Rttmp;
        quat2proj(params, Rttmp);
        
        for (int y = 0; y < 4; ++y) {
            for (int x = 0; x < 4; ++x) {
                Rt(y, x) = Rttmp(y, x);
            }
        }
    }
    
    void Camera::setR(Mat3f &R) const {
        if (param_type_ != 2) {
            cerr << "Not supported: " << param_type_ << endl;
            exit(1);
        }
        
        float params[6];
        for (int i = 0; i < 6; ++i) {
            params[i] = extrinsics_[i];
        }
        
        Mat4f Rttmp;
        quat2proj(params, Rttmp);
        for (int y = 0; y < 3; ++y) {
            for (int x = 0; x < 3; ++x) {
                R(y, x) = Rttmp(y, x);
            }
        }
    }
    
    //----------------------------------------------------------------------------------------
    
    //void Camera::init(const string cname) {
    //    m_cname = cname;
    //
    //    intrinsics_.resize(6);
    //    extrinsics_.resize(6);
    //
    //    ifstream ifstr;
    //    ifstr.open(cname.c_str());
    //    string header;
    //    ifstr >> header;
    //    if (header == "CONTOUR")
    //        param_type_ = 0;
    //    else if (header == "CONTOUR2")
    //        param_type_ = 2;
    //    else if (header == "CONTOUR3")
    //        param_type_ = 3;
    //    else {
    //        cerr << "Unrecognizable text format" << endl;
    //        exit(1);
    //    }
    //
    //    for (int i = 0; i < 6; ++i)
    //        ifstr >> intrinsics_[i];
    //    for (int i = 0; i < 6; ++i)
    //        ifstr >> extrinsics_[i];
    //
    //    ifstr.close();
    //
    //    // initialize projection matrices, CoP and various axes
    //    projection_s.resize(maxLevel);
    //    for (int level = 0; level < maxLevel; ++level) {
    //        projection_s[level].resize(3, 4);
    //    }
    //    updateCamera();
    //}
    
    //void Camera::updateCamera() {
    //    updateProjection();
    //
    //    oaxis_ = projection_s[0].row(2);
    //    oaxis_ /= oaxis_.head(3).norm();
    //
    //    center_ = getCameraCenter();
    //
    //    zaxis_ = Vec3f(oaxis_.head(3));
    //    xaxis_ = Vec3f(projection_s[0].row(0).head(3));
    //    yaxis_ = zaxis_.cross(xaxis_);
    //    yaxis_ /= yaxis_.norm();
    //    xaxis_ = yaxis_.cross(zaxis_);
    //
    //    // ??? the scale of the x,y-axis of the patch projected onto the image plane
    //    Vec4f xaxis = projection_s[0].row(0);
    //    xaxis(3) = 0.0f;
    //    Vec4f yaxis = projection_s[0].row(1);
    //    yaxis(3) = 0.0f;
    //    float scale = (xaxis.norm() + yaxis.norm()) / 2.0f;
    //    if (scale == 0.0f) {
    //        scale = 1.0f;
    //    }
    //    m_ipscale = scale;
    //}
    
    //void Camera::updateProjection() {
    //    // set bottom level
    //    setProjection(intrinsics_, extrinsics_, projection_s[0], param_type_);
    //
    //    for (int level = 1; level < m_maxLevel; ++level) {
    //        projection_s[level] = projection_s[level - 1];
    //        projection_s[level].row(0) /= 2.0f;
    //        projection_s[level].row(1) /= 2.0f;
    //    }
    //}
    
    //void Camera::setProjection(const vector<float>& intrinsics, const vector<float>& extrinsics, Matrix3Xf& projection, const int txtType) {
    //    float params[12];
    //    for (int i = 0; i < 6; ++i) {
    //        params[i] = intrinsics[i];
    //        params[i + 6] = extrinsics[i];
    //    }
    //
    //    if (txtType == 0) {
    //        for (int y = 0; y < 3; ++y) {
    //            for (int x = 0; x < 4; ++x) {
    //                projection(y, x) = params[4 * y + x];
    //            }
    //        }
    //    }
    //    else if (txtType == 2) {
    //        Mat4f K;
    //        K << params[0], params[2], params[3], 0.0f,
    //             0.0f, params[1], params[4], 0.0f,
    //             0.0f, 0.0f, 1.0f, 0.0f,
    //             0.0f, 0.0f, 0.0f, 1.0f;
    //
    //        Mat4f Rttmp;
    //        quat2proj(&params[6], Rttmp);
    //        Rttmp = K * Rttmp;
    //
    //        projection.resize(3, 4);
    //        for (int y = 0; y < 3; ++y) {
    //            for (int x = 0; x < 4; ++x) {
    //                projection(y, x) = Rttmp(y, x);
    //            }
    //        }
    //    }
    //    else if (txtType == 3) {
    //        // to be written
    //    }
    //    else {
    //        cerr << "Impossible setProjection" << endl;
    //        exit(1);
    //    }
    //}
    
    
    
    void Camera::proj2quat(Mat4f& proj, float q[6]) {
        float s;
        
        q[3] = proj(0, 3);
        q[4] = proj(1, 3);
        q[5] = proj(2, 3);
        q[0] = 0;
        q[1] = 0;
        q[2] = 0;
        if (proj(2, 0) == 1.0f) {
            q[1] = -M_PI / 2.0f;
            q[2] = 0.0f;
            q[0] = atan2f(-proj(0, 1), proj(1, 1));
        }
        else {
            if (proj(2, 0) == -1.0f) {
                q[1] = M_PI / 2.0f;
                q[2] = 0.0f;
                q[0] = atan2f(proj(0, 1), proj(1, 1));
            }
            else {
                q[1] = asinf(-proj(2, 0));
                if (cosf(q[1]) > 0.0f) {
                    s = 1.0f;
                }
                else {
                    s = -1.0f;
                }
                q[0] = atan2f(proj(2, 1) * s, proj(2, 2) * s);
                q[2] = atan2f(proj(1, 0) * s, proj(0, 0) * s);
            }
        }
        q[0] = q[0] * 180.0f / M_PI;//RadInDeg;
        q[1] = q[1] * 180.0f / M_PI;//RadInDeg;
        q[2] = q[2] * 180.0f / M_PI;//RadInDeg;
        for(int i = 0; i < 3; i++){
            if (fabsf(q[i]) > 180.0f){
                q[i] = (q[i] > 0) ? q[i] - 360.0f : q[i] + 360.0f;
            }
        }
    }
    
    void Camera::quat2proj(const float q[6], Mat4f &proj) {
        const float a = q[0] * M_PI / 180.0;
        const float b = q[1] * M_PI / 180.0;
        const float g = q[2] * M_PI / 180.0;
        
        const float s1 = sinf(a);  const float s2 = sinf(b);  const float s3 = sinf(g);
        const float c1 = cosf(a);  const float c2 = cosf(b);  const float c3 = cosf(g);
        
        /*   Premiere colonne*/	/*   Seconde colonne	*/
        proj(0, 0) = c2 * c3; 		proj(0, 1) = c3 * s2 * s1 - s3 * c1;
        proj(1, 0) = s3 * c2; 		proj(1, 1) = s3 * s2 * s1 + c3 * c1;
        proj(2, 0) = -s2;   		proj(2, 1) = c2 * s1;
        
        /*   Troisieme colonne*/	/*  Quatrieme colonne	*/
        proj(0, 2) = c3 * s2 * c1 + s3 * s1; 	proj(0, 3) = q[3];
        proj(1, 2) = s3 * s2 * c1 - c3 * s1; 	proj(1, 3) = q[4];
        proj(2, 2) = c2 * c1;                   proj(2, 3) = q[5];
        
        proj(3, 0) = proj(3, 1) = proj(3, 2) = 0.0f;
        proj(3, 3) = 1.0f;
    }
    
    //void Camera::writeCamera(const string file) {
    //    ofstream ofstr;
    //    ofstr.open(file.c_str());
    //    if(param_type_ == 0) {
    //        ofstr << "CONTOUR" << endl
    //            << intrinsics_[0] << ' ' << intrinsics_[1] << ' '
    //            << intrinsics_[2] << ' ' << intrinsics_[3] << endl
    //            << intrinsics_[4] << ' ' << intrinsics_[5] << ' '
    //            << extrinsics_[0] << ' ' << extrinsics_[1] << endl
    //            << extrinsics_[2] << ' ' << extrinsics_[3] << ' '
    //            << extrinsics_[4] << ' ' << extrinsics_[5] << endl;
    //    }
    //    else if(param_type_ == 2) {
    //        ofstr << "CONTOUR2" << endl;
    //        for (int i = 0; i < 6; ++i)
    //            ofstr << intrinsics_[i] << ' ';
    //        ofstr << endl;
    //        for (int i = 0; i < 6; ++i)
    //            ofstr << extrinsics_[i] << ' ';
    //        ofstr << endl;
    //    }
    //    else if(param_type_ == 3) {
    //        // to be written
    //    }
    //    else {
    //        cerr << "Incorrect txt type" << endl;
    //        exit(1);
    //    }
    //    ofstr.close();
    //}
    
    // Note: the last coordinate of the camera center is 1
    //Vec4f Camera::getCameraCenter() const {
    //    Vec4f center;
    //
    //    // orthographic camera
    //    if(projection_s[0](2, 0) == 0.0f && projection_s[0](2, 1) == 0.0f && projection_s[0](2, 2) == 0.0f) {
    //        // not yet implemented
    //    } else {
    //        Mat3f M = projection_s[0].block(0, 0, 3, 3);
    //        Vec3f q = projection_s[0].col(3);
    //        Vec3f center3 = -M.inverse() * q;
    //        center << center3(0), center3(1), center3(2), 1.0f;
    //    }
    //    return center;
    //}
    
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
    
    float Camera::computeDepth(const Vec4f& coord) const {
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
