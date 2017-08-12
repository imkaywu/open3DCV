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

using namespace open3DCV;
using std::vector;
using std::string;
using std::cerr;
using std::endl;
using std::ifstream;
using std::ofstream;
using Eigen::Matrix3f;
using Eigen::Matrix3Xf;
using Eigen::Matrix4f;
using Eigen::Vector3f;
using Eigen::Vector4f;

Camera::Camera()
{
    m_axisScale = 1.0f;
}

Camera::~Camera() { }

void Camera::readCamera(const string cname)
{
    m_cname = cname;
    
    m_intrinsics.resize(6);
    m_extrinsics.resize(6);
    m_projection.resize(3, 4);
    
    ifstream ifstr;
    ifstr.open(cname.c_str());
    string header;
    ifstr >> header;
    if (header == "CONTOUR")
        m_txtType = 0;
    else if (header == "CONTOUR2")
        m_txtType = 2;
    else if (header == "CONTOUR3")
        m_txtType = 3;
    else
    {
        cerr << "Unrecognizable text format" << endl;
        exit(1);
    }
    
    switch (m_txtType)
    {
        case 0:
            for (int i = 0; i < 6; ++i)
                ifstr >> m_intrinsics[i];
            for (int i = 0; i < 6; ++i)
                ifstr >> m_extrinsics[i];
            break;
        case 2:
            // read file
//            m_projection.block(0, 0, 3, 3) = getK() * getR();
//            m_projection.block(0, 3, 1, 3) = -getR() * getC();
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
    ofstr.open(m_cname.c_str());
    
    switch(m_txtType)
    {
        case 0:
            ofstr << "CONTOUR\n"
            << m_projection(0, 0) << " " << m_projection(0, 1) << " " << m_projection(0, 2) << " " << m_projection(0, 3) << endl
            << m_projection(1, 0) << " " << m_projection(1, 1) << " " << m_projection(1, 2) << " " << m_projection(1, 3) << endl
            << m_projection(2, 0) << " " << m_projection(2, 1) << " " << m_projection(2, 2) << " " << m_projection(2, 3);
            break;
        case 1:
            break;
        default:
            break;
    }
    ofstr.close();
}

void Camera::updateProjection()
{
    float params[12];
    for (int i = 0; i < 6; ++i)
    {
        params[i] = m_intrinsics[i];
        params[i + 6] = m_extrinsics[i];
    }
    
    if (m_txtType == 0)
    {
        for (int y = 0; y < 3; ++y)
        {
            for (int x = 0; x < 4; ++x)
            {
                m_projection(y, x) = params[4 * y + x];
//                decompose();
            }
        }
    }
    else if (m_txtType == 2) {
        Matrix4f K;
        K << params[0], params[2], params[3], 0.0f,
             0.0f, params[1], params[4], 0.0f,
             0.0f, 0.0f, 1.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 1.0f;
        
        Matrix4f Rttmp;
        quat2proj(&params[6], Rttmp);
        Rttmp = K * Rttmp;
        
        for (int y = 0; y < 3; ++y) {
            for (int x = 0; x < 4; ++x) {
                m_projection(y, x) = Rttmp(y, x);
            }
        }
    }
    else if (m_txtType == 3) {
        // to be written
    }
    else {
        cerr << "Impossible setProjection" << endl;
        exit(1);
    }
}

void Camera::getAxes()
{
    m_oaxis = m_projection.row(2);
    m_oaxis /= m_oaxis.head(3).norm();
    
    m_zaxis = Vector3f(m_oaxis.head(3));
    m_xaxis = Vector3f(m_projection.row(0).head(3));
    m_yaxis = m_zaxis.cross(m_xaxis);
    m_yaxis /= m_yaxis.norm();
    m_xaxis = m_yaxis.cross(m_zaxis);
}

int Camera::decompose(Matrix3f& K, Matrix3f& R) const
{
    Matrix3f M;
    M = m_projection.block(0, 0, 3, 3);
    int ret = rq(M, K, R);
    if(ret)
    {
        cerr << "decomposition not successful" << endl;
        return 1;
    }
    
    return 0;
}

void Camera::setC(Vector3f& c) const
{
    Matrix3f M;
    M = m_projection.block(0, 0, 3, 3);
    
    c = M.inverse() * m_projection.col(3);
}

void Camera::setK(Matrix3f& K) const {
    if (m_txtType != 2) {
        cerr << "gerK not supported for txtType" << m_txtType << endl;
        exit(1);
    }
    
    K = Matrix3f::Zero();
    K(0, 0) = m_intrinsics[0];
    K(1, 1) = m_intrinsics[1];
    K(0, 1) = m_intrinsics[2];
    K(0, 2) = m_intrinsics[3];
    K(1, 2) = m_intrinsics[4];
    K(2, 2) = 1.0f;
}

void Camera::setRt(Matrix4f& Rt) const {
    if (m_txtType != 2) {
        cerr << "gerRT not supported for txtType: " << m_txtType << endl;
        exit(1);
    }
    
    float params[6];
    for (int i = 0; i < 6; ++i) {
        params[i] = m_extrinsics[i];
    }
    
    Matrix4f Rttmp;
    quat2proj(params, Rttmp);
    
    for (int y = 0; y < 4; ++y) {
        for (int x = 0; x < 4; ++x) {
            Rt(y, x) = Rttmp(y, x);
        }
    }
}

void Camera::setR(Matrix3f &R) const {
    if (m_txtType != 2) {
        cerr << "Not supported: " << m_txtType << endl;
        exit(1);
    }
    
    float params[6];
    for (int i = 0; i < 6; ++i) {
        params[i] = m_extrinsics[i];
    }
    
    Matrix4f Rttmp;
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
//    m_intrinsics.resize(6);
//    m_extrinsics.resize(6);
//    
//    ifstream ifstr;
//    ifstr.open(cname.c_str());
//    string header;
//    ifstr >> header;
//    if (header == "CONTOUR")
//        m_txtType = 0;
//    else if (header == "CONTOUR2")
//        m_txtType = 2;
//    else if (header == "CONTOUR3")
//        m_txtType = 3;
//    else {
//        cerr << "Unrecognizable text format" << endl;
//        exit(1);
//    }
//    
//    for (int i = 0; i < 6; ++i)
//        ifstr >> m_intrinsics[i];
//    for (int i = 0; i < 6; ++i)
//        ifstr >> m_extrinsics[i];
//    
//    ifstr.close();
//    
//    // initialize projection matrices, CoP and various axes
//    m_projections.resize(maxLevel);
//    for (int level = 0; level < maxLevel; ++level) {
//        m_projections[level].resize(3, 4);
//    }
//    updateCamera();
//}

//void Camera::updateCamera() {
//    updateProjection();
//    
//    m_oaxis = m_projections[0].row(2);
//    m_oaxis /= m_oaxis.head(3).norm();
//    
//    m_center = getCameraCenter();
//    
//    m_zaxis = Vector3f(m_oaxis.head(3));
//    m_xaxis = Vector3f(m_projections[0].row(0).head(3));
//    m_yaxis = m_zaxis.cross(m_xaxis);
//    m_yaxis /= m_yaxis.norm();
//    m_xaxis = m_yaxis.cross(m_zaxis);
//    
//    // ??? the scale of the x,y-axis of the patch projected onto the image plane
//    Vector4f xaxis = m_projections[0].row(0);
//    xaxis(3) = 0.0f;
//    Vector4f yaxis = m_projections[0].row(1);
//    yaxis(3) = 0.0f;
//    float scale = (xaxis.norm() + yaxis.norm()) / 2.0f;
//    if (scale == 0.0f) {
//        scale = 1.0f;
//    }
//    m_ipscale = scale;
//}

//void Camera::updateProjection() {
//    // set bottom level
//    setProjection(m_intrinsics, m_extrinsics, m_projections[0], m_txtType);
//    
//    for (int level = 1; level < m_maxLevel; ++level) {
//        m_projections[level] = m_projections[level - 1];
//        m_projections[level].row(0) /= 2.0f;
//        m_projections[level].row(1) /= 2.0f;
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
//        Matrix4f K;
//        K << params[0], params[2], params[3], 0.0f,
//             0.0f, params[1], params[4], 0.0f,
//             0.0f, 0.0f, 1.0f, 0.0f,
//             0.0f, 0.0f, 0.0f, 1.0f;
//        
//        Matrix4f Rttmp;
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



void Camera::proj2quat(Matrix4f& proj, float q[6]) {
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

void Camera::quat2proj(const float q[6], Matrix4f &proj) {
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
//    if(m_txtType == 0) {
//        ofstr << "CONTOUR" << endl
//            << m_intrinsics[0] << ' ' << m_intrinsics[1] << ' '
//            << m_intrinsics[2] << ' ' << m_intrinsics[3] << endl
//            << m_intrinsics[4] << ' ' << m_intrinsics[5] << ' '
//            << m_extrinsics[0] << ' ' << m_extrinsics[1] << endl
//            << m_extrinsics[2] << ' ' << m_extrinsics[3] << ' '
//            << m_extrinsics[4] << ' ' << m_extrinsics[5] << endl;
//    }
//    else if(m_txtType == 2) {
//        ofstr << "CONTOUR2" << endl;
//        for (int i = 0; i < 6; ++i)
//            ofstr << m_intrinsics[i] << ' ';
//        ofstr << endl;
//        for (int i = 0; i < 6; ++i)
//            ofstr << m_extrinsics[i] << ' ';
//        ofstr << endl;
//    }
//    else if(m_txtType == 3) {
//        // to be written
//    }
//    else {
//        cerr << "Incorrect txt type" << endl;
//        exit(1);
//    }
//    ofstr.close();
//}

// Note: the last coordinate of the camera center is 1
//Vector4f Camera::getCameraCenter() const {
//    Vector4f center;
//    
//    // orthographic camera
//    if(m_projections[0](2, 0) == 0.0f && m_projections[0](2, 1) == 0.0f && m_projections[0](2, 2) == 0.0f) {
//        // not yet implemented
//    } else {
//        Matrix3f M = m_projections[0].block(0, 0, 3, 3);
//        Vector3f q = m_projections[0].col(3);
//        Vector3f center3 = -M.inverse() * q;
//        center << center3(0), center3(1), center3(2), 1.0f;
//    }
//    return center;
//}

Vector3f Camera::project(const Vector4f& coord) const {
    Vector3f icoord = m_projection * coord;
    
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
Vector4f Camera::unproject(const Vector3f& icoord) const {
    Vector3f b(icoord); // the third element should store the depth info
    Matrix3f M = m_projection.block(0, 0, 3, 3);
    Vector3f p = m_projection.col(3);
    b = b - p;
    Vector4f coord = Vector4f::Ones();
    coord.head(3) = M.inverse() * b;
    return coord;
}

float Camera::computeDepth(const Vector4f& coord) const {
	if (m_projection(2, 0) == 0.0f && m_projection(2, 1) == 0.0f && m_projection(2, 2) == 0.0f) {
		return -m_center.dot(coord);
	}
	else {
		return m_oaxis.dot(coord);
	}
}

// Note: the last coordinate of coord should be 1
//float Camera::getScale(const Vector4f& coord, const int level) const {
//    if (m_maxLevel <= level) {
//        cerr << "Level is not within a range: " << level << " " << m_maxLevel << endl;
//        exit(1);
//    }
//    
//    float scale;
//    if (m_projections[0](2, 0) == 0.0f && m_projections[0](2, 1) == 0.0f && m_projections[0](2, 2) == 0.0f) {
//        // for orthographic case
//    }
//    else {
//        Vector4f ray = coord - m_center;
//        scale = ray.norm() * (0x0001 << level) / m_ipscale;
//    }
//    return scale;
//}

//void Camera::getPAxes(const Vector4f& coord, const Vector4f& normal, Vector4f& pxaxis, Vector4f& pyaxis, const int level) const {
//    const float pscale = getScale(coord, level); // projection scale?
//    
//    Vector3f normal3(normal(0), normal(1), normal(2));
//    Vector3f yaxis3 = normal3.cross(m_xaxis);
//    yaxis3 /= yaxis3.norm();
//    Vector3f xaxis3 = yaxis3.cross(normal3);
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
