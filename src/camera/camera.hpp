#ifndef open3dcv_image_hpp
#define open3dcv_image_hpp

#include <iostream>
#include <vector>
#include "Eigen/Dense"

using std::vector;
using std::string;
using Eigen::Matrix3f;
using Eigen::Matrix3Xf;
using Eigen::Matrix4f;
using Eigen::Vector3f;
using Eigen::Vector4f;

namespace open3DCV
{
class Camera
{
public:
    Camera();
    virtual ~Camera();
    
    // IO: read/write from file
    void readCamera(const string file);
    void writeCamera(const string file);
    
    // intrinsic, extrinsic parameters, K, R, t conversion
    int decompose(Matrix3f& K, Matrix3f& R) const;
    void setC(Vector3f& c) const;
    void setK(Matrix3f& K) const;
    void setR(Matrix3f& R) const;
    void setRt(Matrix4f& Rt) const;
    
    // get camera coordinate system
    void getAxes();
    
    // quaternion, rotation
    static void proj2quat(Matrix4f& proj, float q[6]);
    static void quat2proj(const float q[6], Matrix4f& proj);
    
    Vector3f project(const Vector4f& coord) const;
    Vector4f unproject(const Vector3f& icoord) const;
    float computeDepth(const Vector4f& coord) const;
    //----------------------------------------------------------------------------------------
    
//    virtual void init(const string cname);
    // update all camera-related parameters: projection matrices, and various axes
    void updateCamera();
    // update projection matrices from intrinsics and extrinsics
    void updateProjection();
    // set projection matrics from intrinsics and extrinsics
    void setProjection(const vector<float>& intrinsics, const vector<float>& extrinsics, Matrix3Xf& projection, const int txtType);
    // get
    float getScale(const Vector4f& coord, const int level) const;
    // get patch axes
    void getPAxes(const Vector4f& coord, const Vector4f& normal, Vector4f& pxaxis, Vector4f& pyaxis, const int level = 0) const;
    void setAxisScale(const float axisScale);
    
    // text file name of camera parameters
    string m_cname;
    // camera pamameter type
    int m_txtType;
    // optical center
    Vector4f m_center;
    // optical axis
    Vector4f m_oaxis;
    // x-axis of the camera-centered coordinate system
    Vector3f m_xaxis;
    // y-axis of the camera-centered coordinate system
    Vector3f m_yaxis;
    // z-axis of the camera-centered coordinate system
    Vector3f m_zaxis;
    // 3x4 projection matrix
    Matrix3Xf m_projection;
    // intrinsic and extrinsic camera parameters
    vector<float> m_intrinsics;
    vector<float> m_extrinsics;
    // image plane scale (fx + fy), used to compute the projection/scene-image scale
    float m_ipscale;
protected:
    float m_axisScale;
    
    Vector4f getCameraCenter() const;
};
    
} // end of namespace open3DCV

#endif /* camera_hpp */
