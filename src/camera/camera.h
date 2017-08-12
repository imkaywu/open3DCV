#ifndef open3dcv_image_h
#define open3dcv_image_h

#include <iostream>
#include <vector>
#include "math/numeric.h"

namespace open3DCV
{
class Camera
{
public:
    Camera();
    virtual ~Camera();
    
    // IO: read/write from file
    void readCamera(const std::string file);
    void writeCamera(const std::string file);
    
    // intrinsic, extrinsic parameters, K, R, t conversion
    int decompose(Mat3f & K, Mat3f& R) const;
    void setC(Vec3f& c) const;
    void setK(Mat3f& K) const;
    void setR(Mat3f& R) const;
    void setRt(Mat4f& Rt) const;
    
    // get camera coordinate system
    void getAxes();
    
    // quaternion, rotation
    static void proj2quat(Mat4f& proj, float q[6]);
    static void quat2proj(const float q[6], Mat4f& proj);
    
    Vec3f project(const Vec4f& coord) const;
    Vec4f unproject(const Vec3f& icoord) const;
    float computeDepth(const Vec4f& coord) const;
    //----------------------------------------------------------------------------------------
    
//    virtual void init(const string cname);
    // update all camera-related parameters: projection matrices, and various axes
    void updateCamera();
    // update projection matrices from intrinsics and extrinsics
    void updateProjection();
    // set projection matrics from intrinsics and extrinsics
    void setProjection(const std::vector<float>& intrinsics, const std::vector<float>& extrinsics, Mat3Xf& projection, const int txtType);
    // get
    float getScale(const Vec4f& coord, const int level) const;
    // get patch axes
    void getPAxes(const Vecf& coord, const Vec4f& normal, Vec4f& pxaxis, Vec4f& pyaxis, const int level = 0) const;
    void setAxisScale(const float axisScale);
    
    // text file name of camera parameters
    std::string m_cname;
    // camera pamameter type
    int m_txtType;
    // optical center
    Vec4f m_center;
    // optical axis
    Vec4f m_oaxis;
    // x-axis of the camera-centered coordinate system
    Vec3f m_xaxis;
    // y-axis of the camera-centered coordinate system
    Vec3f m_yaxis;
    // z-axis of the camera-centered coordinate system
    Vec3f m_zaxis;
    // 3x4 projection matrix
    Mat3Xf m_projection;
    // intrinsic and extrinsic camera parameters
    std::vector<float> m_intrinsics;
    std::vector<float> m_extrinsics;
    // image plane scale (fx + fy), used to compute the projection/scene-image scale
    float m_ipscale;
protected:
    float m_axisScale;
    
    Vec4f getCameraCenter() const;
};
    
} // end of namespace open3DCV

#endif /* camera_hpp */
