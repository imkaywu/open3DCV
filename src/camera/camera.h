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
    Camera(const std::vector<float>& r_intrinsics, const std::vector<float>& r_extrinsics);
    Camera(const Mat34f& r_projection);
    Camera(const Mat3f& R, const Vec3f& t);
    Camera(const Mat3f& K, const Mat3f& R, const Vec3f& t);
    virtual ~Camera();
    
    void update_projection();
    void update_parameters();
    void update_matrices(const int is_proj = 1);
    void update_center();
    void update_axes();
    
    int decompose(Mat3f & K, Mat3f& R) const;

    
    const Mat34f& projection() const;
    Mat34f& projection();
    const Vec3f& direction() const;
    Vec3f& direction();
    const Vec3f& center() const;
    Vec3f& center();
    
    Vec3f pixel2ray(const Vec2f& pixel) const;
    Vec3f project(const Vec4f& coord) const;
    Vec4f unproject(const Vec3f& icoord) const;
    float compute_depth(const Vec4f& coord) const;

//    float getScale(const Vec4f& coord, const int level) const;
    // get patch axes
//    void getPAxes(const Vecf& coord, const Vec4f& normal, Vec4f& pxaxis, Vec4f& pyaxis, const int level = 0) const;
//    void setAxisScale(const float axisScale);
    
private:
    Vec3f center_; // optical center
    Vec3f oaxis_; // optical axis
    Vec3f xaxis_; // x-axis of the camera-centered coordinate system
    Vec3f yaxis_; // y-axis of the camera-centered coordinate system
    Vec3f zaxis_; // z-axis of the camera-centered coordinate system
    Mat34f projection_; // projection matrix
    Mat3f K_; // intrinsic matrix
    Mat3f R_; // rotation
    Vec3f om_; // axis-angle
    Vec3f t_; // translation
    std::vector<float> intrinsics_; // intrinsic params: f_x, f_y, c_x, c_y, alpha
    std::vector<float> extrinsics_; // extrinsic params: om(0), om(1), om(2), t(0), t(1), t(2);
    
    // currently not used, from PMVS
    float ip_scale_; // image plane scale (fx + fy), used to compute the projection/scene-image scale
    float axis_scale_;
};
    
} // end of namespace open3DCV

#endif
