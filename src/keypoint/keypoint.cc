#include "keypoint/keypoint.h"

namespace open3DCV
{
    Keypoint::Keypoint() : coords_(0,0), scale_(open3DCV_KEYPOINT_VAR), orientation_(open3DCV_KEYPOINT_VAR), index_(open3DCV_KEYPOINT_VAR), color_(0,0,0)
    {
        //no-op
    }
    Keypoint::Keypoint(const Vec2f &r_coords) : coords_(r_coords), scale_(open3DCV_KEYPOINT_VAR), orientation_(open3DCV_KEYPOINT_VAR), index_(open3DCV_KEYPOINT_VAR), color_(0,0,0)
    {
        //no-op
    }
    
    Keypoint::Keypoint(const Vec2f &r_coords, unsigned int r_i) : coords_(r_coords), index_(r_i), scale_(open3DCV_KEYPOINT_VAR), orientation_(open3DCV_KEYPOINT_VAR), color_(0,0,0)
    {
        //no-op
    }
    
    Keypoint::Keypoint(const Vec2f &r_coords, unsigned int r_i, const Vec3i& r_c) : coords_(r_coords), index_(r_i), color_(r_c), scale_(open3DCV_KEYPOINT_VAR), orientation_(open3DCV_KEYPOINT_VAR)
    {
        //no-op
    }
    
    Keypoint::Keypoint(const Vec2f &r_coords, const float s, const float o) : coords_(r_coords), scale_(s), orientation_(o), index_(open3DCV_KEYPOINT_VAR), color_(0,0,0)
    {
        //no-op
    }
    
    Keypoint::Keypoint(const Keypoint& key)
    {
        coords_ = key.coords();
        index_ = key.index();
        color_ = key.color();
        scale_ = key.scale();
        orientation_ = key.orientation();
    }
    
    Keypoint& Keypoint::operator=(const Keypoint& key)
    {
        coords_ = key.coords();
        index_ = key.index();
        color_ = key.color();
        scale_ = key.scale();
        orientation_ = key.orientation();
        return *this;
    }
    
    const Vec2f& Keypoint::coords() const {
        return coords_;
    }
    
    Vec2f& Keypoint::coords() {
        return coords_;
    }
    
    const unsigned int& Keypoint::index() const {
        return index_;
    }
    
    unsigned int& Keypoint::index() {
        return index_;
    }
    
    const Vec3i& Keypoint::color() const {
        return color_;
    }
    
    Vec3i& Keypoint::color() {
        return color_;
    }
    
    const double& Keypoint::scale() const {
        return scale_;
    }
    
    double& Keypoint::scale() {
        return scale_;
    }
    
    const int Keypoint::has_scale() const {
        return scale_ != open3DCV_KEYPOINT_VAR;
    }
    
    const double& Keypoint::orientation() const {
        return orientation_;
    }
    
    double& Keypoint::orientation() {
        return orientation_;
    }
    
    const int Keypoint::has_orientation() const {
        return orientation_ != open3DCV_KEYPOINT_VAR;
    }
    
    bool Keypoint::operator<(const Keypoint& rhs) const
    {
        return index() < rhs.index();
    }
    
    std::ostream& operator<<(std::ostream& ostr, const Keypoint& rhs)
    {
        ostr << rhs.coords().x() << " " << rhs.coords().y() << " " << rhs.index();
        return ostr;
    }
    
    std::istream& operator>>(std::istream& istr, Keypoint& rhs)
    {
        istr >> rhs.coords().x() >> rhs.coords().y() >> rhs.index();
        return istr;
    }
    
    int Keypoint::is_identical(const Keypoint& key1, const Keypoint& key2)
    {
        float dist = 1e-10;
        Vec2f dx = key1.coords() - key2.coords();
        
        if (key1.index() == key2.index() && sqrtf(dx.dot(dx)) < dist)
            return 1;
        
        return 0;
    }
}
