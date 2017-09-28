#include "sfm/structure_point.h"

namespace open3DCV
{
    Structure_Point::Structure_Point() : p_(0, 0, 0), c_(0, 0, 0)
    {
        // no op
    }
    
    Structure_Point::Structure_Point(const Vec3f& coord) : p_(coord), c_(0, 0, 0)
    {
        // no op
    }
    
    Structure_Point::Structure_Point(const Vec3f &coord, const Vec3i &color) : p_(coord), c_(color)
    {
        // no op
    }
    
    Structure_Point& Structure_Point::operator=(const Structure_Point& struct_pt)
    {
        p_ = struct_pt.p_;
        c_ = struct_pt.c_;
        return *this;
    }
    
    Structure_Point::~Structure_Point()
    {
        // no op
    }
    
    const Vec3f &Structure_Point::coords() const
    {
        return p_;
    }
    
    Vec3f &Structure_Point::coords()
    {
        return p_;
    }
    
    const Vec3i &Structure_Point::color() const
    {
        return c_;
    }
    
    Vec3i& Structure_Point::color()
    {
        return c_;
    }
}
