#ifndef structure_point_
#define structure_point_

namespace open3DCV
{
    class Structure_Point
    {
    public:
        Structure_Point();
        Structure_Point(const Vec3f &coord);
        Structure_Point(const Vec3f &coord, const Vec3i &color);
        
        const Vec3f &coords() const;
        Vec3f &coords();
        const Vec3i &color() const;
        Vec3i &color();
        
    protected:
        Vec3f p_; // The x,y,z coordinates, better is it's of double
        Vec3i c_; //!< The r,g,b color components (0-255 for each)
        
    };
        
    inline Structure_Point::Structure_Point() : p_(0, 0, 0), c_(0, 0, 0)
    {
        // no op
    }
    
    inline Structure_Point::Structure_Point(const Vec3f& coord) : p_(coord), c_(0, 0, 0)
    {
        // no op
    }
    
    inline Structure_Point::Structure_Point(const Vec3f &coord, const Vec3i &color) : p_(coord), c_(color)
    {
        // no op
    }
    
    inline const Vec3f &Structure_Point::coords() const
    {
        return p_;
    }
    
    inline Vec3f &Structure_Point::coords()
    {
        return p_;
    }
    
    inline const Vec3i &Structure_Point::color() const
    {
        return c_;
    }
    
    inline Vec3i& Structure_Point::color()
    {
        return c_;
    }
    
}

#endif // structure_point_h_
