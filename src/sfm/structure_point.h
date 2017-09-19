#ifndef structure_point_
#define structure_point_

namespace open3DCV
{
    class Structure_Point
    {
    public:
        Structure_Point(const Vec3f &coord, const Vec3i &col) { }
        
        const Vec3f &coords() const;
        Vec3f &coords();
        const Vector3i &color() const;
        
    protected:
        Vec3f p; // The x,y,z coordinates
        Vec3i c; //!< The r,g,b color components (0-255 for each)
        
    };
    
    inline const Vec3f &Structure_Point::coords() const {
        return p;
    }
    
    inline Vec3f &Structure_Point::coords() {
        return p;
    }
    
    inline const Vec3i &Structure_Point::color() const {
        return c;
    }
}

#endif // structure_point_h_
