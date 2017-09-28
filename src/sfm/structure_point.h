#ifndef structure_point_
#define structure_point_

#include "math/numeric.h"

namespace open3DCV
{
    class Structure_Point
    {
    public:
        Structure_Point();
        Structure_Point(const Vec3f &coord);
        Structure_Point(const Vec3f &coord, const Vec3i &color);
        Structure_Point& operator=(const Structure_Point& struct_pt);
        
        const Vec3f &coords() const;
        Vec3f &coords();
        const Vec3i &color() const;
        Vec3i &color();
        
    protected:
        Vec3f p_; // The x,y,z coordinates, better is it's of double
        Vec3i c_; //!< The r,g,b color components (0-255 for each)
        
    };
    
}

#endif // structure_point_h_
