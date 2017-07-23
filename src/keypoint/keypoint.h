#ifndef keypoint_h
#define keypoint_h

#include "numeric.h"

namespace open3DCV {

    //! Keypoint class
    /*!
     * This class encapsulates the data associated with a 2D image keypoint.
     * This class stores the x and y coordinates of the image location, along
     * with the index (i) of the image (camera) it is found, and color value
     * (stored in a 0-255 format).
     */
    class Keypoint {
    public:
       
        Keypoint(const Vec2 &x, unsigned int i);
        Keypoint(const Vec2 &x, unsigned int i, const Vec3i &c);
        
        //virtual ~Keypoint();
        
        const Vec2 &coords() const;
        unsigned int index() const;
        const Vec3i &color() const;

    protected:
        Vec2 p;     // Image coordinates
        unsigned int i; // Image index
        Vec3i c;     // Image color
    };

    inline const Vec2 &Keypoint::coords() const {
        return p;
    }

    inline unsigned int Keypoint::index() const {
        return i;
    }

    inline const Vec3i &Keypoint::color() const {
        return c;
    }
}
#endif // keypoint_h
