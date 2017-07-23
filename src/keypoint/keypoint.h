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
       
        Keypoint(const Vec2 &r_x, unsigned int r_i);
        Keypoint(const Vec2 &r_x, unsigned int r_i, const Vec3i &r_c);
        
        //virtual ~Keypoint();
        
        const Vec2 &coords() const;
        unsigned int index() const;
        const Vec3i &color() const;

    protected:
        Vec2 m_p;     // Image coordinates
        unsigned int m_i; // Image index
        Vec3i m_c;     // Image color
    };

    inline const Vec2 &Keypoint::coords() const {
        return m_p;
    }

    inline unsigned int Keypoint::index() const {
        return m_i;
    }

    inline const Vec3i &Keypoint::color() const {
        return m_c;
    }
}
#endif // keypoint_h
