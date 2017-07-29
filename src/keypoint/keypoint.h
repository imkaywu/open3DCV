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
    enum KeypointType
    {
        INVALID = -1,
        DOG = 0,
        HARRIS = 1,
        SIFT = 2,
        SURF = 3,
    };
    
    class Keypoint {
    public:
        Keypoint(const Vec2 &r_x, KeypointType &r_t);
        Keypoint(const Vec2 &r_x, KeypointType &r_t, unsigned int r_i);
        Keypoint(const Vec2 &r_x, KeypointType &r_t, unsigned int r_i, const Vec3i &r_c);
        virtual ~Keypoint() { };
        
        const Vec2 &coords() const;
        void coords(const Vec2 r_coords);
        const unsigned int index() const;
        void index(const unsigned int r_i);
        const Vec3i &color() const;
        void color(const Vec3i r_c);
        const double scale() const;
        void scale(const double r_s);
        const double orientation() const;
        void orientation(const double r_o);

    private:
        Vec2 coords_;                   // coordinates
        unsigned int i_;                // Image index
        Vec3i c_;                       // color
        KeypointType keypoint_type_;
        double scale_;
        double orientation_;
        
    };

    inline const Vec2& Keypoint::coords() const {
        return coords_;
    }
    
    inline void Keypoint::coords(const Vec2 r_coords) {
        coords_ = r_coords;
    }

    inline const unsigned int Keypoint::index() const {
        return i_;
    }
    
    inline void Keypoint::index(const unsigned int r_i) {
        i_ = r_i;
    }

    inline const Vec3i& Keypoint::color() const {
        return c_;
    }
    
    inline void Keypoint::color(const Vec3i r_c) {
        c_ = r_c;
    }
    
    inline const double Keypoint::scale() const {
        return scale_;
    }
    
    inline void Keypoint::scale(const double r_s) {
        scale_ = r_s;
    }
    
    inline const double Keypoint::orientation() const {
        return orientation_;
    }
    
    inline void Keypoint::orientation(const double r_o) {
        orientation_ = r_o;
    }
}
#endif // keypoint_h
