#ifndef keypoint_h
#define keypoint_h

#include "math/numeric.h"

namespace open3DCV {
    //! Keypoint class
    /*!
     * This class encapsulates the data associated with a 2D image keypoint.
     * This class stores the x and y coordinates of the image location, along
     * with the index (i) of the image (camera) it is found, and color value
     * (stored in a 0-255 format).
     */
    # define open3DCV_KEYPOINT_VAR -9999
    enum KeypointType
    {
        INVALID = -1,
        DOG = 0,
        HARRIS = 1,
        SIFT = 2,
        SURF = 3,
    };
    
    // The only two members that must be set are coords_, and keypoint_type_
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
        const int has_scale() const;
        const double orientation() const;
        void orientation(const double r_o);
        const int has_orientation() const;

    private:
        // The difference between the implemented Keypoint and VLFeat is that the latter assumes that
        // the image origin (top-left corner) has coordinate (0,0) as opposed to (1,1)
        Vec2 coords_;                       // coordinates
        unsigned int index_;                // Image index
        Vec3i color_;                       // color
        KeypointType keypoint_type_;
        double scale_;
        double orientation_;
        
    };
    
    inline Keypoint::Keypoint(const Vec2 &r_coords, KeypointType &r_t) : coords_(r_coords), keypoint_type_(r_t), scale_(open3DCV_KEYPOINT_VAR), orientation_(open3DCV_KEYPOINT_VAR), index_(open3DCV_KEYPOINT_VAR), color_(0,0,0)
    {
        //no-op
    }
    
    inline Keypoint::Keypoint(const Vec2& r_coords, KeypointType &r_t, unsigned int r_i) : coords_(r_coords), keypoint_type_(r_t), scale_(open3DCV_KEYPOINT_VAR), orientation_(open3DCV_KEYPOINT_VAR), index_(r_i), color_(0,0,0)
    {
        //no-op
    }
    
    inline Keypoint::Keypoint(const Vec2& r_coords, KeypointType &r_t, unsigned int r_i, const Vec3i& r_c) : coords_(r_coords), keypoint_type_(r_t), scale_(open3DCV_KEYPOINT_VAR), orientation_(open3DCV_KEYPOINT_VAR), index_(r_i), color_(r_c)
    {
        //no-op
    }

    inline const Vec2& Keypoint::coords() const {
        return coords_;
    }
    
    inline void Keypoint::coords(const Vec2 r_coords) {
        coords_ = r_coords;
    }

    inline const unsigned int Keypoint::index() const {
        return index_;
    }
    
    inline void Keypoint::index(const unsigned int r_i) {
        index_ = r_i;
    }

    inline const Vec3i& Keypoint::color() const {
        return color_;
    }
    
    inline void Keypoint::color(const Vec3i r_c) {
        color_ = r_c;
    }
    
    inline const double Keypoint::scale() const {
        return scale_;
    }
    
    inline void Keypoint::scale(const double r_s) {
        scale_ = r_s;
    }
    
    inline const int Keypoint::has_scale() const {
        return scale_ != open3DCV_KEYPOINT_VAR;
    }
    
    inline const double Keypoint::orientation() const {
        return orientation_;
    }
    
    inline void Keypoint::orientation(const double r_o) {
        orientation_ = r_o;
    }
    
    inline const int Keypoint::has_orientation() const {
        return orientation_ != open3DCV_KEYPOINT_VAR;
    }
}
#endif // keypoint_h
