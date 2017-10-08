#ifndef keypoint_h
#define keypoint_h

#include "iostream"
#include "math/numeric.h"

namespace open3DCV {
    
    #define open3DCV_KEYPOINT_VAR -9999
    
    enum KeypointType
    {
        INVALID = -1,
        DOG = 0,
        HARRIS = 1,
        SIFT = 2,
        SURF = 3,
    };
    
    // The only member that must be set is coords_
    // for SfM, index_ must be set, id_ is optional
    class Keypoint {
    public:
        Keypoint();
        Keypoint(const Vec2f &r_x);
        Keypoint(const Vec2f &r_x, unsigned int r_i);
        Keypoint(const Vec2f &r_x, unsigned int r_i, const Vec3i &r_c);
        Keypoint(const Vec2f &r_x, const float r_s, const float r_o);
        Keypoint(const Keypoint& key);
        Keypoint& operator=(const Keypoint& key);
        virtual ~Keypoint() { };
        
        const Vec2f &coords() const;
        Vec2f& coords();
        const unsigned int& index() const;
        unsigned int& index();
        const Vec3i& color() const;
        Vec3i& color();
        const double& scale() const;
        double& scale();
        const int has_scale() const;
        const double& orientation() const;
        double& orientation();
        const int has_orientation() const;
        
        bool operator<(const Keypoint& rhs) const;
        friend std::ostream& operator<<(std::ostream& ostr, const Keypoint& rhs);
        friend std::istream& operator>>(std::istream& istr, Keypoint& rhs);
        
        static int is_identical(const Keypoint& key1, const Keypoint& key2);

    private:
        // The difference between the implemented Keypoint and VLFeat is that the latter assumes that
        // the image origin (top-left corner) has coordinate (0,0) as opposed to (1,1)
        Vec2f coords_;                      // coordinates
        unsigned int index_;                // Image index
        Vec3i color_;                       // color
        double scale_;                      // scale
        double orientation_;                // orientation
        std::pair<int, int> id_;            // id of the feature, <img_ind, feat_ind>, not used
        KeypointType keypoint_type_;        // keypoint type, not used
        
    };
}
#endif // keypoint_h
