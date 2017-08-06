#include "keypoint.h"

namespace open3DCV {
    
    Keypoint::Keypoint(const Vec2 &r_coords, KeypointType &r_t) : coords_(r_coords), keypoint_type_(r_t)
    {
        
    }

    Keypoint::Keypoint(const Vec2& r_coords, KeypointType &r_t, unsigned int r_i) : coords_(r_coords), keypoint_type_(r_t), index_(r_i), color_(0,0,0)
    {
      //no-op
    }

    Keypoint::Keypoint(const Vec2& r_coords, KeypointType &r_t, unsigned int r_i, const Vec3i& r_c) : coords_(r_coords), keypoint_type_(r_t), index_(r_i), color_(r_c)
    {
      //no-op
    }
}

