#include "keypoint.h"

namespace open3DCV {

    Keypoint::Keypoint(const Vec2& r_x, unsigned int r_i) : m_p(r_x), m_i(r_i), m_c(0,0,0)
    {
      //no-op
    }

    Keypoint::Keypoint(const Vec2& r_x, unsigned int r_i, const Vec3i& r_c) : m_p(r_x), m_i(r_i), m_c(r_c)
    {
      //no-op
    }
}

