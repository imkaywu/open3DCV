#include "keypoint.h"

namespace open3DCV {


    Keypoint::Keypoint(const Vec2 &x, unsigned int i_) : p(x), i(i_),
      c(0,0,0)
    {
      //no-op
    }

    Keypoint::Keypoint(const Vec2 &x, unsigned int i_, const Vec3i &c_) :
      p(x), i(i_), c(c_)
    {
      //no-op
    }


}

