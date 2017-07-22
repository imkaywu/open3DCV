#include "keypoint.h"

namespace SfMLibrary {
  namespace Core {

    Keypoint::Keypoint(const Vector2d &x, unsigned int i_) : p(x), i(i_),
      c(0,0,0)
    {
      //no-op
    }

    Keypoint::Keypoint(const Vector2d &x, unsigned int i_, const Vector3i &c_) :
      p(x), i(i_), c(c_)
    {
      //no-op
    }

  }
}

