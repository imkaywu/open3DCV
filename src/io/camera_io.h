#ifndef camera_io_h_
#define camera_io_h_

#include "camera/camera.h"

namespace open3DCV
{
    int read_camera(const std::string fname, Camera& camera);
    int write_camera(const std::string fname, const Camera& camera);
}

#endif
