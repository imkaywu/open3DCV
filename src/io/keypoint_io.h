#ifndef keypoint_io_h_
#define keypoint_io_h_

#include "keypoint/keypoint.h"

namespace open3DCV
{
    int read_keypoints(std::vector<Keypoint>& keypoints, const std::string fname);
    int write_keypoints(const std::vector<Keypoint>& keypoints, const std::string fname);
}

#endif
