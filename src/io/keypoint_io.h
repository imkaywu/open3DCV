#ifndef keypoint_io_h_
#define keypoint_io_h_

#include "keypoint/keypoint.h"

namespace open3DCV
{
    int read_keypoints(const std::string fname, std::vector<Keypoint>& keypoints);
    int read_keypoints(const std::string fname, std::vector<Vec2f>& keys);
    
    int write_keypoints(const std::string fname, const std::vector<Keypoint>& keypoints);
}

#endif
