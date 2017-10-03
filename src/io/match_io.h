#ifndef match_io_h_
#define match_io_h_

#include <vector>
#include <string>
#include "keypoint/keypoint.h"
#include "matching/dmatch.h"

namespace open3DCV
{
    int read_matches(const std::string fname, std::vector<DMatch>& matches);
    int read_matches(const std::string fname, const std::vector<Keypoint>& keys1, const std::vector<Keypoint>& keys2, std::vector<DMatch>& matches);
    int read_matches(const std::string fname, std::vector<std::pair<Vec2f, Vec2f> >& matches);
    int write_matches(const std::string fname, const std::vector<DMatch>& matches);
    int write_matches(const std::string fname, const std::vector<Keypoint>& keys1, const std::vector<Keypoint>& keys2, const std::vector<DMatch>& matches);
    int write_matches(const std::string fname, const std::vector<std::pair<Vec2f, Vec2f> >& matches);
}

#endif
