#ifndef match_io_h_
#define match_io_h_

#include <vector>
#include <string>
#include "matching/match.h"

namespace open3DCV
{
    int read_matches(std::vector<Match>& matches, const std::string fname);
    int write_matches(const std::vector<Match>& matches, const std::string fname);
}

#endif
