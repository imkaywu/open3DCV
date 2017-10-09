#ifndef sfm_io_h_
#define sfm_io_h_

#include "sfm/graph.h"

namespace open3DCV
{
    int write_sfm(const std::string dir, const Graph& graph);
}

#endif
