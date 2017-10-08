#ifndef sfm_io_h_
#define sfm_io_h_

#include "sfm/graph.h"

namespace open3DCV
{
    void read_sfm(const std::string fname, Graph& graph);
    void write_sfm(const Graph& graph);
}

#endif
