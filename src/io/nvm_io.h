#ifndef nvm_io_h_
#define nvm_io_h_

#include "sfm/graph.h"

namespace open3DCV
{
    void read_nvm(Graph& graph);
    void write_nvm(const Graph& graph);
}

#endif
