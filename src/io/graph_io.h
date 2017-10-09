#ifndef graph_io_h
#define graph_io_h

#include "sfm/graph.h"

namespace open3DCV
{
    int read_graph(const std::string fname, Graph& graph);
    int write_graph(const std::string fname, const Graph& graph);
}

#endif
