#include "graph.h"

namespace open3DCV
{
    Graph::Graph()
    {
        // no op
    }
    
    Graph::Graph(const int& nframes, const std::vector<int>& iframes, const std::vector<Keypoint>& keys, const std::vector<Match>& matches) :
                 nframes_(nframes), iframes_(iframes), keys_(keys), matches_(matches)
    {
        // no op
    }
}
