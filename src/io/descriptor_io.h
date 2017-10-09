#ifndef descriptor_io_h_
#define descriptor_io_h_

#include "math/numeric.h"

namespace open3DCV
{
    int read_descriptors(const std::string fname, std::vector<Vecf>& descriptors);
    int write_descriptors(const std::string fname, const std::vector<Vecf>& descriptors);
}

#endif
