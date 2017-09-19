#ifndef descriptor_io_h_
#define descriptor_io_h_

#include "math/numeric.h"

namespace open3DCV
{
    int read_descriptors(std::vector<Vecf>& descriptors, const std::string fname);
    int write_descriptors(const std::vector<Vecf>& descriptors, const std::string fname);
}

#endif
