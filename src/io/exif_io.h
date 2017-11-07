#ifndef exif_io_h_
#define exif_io_h_

#include <iostream>

namespace open3DCV
{
    int read_exif(const std::string fname, float focal_length);
}

#endif
