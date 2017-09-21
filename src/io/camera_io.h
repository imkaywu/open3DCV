#ifndef camera_io_h_
#define camera_io_h_

namespace open3DCV
{
    void read_camera(const std::string file, const int param_type);
    void write_camera(const std::string file, const int param_type);
}

#endif
