#ifndef estimate_rt_from_e_h_
#define estimate_rt_from_e_h_

#include "math/numeric.h"
#include "sfm/graph.h"

namespace open3DCV
{
    void Rt_from_E(const Mat3f& E, std::vector<Mat3f>& R, std::vector<Vec3f>& t);
    void Rt_from_E(const Mat3f& E, Mat3f& R, Vec3f& t);
    void Rt_from_E(Graph& graph);
    
    template <typename T>
    std::vector<size_t> sort_indexes(const std::vector<T> &v) {
        
        // initialize original index locations
        std::vector<size_t> idx(v.size());
        iota(idx.begin(), idx.end(), 0);
        
        // sort indexes based on comparing values in v
        sort(idx.begin(), idx.end(),
             [&v](size_t i1, size_t i2) {return v[i1] > v[i2];}); // sort in descending order
        
        return idx;
    }
}

#endif
