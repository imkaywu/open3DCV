#ifndef distance_h_
#define distance_h_

#include "math/numeric.h"

namespace open3DCV
{
inline float l2_dist(const Vec& desc1, const Vec& desc2)
{
//    return (desc1 - desc2).squaredNorm();
    return (desc1 - desc2).norm();
};
    
inline float l1_dist(const Vec& desc1, const Vec& desc2)
{
    return (desc1 - desc2).lpNorm<1>();
}
    
inline float ang_dist(const Descriptor& desc1, const Descriptor& desc2)
{
    return 0;
}
}

#endif
