#ifndef rodrigues_h_
#define rodrigues_h_

#include "math/numeric.h"

namespace open3DCV
{
    // modified from 'rodrigues.h' and 'rodrigues.c' from VLFEAT
    template<typename T>
    void rodrigues(T* R, T* dR, const T* om);
    template<typename T>
    void irodrigues(T* om, T* dR, const T* R);
    
    inline void rodrigues(Mat3f& R, Matf* dR, const Vec3f& om)
    {
        rodrigues<float>(R.data(), dR->data(), om.data());
    }
    
    inline void irodrigues(Vec3f& om, Matf* dom, const Mat3f& R)
    {
        irodrigues<float>(om.data(), dom->data(), R.data());
    }
}

#endif
