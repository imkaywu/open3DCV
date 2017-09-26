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
    
    inline void rodrigues(Mat3f& r_R, Matf* r_dR, const Vec3f& r_om)
    {
        float *R_pt = nullptr, *dR_pt = nullptr, *om_pt = nullptr;
        R_pt = new float[9];
        om_pt = new float[3];
        
#define R(i,j)  R_pt[(i)+3*(j)]
#define DR(i,j) dR_pt[(i)+9*(j)]
#define OM(i) om_pt[(i)]
        
        if (r_dR)
        {
            r_dR->resize(9, 3);
            dR_pt = new float[27];
        }
        for (int i = 0; i < 3; ++i)
            OM(i) = r_om(i);
        
        rodrigues<float>(R_pt, dR_pt, om_pt);
        
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                r_R(i,j) = R(i, j);
        if (r_dR)
        {
            for (int i = 0; i < 9; ++i)
                for (int j = 0; j < 3; ++j)
                    (*r_dR)(i, j) = DR(i, j);
        }
#undef R
#undef DR
#undef OM
        
        delete[] R_pt;
        delete[] dR_pt;
        delete[] om_pt;
    }
    
    inline void irodrigues(Vec3f& r_om, Matf* r_dom, const Mat3f& r_R)
    {
        float *om_pt = nullptr, *dom_pt = nullptr, *R_pt = nullptr;
        om_pt = new float[3];
        R_pt = new float[9];
        
#define OM(i) om_pt[(i)]
#define DOM(i,j) dom_pt[(i)+3*(j)]
#define R(i,j)  R_pt[(i)+3*(j)]
        
        if (r_dom)
        {
            r_dom->resize(3, 9);
            dom_pt = new float[27];
        }
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                R(i, j) = r_R(i, j);
        
        irodrigues<float>(om_pt, dom_pt, R_pt);
        
        for (int i = 0; i < 3; ++i)
            r_om(i) = OM(i);
        if (r_dom)
        {
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 9; ++j)
                    (*r_dom)(i, j) = DOM(i, j);
        }
        
#undef OM
#undef DOM
#undef R
        
        delete [] om_pt;
        delete [] dom_pt;
        delete [] R_pt;
    }
}

#endif
