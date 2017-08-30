#ifndef matcher_param_h_
#define matcher_param_h_

namespace open3DCV
{
    struct Matcher_Param
    {
        // all matchers
        float ratio = 0.8;
        
        // Flann matcher
        int ndims = 128;
        int leaf_max_size = 10;
        int nresults = 3;
        
        Matcher_Param() { };
        Matcher_Param(const float r_ratio) : ratio(r_ratio) { };
        Matcher_Param(const float r_ratio, const int r_ndims, const int r_leaf_max_size, const int r_nresults) :
                      ratio(r_ratio), ndims(r_ndims), leaf_max_size(r_leaf_max_size), nresults(r_nresults) { };
        
    };
}

#endif
