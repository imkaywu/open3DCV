#ifndef matcher_param_h_
#define matcher_param_h_

namespace open3DCV
{
    struct Matcher_Param
    {
        // all matchers
        float ratio = 0.6 * 0.6;
        int nmatches_min = 50;
        
        // Flann matcher
        int ndims = 128;
        int leaf_max_size = 10;
        int nresults = 2;
        
        Matcher_Param() { };
        Matcher_Param(const float r_ratio, const int r_nmatches) : ratio(r_ratio), nmatches_min(r_nmatches) { };
        Matcher_Param(const float r_ratio, const int r_nmatches, const int r_ndims, const int r_leaf_max_size, const int r_nresults) :
                      ratio(r_ratio), nmatches_min(r_nmatches), ndims(r_ndims), leaf_max_size(r_leaf_max_size), nresults(r_nresults) { };
        
    };
}

#endif
