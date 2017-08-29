#ifndef matcher_flann_h_
#define matcher_flann_h_

#include "keypoint/descriptor.h"
#include "matching/matcher.h"

namespace open3DCV
{
    enum Match_Type {BF = 0, FLANN};
    
    class Matcher_Flann : Match
    {
    public:
        Matcher_Flann() { };
        Matcher_Flann(const Matcher_Param r_matcher_param) : matcher_param_(r_matcher_param) { }
        virtual ~Matcher_Flann() { };
        
        void init_param(Matcher_Param r_matcher_param) : matcher_params_(r_matcher_param) { };
        int match(const vector<Vecf>& desc1, const vector<Vecf>& desc2, vector<Match>& matches);
    };
    
    inline Matcher_Flann::match(const vector<Vecf>& desc1, const vector<Vecf>& desc2, vector<Match>& matches)
    {
        
        return 0;
    }
    
} // namespace open3DCV

#endif
