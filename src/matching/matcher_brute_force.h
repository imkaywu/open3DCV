#ifndef matcher_brute_force_h_
#define matcher_brute_force_h_

#include "matching/matcher.h"

namespace open3DCV
{
    
    class Matcher_Brute_Force : public Matcher
    {
    public:
        Matcher_Brute_Force() { };
        Matcher_Brute_Force(Matcher_Param r_matcher_param);
        virtual ~Matcher_Brute_Force() { };
        
        void init_param(Matcher_Param r_matcher_param);
        int match(const std::vector<Vecf>& desc1, const std::vector<Vecf>& desc2, std::vector<DMatch>& matches);
        int match(const std::vector<Vecf>& desc1, const std::vector<Vecf>& desc2, std::vector<DMatch>& matches, float (*dist_metric)(const Vecf& desc1, const Vecf& desc2));
    };
    
} // namespace open3DCV

#endif
