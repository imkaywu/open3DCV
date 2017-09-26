#ifndef matcher_flann_h_
#define matcher_flann_h_

#include "matching/matcher.h"
#include "utils/KDTreeVectorOfVectorAdaptor.h"

namespace open3DCV
{
    enum Matching_Dirc {FORWARD = 0, BACKWARD};
    
    typedef KDTreeVectorOfVectorsAdaptor<std::vector<Vecf>, float>  kd_tree_t;
   
    class Matcher_Flann : public Matcher
    {
    public:
        Matcher_Flann() { };
        Matcher_Flann(Matcher_Param r_matcher_param);
        virtual ~Matcher_Flann() { };
        
        void init_param(Matcher_Param r_matcher_param);
        int match(const std::vector<Vecf>& desc1, const std::vector<Vecf>& desc2, std::vector<DMatch>& matches);
        int match_bidirect(const std::vector<Vecf>& desc1, const std::vector<Vecf>& desc2, const kd_tree_t& kd_tree_1, const kd_tree_t& kd_tree_2, const float ratio, std::vector<DMatch>& matches);
        int match(const std::vector<Vecf>& desc1, const std::vector<Vecf>& desc2, std::vector<DMatch>& matches, float (*dist_metric)(const Vecf& desc1, const Vecf& desc2));
    };
    
} // namespace open3DCV

#endif
