#ifndef matcher_h_
#define matcher_h_

#include <fstream>
#include "keypoint/keypoint.h"
#include "keypoint/descriptor.h"
#include "matching/matcher_param.h"
#include "matching/match.h"
#include "flann/nanoflann.h"

namespace open3DCV
{
    
    class Matcher
    {
    public:
        Matcher() { };
        Matcher(Matcher_Param r_matcher_param);
        virtual ~Matcher() { };
        
        virtual void init_param(Matcher_Param r_matcher_param) = 0;
        virtual int match(const vector<Vecf>& desc1, const vector<Vecf>& desc2, vector<Match>& matches) = 0;
        virtual int match(const vector<Vecf>& desc1, const vector<Vecf>& desc2, vector<Match>& matches, float (*dist_metric)(const Vecf& desc1, const Vecf& desc2)) = 0;
        
    protected:
        Matcher_Param matcher_param_;
    };
    
} // namespace open3DCV

#endif
