#ifndef matcher_h_
#define matcher_h_

#include "keypoint/keypoint.h"
#include "keypoint/descriptor.h"
#include "matching/matcher_param.h"
#include "matching/match.h"
#include "flann/nanoflann.h"

namespace open3DCV
{
enum Match_Type {BF = 0, FLANN};
    
class Matcher
{
public:
    Matcher();
    Matcher(const Matcher_Param r_matcher_param);
    virtual ~Matcher();
    
    virtual void init_param(Matcher_Param r_type) = 0;
    virtual int match(const vector<Vecf>& desc1, const vector<Vecf>& desc2, vector<Match>& matches) = 0;
    
private:
    Matcher_Param matcher_param_;
};
    
} // namespace open3DCV

#endif
