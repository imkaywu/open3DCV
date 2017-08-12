#ifndef matcher_h_
#define matcher_h_

#include "keypoint/keypoint.h"
#include "keypoint/descriptor.h"
#include "matching/match.h"

namespace open3DCV
{
class Matcher
{
public:
    Matcher() { };
    virtual ~Matcher() { };
    
    virtual int match(const vector<Vec>& desc1, const vector<Vec>& desc2, vector<Match>& matches);
};
    
} // namespace open3DCV

#endif
