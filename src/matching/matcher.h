#ifndef matcher_h_
#define matcher_h_

#include <fstream>
#include "math/numeric.h"
#include "keypoint/keypoint.h"
#include "matching/matcher_param.h"
#include "matching/dmatch.h"

namespace open3DCV
{
    
    class Matcher
    {
    public:
        Matcher() { };
        Matcher(Matcher_Param r_matcher_param);
        virtual ~Matcher() { };
        
        virtual void init_param(Matcher_Param r_matcher_param) = 0;
        virtual int match(const std::vector<Vecf>& desc1, const std::vector<Vecf>& desc2, std::vector<DMatch>& matches) = 0;
        virtual int match(const std::vector<Vecf>& desc1, const std::vector<Vecf>& desc2, std::vector<DMatch>& matches, float (*dist_metric)(const Vecf& desc1, const Vecf& desc2)) = 0;
        void matching_keys(const std::vector<Keypoint>& keys1, const std::vector<Keypoint>& keys2, std::vector<DMatch>& matches);
        
    protected:
        Matcher_Param matcher_param_;
    };
    
    inline void Matcher::matching_keys(const std::vector<Keypoint>& keys1, const std::vector<Keypoint>& keys2, std::vector<DMatch>& matches)
    {
        for (int i = 0; i < matches.size(); ++i)
        {
            DMatch& match = matches[i];
            std::pair<Vec2f, Vec2f>& match_pair = match.point_;
            const int ikey1 = match.ind_key_.first;
            const int ikey2 = match.ind_key_.second;
            match_pair.first = keys1[ikey1].coords();
            match_pair.second = keys2[ikey2].coords();
        }
    }
    
} // namespace open3DCV

#endif
