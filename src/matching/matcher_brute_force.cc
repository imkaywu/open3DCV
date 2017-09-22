#include "matching/matcher_brute_force.h"
#include "matching/distance.h"

using std::vector;

namespace open3DCV
{
    Matcher_Brute_Force::Matcher_Brute_Force (Matcher_Param r_matcher_param)
    {
        matcher_param_ = r_matcher_param;
    }
    
    void Matcher_Brute_Force::init_param(Matcher_Param r_matcher_param)
    {
        matcher_param_ = r_matcher_param;
    }
    
    int Matcher_Brute_Force::match(const vector<Vecf>& desc1, const vector<Vecf>& desc2, vector<DMatch>& matches)
    {
        return match(desc1, desc2, matches, l2_dist);
    }
    
    int Matcher_Brute_Force::match(const vector<Vecf>& desc1, const vector<Vecf>& desc2, vector<DMatch>& matches, float (*dist_metric)(const Vecf& desc1, const Vecf& desc2))
    {
        float dist = 0, min_dist = 1e8, sec_min_dist = 1e8, ratio = matcher_param_.ratio;
        int ind_min_key = 0;
        
        for (int i = 0; i < desc1.size(); ++i)
        {
            min_dist = sec_min_dist = 1e8;
            for (int j = 0; j < desc2.size(); ++j)
            {
                dist = dist_metric(desc1[i], desc2[j]);
                if (dist < min_dist)
                {
                    sec_min_dist = min_dist;
                    min_dist = dist;
                    ind_min_key = j;
                }
                else if (dist < sec_min_dist)
                {
                    sec_min_dist = dist;
                }
            }
            if (min_dist < ratio * sec_min_dist)
            {
                DMatch m(i, ind_min_key, min_dist);
                matches.push_back(m);
            }
        }
        
        return 0;
    }

}
