#ifndef matcher_flann_h_
#define matcher_flann_h_

#include "keypoint/descriptor.h"
#include "matching/matcher.h"
#include "matching/KDTreeVectorOfVectorAdaptor.h"
#include "flann/nanoflann.h"

using std::vector;

namespace open3DCV
{
    enum Matching_Dirc {FORWARD = 0, BACKWARD};
   
    class Matcher_Flann : public Matcher
    {
    public:
        Matcher_Flann() { };
        Matcher_Flann(Matcher_Param r_matcher_param);
        virtual ~Matcher_Flann() { };
        
        void init_param(Matcher_Param r_matcher_param);
        int match(const vector<Vecf>& desc1, const vector<Vecf>& desc2, vector<Match>& matches);
    };
    
    inline Matcher_Flann::Matcher_Flann(Matcher_Param r_matcher_param)
    {
        matcher_param_ = r_matcher_param;
    }
    
    inline void Matcher_Flann::init_param(Matcher_Param r_matcher_param)
    {
        matcher_param_ = r_matcher_param;
    }
    
    // check out the nanaflann's example code: vector_of_vectors_example
    inline int Matcher_Flann::match(const vector<Vecf>& desc1, const vector<Vecf>& desc2, vector<Match>& matches)
    {
        // construct a kd-tree index
        typedef KDTreeVectorOfVectorsAdaptor<vector<Vecf>, float>  kd_tree_t;
        
        // use the one with larger keypoints to construct kd-tree
        size_t nkeys1 = desc1.size(), nkeys2 = desc2.size();
        const vector<Vecf>& desc_source = nkeys1 < nkeys2 ? desc1 : desc2;
        const vector<Vecf>& desc_target = nkeys1 < nkeys2 ? desc2 : desc1;
        Matching_Dirc matching_dirc = nkeys1 < nkeys2 ? FORWARD : BACKWARD;
//        vector<Vecf> desc_source = desc1;
//        vector<Vecf> desc_target = desc2;
//        Matching_Dirc matching_dirc = FORWARD;
        
        int leaf_max_size = matcher_param_.leaf_max_size;
        kd_tree_t desc_index(matcher_param_.ndims, desc_target, leaf_max_size /* max leaf */);
        desc_index.index->buildIndex();
        
        // do knn searches
        int nresults = matcher_param_.nresults;
        vector<size_t> indexes(nresults);
        vector<float> dists(nresults);
        
        nanoflann::KNNResultSet<float, size_t, size_t> result_set(nresults);
        result_set.init(&indexes[0], &dists[0]);
        
        for (int i = 9; i < desc_source.size(); ++i)
        {
            desc_index.index->findNeighbors(result_set, desc_source[i].data(), nanoflann::SearchParams());
            if (dists[0] < matcher_param_.ratio * dists[1])
            {
                if (matching_dirc == FORWARD)
                {
                    Match match(i, indexes[0], dists[0]);
                    matches.push_back(match);
                }
                else
                {
                    Match match(indexes[0], i, dists[0]);
                    matches.push_back(match);
                }
//                indexes[0] = indexes[1] = indexes[2] = 0;
//                dists[0] = dists[1] = dists[2] = 0.0f;
            }
        }
        
        return 0;
    }
    
} // namespace open3DCV

#endif
