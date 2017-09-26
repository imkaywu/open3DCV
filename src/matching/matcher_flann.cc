#include "matching/distance.h"
#include "matching/matcher_flann.h"
#include "flann/nanoflann.h"

using std::vector;

namespace open3DCV
{
    Matcher_Flann::Matcher_Flann(Matcher_Param r_matcher_param)
    {
        matcher_param_ = r_matcher_param;
    }
    
    void Matcher_Flann::init_param(Matcher_Param r_matcher_param)
    {
        matcher_param_ = r_matcher_param;
    }
    
    int Matcher_Flann::match(const vector<Vecf>& desc1, const vector<Vecf>& desc2, vector<DMatch>& matches)
    {
        float ratio = matcher_param_.ratio;
        const int nmatches_min = matcher_param_.nmatches_min;
        
        // construct two kd-tree indexes
        int leaf_max_size = matcher_param_.leaf_max_size;
        kd_tree_t desc_index_1(matcher_param_.ndims, desc1, leaf_max_size /* max leaf */);
        desc_index_1.index->buildIndex();
        kd_tree_t desc_index_2(matcher_param_.ndims, desc2, leaf_max_size /* max leaf */);
        desc_index_2.index->buildIndex();
        
        match_bidirect(desc1, desc2, desc_index_1, desc_index_2, ratio, matches);
        
        if (matches.size() < nmatches_min)
        {
            ratio = 0.7 * 0.7;
            match_bidirect(desc1, desc2, desc_index_1, desc_index_2, ratio, matches);
            if (matches.size() < nmatches_min)
            {
                ratio = 0.8 * 0.8;
                match_bidirect(desc1, desc2, desc_index_1, desc_index_2, ratio, matches);
                if (matches.size() < nmatches_min)
                {
                    ratio = 0.9 * 0.9;
                    match_bidirect(desc1, desc2, desc_index_1, desc_index_2, ratio, matches);
                    if (matches.size() < nmatches_min)
                    {
                        ratio = 0.95 * 0.95;
                        match_bidirect(desc1, desc2, desc_index_1, desc_index_2, ratio, matches);
                    }
                }
            }
        }
        
        return 0;
    }
    
    int Matcher_Flann::match_bidirect(const vector<Vecf>& desc1, const vector<Vecf>& desc2, const kd_tree_t& desc_index_1, const kd_tree_t& desc_index_2, const float ratio, std::vector<DMatch>& matches)
    {
        // do knn searches
        int nresults = matcher_param_.nresults;
        vector<size_t> ret_indexes(nresults);
        vector<float> dists(nresults);
        
        nanoflann::KNNResultSet<float> result_set(nresults);
        for (int i = 0; i < desc1.size(); ++i)
        {
            result_set.init(&ret_indexes[0], &dists[0]); // has to be inside loop, there is a 'dist' variable that needs to be set as MAX
            desc_index_2.index->findNeighbors(result_set, &desc1[i](0), nanoflann::SearchParams(10));
            if (dists[0] < ratio * dists[1])
            {
                size_t idx = ret_indexes[0];
                result_set.init(&ret_indexes[0], &dists[0]);
                desc_index_1.index->findNeighbors(result_set, &desc2[idx](0), nanoflann::SearchParams(10));
                if (ret_indexes[0] == i && dists[0] < ratio * dists[1])
                {
                    DMatch match(i, static_cast<int>(idx), dists[0]);
                    matches.push_back(match);
                }
            }
        }
        
        return 0;
    }
    
    int Matcher_Flann::match(const vector<Vecf>& desc1, const vector<Vecf>& desc2, vector<DMatch>& matches, float (*dist_metric)(const Vecf& desc1, const Vecf& desc2))
    {
        // use the one with larger keypoints to construct kd-tree
        size_t nkeys1 = desc1.size(), nkeys2 = desc2.size();
        const vector<Vecf>& desc_source = nkeys1 < nkeys2 ? desc1 : desc2;
        const vector<Vecf>& desc_target = nkeys1 < nkeys2 ? desc2 : desc1;
        Matching_Dirc matching_dirc = nkeys1 < nkeys2 ? FORWARD : BACKWARD;
        
        // construct a kd-tree index
        typedef KDTreeVectorOfVectorsAdaptor<vector<Vecf>, float>  kd_tree_t;
        
        int leaf_max_size = matcher_param_.leaf_max_size;
        kd_tree_t desc_index(matcher_param_.ndims, desc_target, leaf_max_size /* max leaf */);
        desc_index.index->buildIndex();
        
        // do knn searches
        int nresults = matcher_param_.nresults;
        vector<size_t> ret_indexes(nresults);
        vector<float> dists(nresults);
        
        nanoflann::KNNResultSet<float> result_set(nresults);
        
        for (int i = 0; i < desc_source.size(); ++i)
        {
            result_set.init(&ret_indexes[0], &dists[0]); // has to be inside loop, there is a 'dist' variable that needs to be set as MAX
            desc_index.index->findNeighbors(result_set, desc_source[i].data(), nanoflann::SearchParams(10));
            if (dists[0] < matcher_param_.ratio * dists[1])
            {
                if (matching_dirc == FORWARD)
                {
                    DMatch match(i, static_cast<int>(ret_indexes[0]), dists[0]);
                    matches.push_back(match);
                }
                else
                {
                    DMatch match(static_cast<int>(ret_indexes[0]), i, dists[0]);
                    matches.push_back(match);
                }
            }
        }
        
        return 0;
    }
}
