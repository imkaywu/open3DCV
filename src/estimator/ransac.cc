#include <set>
#include <vector>
#include <limits>
#include "estimator/ransac.h"

using std::vector;

namespace open3DCV
{

template<class T, class S>
float Ransac<T, S>::estimate(Param_Estimator<T, S>* param_estimator,
                             vector<T>& data,
                             vector<S>& params,
                             float prob_wo_outliers)
{
    unsigned int ndata = static_cast<unsigned int>(data.size());
    unsigned int ndata_min = param_estimator->ndata();
    
    if (ndata < ndata_min || prob_wo_outliers <= 0.0f || prob_wo_outliers >= 1.0f)
        { return 0; }
    
    vector<T> exact_est_data;
    vector<T> ls_est_data;
    vector<S> exact_est_params;
    
    int i, j, k, l, nvotes_best = 0, nvotes_cur = 0, max_ind = INT_MAX;
    unsigned int niters;
    int* vote_best = new int[ndata]; // 1 if data[i] agrees with the best model
    int* vote_cur = new int[ndata]; // 1 if data[i] agrees with the current model
    int* not_chosen = new int[ndata]; // 1 if NOT chosen
    
    Subset_Ind_Cmp subset_ind_cmp(ndata_min);
    std::set<int*, Subset_Ind_Cmp> subset_ind(subset_ind_cmp);
    int* subset_ind_cur;
    float numerator = log(1.0 - prob_wo_outliers);
    float denominator;
    
    unsigned int max_niters = 10000; // choose(ndata, ndata_min); // way too slow
    
    params.clear();
    srand((unsigned)time(NULL));
    
    nvotes_best = 0;
    niters = max_niters;
    
    for (i = 0; i < niters; ++i)
    {
        // randomly select data for exact model fit
        std::fill(not_chosen, not_chosen + ndata, 1);
        subset_ind_cur = new int[ndata_min];
        
        exact_est_data.clear();
        
        max_ind = ndata - 1;
        for (l = 0; l < ndata_min; ++l)
        {
            // selected_ind is in [0, max_ind]
            int selected_ind = (int)(((float)rand()/(float)RAND_MAX) * max_ind + 0.5);
            for (j = -1, k = 0; j < selected_ind && k < ndata; ++k)
            {
                if (not_chosen[k])
                    { ++j; }
            }
            --k;
            exact_est_data.push_back(data[k]);
            not_chosen[k] = 0;
            --max_ind;
        }
        // get the indexes of the chosen data so we can check that this subset hasn't been chosen before
        // can we combine the code below with the ones above?
        for (l = 0, j = 0; j < ndata; ++j)
        {
            if (!not_chosen[j])
            {
                subset_ind_cur[l] = j + 1;
                ++l;
            }
        }
        
        // Because elements in a set are unique, the insertion operation checks whether each inserted element is equivalent to an element already in the container, and if so, the element is not inserted, returning an iterator to this existing element (if the function returns a value).
        std::pair<typename std::set<int *, Subset_Ind_Cmp>::iterator, bool> res = subset_ind.insert(subset_ind_cur);
        
        // first time we select this subset
        if (res.second == true)
        {
            param_estimator->estimate(exact_est_data, exact_est_params);
            
            if(exact_est_params.size() == 0)
                continue;
            
            const int nparam = param_estimator->nparam();
            int nresults = static_cast<int>(exact_est_params.size()) / nparam; // should be a integer
            
            for (int iresult = 0; iresult < nresults; ++iresult)
            {
                auto begin = exact_est_params.begin() + iresult * nparam;
                auto end = exact_est_params.begin() + (iresult + 1) * nparam;
                vector<S> exact_est_params_tmp(begin, end);
                
                nvotes_cur = 0;
                std::fill(vote_cur, vote_cur + ndata, 0);
                
                // ndata - j: number of data that have NOT been checked yet
                for (j = 0; j < ndata && nvotes_best - nvotes_cur < ndata - j + 1; ++j)
                {
                    if (param_estimator->check_inliers(data[j], exact_est_params_tmp))
                    {
                        vote_cur[j] = 1;
                        ++nvotes_cur;
                    }
                }
                
                if(nvotes_cur > nvotes_best)
                {
                    nvotes_best = nvotes_cur;
                    std::copy(vote_cur, vote_cur + ndata, vote_best);
                    
                    if(nvotes_best == ndata) // all data are inliers, terminate the loop
                        i = niters;
                    else
                    {
                        denominator = log(1.0 - pow((float)nvotes_cur / (float)ndata, (float)ndata_min));
                        niters = (int)(numerator/denominator + 0.5);
                        niters = niters < max_niters ? niters : max_niters;
                    }
                }
            }
        }
        else // this subset has already been chosen, release memeory
        {
            delete [] subset_ind_cur;
        }
    }
    
    // release the memory
    typename std::set<int*, Subset_Ind_Cmp>::iterator it = subset_ind.begin();
    typename std::set<int*, Subset_Ind_Cmp>::iterator subset_ind_end = subset_ind.end();
    while (it != subset_ind_end)
    {
        delete [] (*it);
        ++it;
    }
    subset_ind.clear();
    
    if (nvotes_best > 0)
    {
        for (j = 0; j < ndata; ++j)
        {
            if (vote_best[j])
                ls_est_data.push_back(data[j]);
        }
        param_estimator->ls_estimate(ls_est_data, params);
    }
    delete [] vote_best;
    delete [] vote_cur;
    delete [] not_chosen;
    
    return (float) nvotes_best / (float) ndata;
}

template<class T, class S>
unsigned int Ransac<T, S>::choose(unsigned int n, unsigned int m)
{
    float denominator_end, numerator_begin, numerator, denominator, i, result;
    if (n - m > m)
    {
        numerator = n - m + 1;
        denominator = m;
    }
    else
    {
        numerator_begin = m + 1;
        denominator_end = n - m;
    }
    
    for (i = numerator_begin, numerator = 1; i <= n; ++i)
        numerator *= i;
    for (i = 1, denominator = 1; i <= denominator_end; ++i)
        denominator *= i;
    result = numerator / denominator;
    
    if (denominator > std::numeric_limits<float>::max() ||
        numerator > std::numeric_limits<float>::max() ||
        static_cast<float>(std::numeric_limits<unsigned int>::max()) < result)
    {
        return std::numeric_limits<unsigned int>::max();
    }
    else
        return static_cast<unsigned int>(result);
}
    
template class Ransac<std::pair<Vec2f, Vec2f>, float>; // for eight point fundamental matrix estimation
template class Ransac<std::pair<Vec2f, Vec2f>, vector<float> >; // for seven point fundamental matrix estimation

}
