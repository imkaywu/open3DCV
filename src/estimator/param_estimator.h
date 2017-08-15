#ifndef param_estimator_
#define param_estimator_

#include <vector>

using std::vector;

namespace open3DCV
{
    
template<class T, class S>
class Param_Estimator
{
public:
    
    Param_Estimator(unsigned int min_data) : min_num_data_(min_data) { }
    
    virtual void estimate(vector<T>& data, vector<S>& params) = 0;
    
    virtual void ls_estimate(vector<T>& data, vector<S>& params) = 0;
    
    virtual int check_inliers(vector<S>& params, T& data) = 0;
    
    unsigned int num_data() const { return min_num_data_; }
    
private:
    
    unsigned int min_num_data_;
};

}
#endif
