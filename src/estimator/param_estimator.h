#ifndef param_estimator_
#define param_estimator_

#include <vector>

using std::vector;

namespace open3DCV
{
/**
 * This class defines the interface for parameter estimators
 * Classes inherit from this interface can be used by the Ransac class to perform robust estimation
 * The class include three methods:
 *      1. estimate(): estimation of parameters using the minimum amount of data (exact estimation)
 *      2. ls_estimate(): estimation of parameters using overdetermined data to minimize a least squares cost function
 *      3. check_inliers(): check if the data is inlier or outlier
 */
template<class T, class S>
class Param_Estimator
{
public:
    
    Param_Estimator(unsigned int min_ndata, unsigned int nparam) : min_ndata_(min_ndata), nparam_(nparam) { }
    
    virtual void estimate(vector<T>& data, vector<S>& params) = 0;
    
    virtual void ls_estimate(vector<T>& data, vector<S>& params) = 0;
    
    virtual int check_inliers(T& data, vector<S>& params) = 0;
    
    unsigned int ndata() const { return min_ndata_; }
    
    unsigned int nparam() const { return nparam_; }
    
private:
    
    unsigned int min_ndata_;
    unsigned int nparam_;
};

}
#endif
