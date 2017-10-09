#ifndef param_estimator_
#define param_estimator_

namespace open3DCV
{
/**
 * This class defines the interface for parameter estimators
 * Classes inherit from this interface can be used by the Ransac class to perform robust estimation
 * The class include three methods:
 *      1. estimate(): estimation of parameters using the minimum amount of data (exact estimation)
 *      2. ls_estimate(): estimation of parameters using overdetermined data to minimize a least squares cost function
 *      3. check_inlier(): check if the data is inlier or outlier
 */
template<class T, class S>
class Param_Estimator
{
public:
    
    Param_Estimator(unsigned int ndata_min, unsigned int nparam) : ndata_min_(ndata_min), nparam_(nparam) { }
    
    virtual void estimate(std::vector<T>& data, std::vector<S>& params) = 0;
    
    virtual void ls_estimate(std::vector<T>& data, std::vector<S>& params) = 0;
    
    virtual int check_inlier(T& data, std::vector<S>& params) = 0;
    
    unsigned int ndata() const { return ndata_min_; }
    
    unsigned int nparam() const { return nparam_; }
    
private:
    
    unsigned int ndata_min_;
    unsigned int nparam_;
};

}
#endif
