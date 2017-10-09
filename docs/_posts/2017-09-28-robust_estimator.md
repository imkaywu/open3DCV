---
layout: page
title: "Robust Estimator"
category: doc
order: 6
---

### Parameter Estimator
`class Param_Estimator`

This class defines the interface for all parameter estimators. Classes inherit from this interface can be used by the `Ransac` class to perform robust estimation. This class include three methods:
* `estimate()`: estimation of parameters using the minimum number of data (exact estimation);
* `ls_estimate()`: estimation of parameters using over-determined data to minimize a least squares cost function;
* `check_inlier()`: check if the data fits the estimated model.

```cpp
virtual void estimate(std::vector<T>& data, std::vector<S>& params) = 0;
```
pure virtual method that estimate parameters using minimum mumber of data.

```cpp
virtual void ls_estimate(std::vector<T>& data, std::vector<S>& params) = 0;
```
pure virtual method that estimate parameters using over-determined data that minimizes a least squares cost function.

```cpp
virtual int check_inlier(T& data, std::vector<S>& params) = 0;
```
pure virtual method that check if `data` fits the estimated parameters.

### RANSAC
`class Ransac`

This class implements the classic RANSAC framework proposed by [Martin] et al.

```cpp
template<class T, class S>
class Ransac
{
public:
    // - params:            a vector containing the estimated parameters
    // - param_estimator:   an instance which can estimate the desired parameters by either an exact
    //                      fit or a least squares fit
    // - data:              the input from which the parameters will be estimated
    // - prob_wo_outlieres: the probability that at least one of the selected subsets doens't contain an outlier,
    //                      must be in (0, 1).
    //
    // - number of iterations (k):
    //
    //               log(1 - p)
    //         k = --------------
    //              log(1 - w^n)
    //
    // - p: desired probability without an outlier
    // - w: percentage of inliers in the data
    // - n: minimum number of data for estimation
    //
    static float estimate(Param_Estimator<T, S>* param_estimator,
                          std::vector<T>& data,
                          std::vector<S>& params,
                          float prob_wo_outliers,
                          int* vote_inlier);
}
```