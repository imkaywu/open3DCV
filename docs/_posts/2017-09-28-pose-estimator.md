---
layout: page
title: "Pose Estimator"
category: doc
order: 5
---

### Fundamental Matrix
`class Fundamental_Estimator`

This class is the sub-class of class `Param_Estimator`, which is a base class for various estimators that can be used by the `Ransac` class. Please visit the `RANSAC` page for details.

#### Seven-point algorithm
```cpp
void fund_seven_pts (const std::vector<Vec2f>& x1, const std::vector<Vec2f>& x2, vector<Mat3f>& F);
```
method that estimates fundamental matrix using seven corresponding pairs, which may return 1 or 3 results.

#### Eigen-point algorithm
```cpp
void fund_eight_pts(const std::vector<Vec2f>& x1, const std::vector<Vec2f>& x2, Mat3f& F);
```
method that estimates fundamental matrix using eight corresponding pairs. The fundamental matrix is estimated up to a scale.

#### Robust estimation using RANSAC
```cpp
void estimate(std::vector<DMatch>& data, std::vector<float>& params);
```
method inherited from class `Param_Estimator`, which is used within the `Ransac` class to estimate parameters using either seven-point or eight-point algorithm. I suggest to use seven point algorithm since fewer data requires fewer iterations.

```cpp
void ls_estimate(std::vector<DMatch>& data, std::vector<float>& params);
```
method that uses eight-point algorithm to estimate the least squares solution. This method is invoked at the end of RANSAC once the inliers have been detected.

```cpp
int check_inlier(std::pair<DMatch>& data, std::vector<float>& params);
```
method used to see if the data fits the estimated model.

### Rigid pose from Essential matrix
`Rt_from_E.h`

This file implements the algorithm to estimate relative pose (rotation and translation) of two views from Essential matrix. This method is used extensively in two-view SfM to estimate relative pose of two viewpoints.

```cpp
void Rt_from_E(const Mat3f& E, std::vector<Mat3f>& R, std::vector<Vec3f>& t);
```
method that returns two rotation matrices and two translation vectors due to ambiguity.

```cpp
void Rt_from_E(Pair& pair);
```
class `Pair` is a data structure that stores various information of two views, including relative pose, matching points, and so on. This method invokes the `void Rt_from_E(const Mat3f& E, std::vector<Mat3f>& R, std::vector<Vec3f>& t)` method, and then eliminates ambiguity by ensuring that the reconstructed structure is in front of the two views.

### Essential Matrix
`class Essential_Estimator`


### Homography
`class Homography_Estimator`