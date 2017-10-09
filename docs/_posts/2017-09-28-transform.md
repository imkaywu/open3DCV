---
layout: page
title: "Transform"
category: doc
order: 7
---

### Projection
`projection.h`

This file implements methods that are related to the usage of camera projection matrix. Since these are widely used methods, it wouldn't make sense to make them member methods of the `Camera` class.

```cpp
void P_from_KRt(const Mat3f& K, const Mat3f& R, const Vec3f& t, Mat34f& P);
```
method that construct $$3\times 4$$ projection matrix from camera intrinsic matrix $$K$$ and extrinsic matrix $$\begin{bmatrix}R | t\end{bmatrix}$$.

```cpp
void KRt_from_P(const Mat34f& P, Mat3f& K, Mat3f& R, Vec3f& t);
```
method that decomposes a projection matrix $$P$$ to obtain the camera intrinsic matrix $$K$$ and extrinsic matrix $$\begin{bmatrix}R | t\end{bmatrix}$$.

```cpp
void project(const Mat34f& P, const Vec4f& X, Vec3f& x);
void project(const Mat34f& P, const Vec4f& X, Vec2f& x);
void project(const Mat34f& P, const Vec3f& X, Vec3f& x);
void project(const Mat34f& P, const Vec3f& X, Vec2f& x);
```
methods that compute the 2D image projection $$x$$ from a 3D point $$X$$ using projection matrix $$P$$.

```cpp
bool is_in_front_of_camera(const Mat34f& P, const Vec4f& X);
bool is_in_front_of_camera(const Mat34f& P, const Vec3f& X);
```
methods that determine if a point $$X$$ is in front of camera based on projection matrix $$P$$.

### Transform
`transform.h`

This file implements methods that creation and operation of various transformation matrices.

```cpp
Mat34f concat_Rt(const Mat34f& outer_Rt, const Mat34f& inner_Rt);
```
method to concatenate two $$3\times 4$$ matrices. The underlying formula is:
$$
outer\_Rt \cdot inner\_Rt = outer\_Rt(1:3, 1:3) \cdot inner\_Rt(1:3, 1:3)+outer\_Rt(1:3, 1:3) \cdot inner\_Rt(1:3, 4)+inner\_Rt(1:3, 4)
$$.

```cpp
Mat34f inv_Rt(const Mat34f& r_Rt);
```
method to compute the inverse of a $$3\times 4$$ matrix $$\begin{bmatrix}R' | t'\end{bmatrix}$$, i.e., $$\begin{bmatrix} R & t \\ \mathbf{0} & 1\end{bmatrix}\cdot \begin{bmatrix} R' & t' \\ \mathbf{0} & 1\end{bmatrix}=\begin{bmatrix} \mathbf{I} & \mathbf{0} \\ \mathbf{0} & 1\end{bmatrix}$$. The formula is as follows:
$$
\begin{align}
R' &= R(1:3, 1:3)^\top\\
t' &= -R(1:3, 1:3)^\top\cdot t(1:3, 4)
\end{align}
$$

```cpp
Mat3f rotation_around_x(float angle);
Mat3f rotation_around_y(float angle);
Mat3f rotation_around_z(float angle);
```
methods to compute the rotation matrix around $$x$$, $$y$$, or $$z$$ axis by `angle` degrees.

```cpp
template<typename T>
T degree2radian(T degree);

template<typename T>
T radian2degree(T radian);
```
methods to convert angle from degree to radian or the other way around.

### Rodrigues
`rodrigues.h`

This file implements the rodrigues formular that converts an axis-angle representation to a rotation matrix, as well as the inverse process.
```cpp
template<typename T>
void rodrigues(T* R, T* dR, const T* om);

void rodrigues(Mat3f& r_R, Matf* r_dR, const Vec3f& r_om);
```
methods that convert from an axis-angle representation to a rotation matrix.

```cpp
template<typename T>
void irodrigues(T* om, T* dR, const T* R);

void irodrigues(Vec3f& r_om, Matf* r_dom, const Mat3f& r_R);
```
methods that convert from a rotation matrix to an axis-angle representation.

### Quaternion
`quaternion.h`

This file implements the quaternion form of the rotation matrix.

```cpp
to be done
```