---
layout: page
title: "Camera Models"
category: doc
order: 1
---

Camera is the one of the means for machines to perceive the world. The most widely used camera model is the pinhole camera model.

### Camera
`class Camera`

This is the base class of all camera classes. 

#### Pinhole camera
`class PinholeCamera`

The `Camera` class contains intrinsic and extrinsic parameters of the camera. The information are stored in both matrix and vector form, and are automatically updated once the other is changed.

* `R_` is the $$3\times 3$$ rotation matrix;
* `t_` is the $$3\times 1$$ translation matrix;
* `std::vector<float> intrinsics_` is a 4-element vector storing intrinsic parameters, which includes $$f_x$$, $$f_y$$, $$c_x$$, $$c_y$$;
* `std::vector<float> extrinsics_` is a 6-element vector storing extrinsic parameters, which include $$om_1$$, $$om_2$$, $$om_3$$, $$t_1$$, $$t_2$$, $$t_3$$. The first three is the axis-angle representation of rotation matrix, the latter three are coefficients of the translation vector.

The position and orientation of the camera are also stored in the `Camera` class.
* `center_` represents the center of projection;
* `xaxis_`, `yaxis_`, `zaxis_` store the axes of camera coordinate system;
* `oaxis_` stores principle axis.

```cpp
void update_projection()
```
update the $$3\times 4$$ projection matrix from intrinsic and extrinsic matrices `K_`, `R_`, `t_`.

```cpp
void update_projection()
```
update the camera parameter vectors from matrices.

```cpp
void update_matrices(const is is_proj_mat)
```
update camera parameter matrices from either projection matrix or camera parameter vectors.

```cpp
const Mat3f& projection() const;
Mat34f& projection();
```
return references to the projection matrix.

```cpp
const Vec3f& direction() const;
Vec3f& direction();
```
return references to the principal axis.

```cpp
const Vec3f& center() const;
Vec3f& center();
```
return references to the center of projection.