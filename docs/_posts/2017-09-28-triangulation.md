---
layout: page
title: "Triangulation"
category: doc
order: 8
---

### Linear triangulation

The linear triangulation minimizes the algebric cost function with the form of $$Ax=0$$, where $$A$$ is a $$2n\times 4$$ matrix. The solution is the right nullspace of $$A$$. This is called a Direct Linear Transformation (DLT) algorithm.

```cpp
void triangulate_linear(const vector<Mat34f>& poses, const vector<Vec2f>& pts, Vec3f& pt3d);
void triangulate_linear(const vector<Mat34f>& poses, const vector<Keypoint>& keys, Vec3f& pt3d);
void triangulate_linear(const vector<Mat34f>& poses, const Track& track, Structure_Point& struct_pts);
```

### Non-linear triangulation

The non-linear triangulation minimizes a geometric cost function, more specifically, the sampsen error. Please refer to p314-315 of Zisserman and Hartley's MVG book, or this [blog post](https://imkaywu.github.io/blog/2017/07/triangulation/) for more details.

```cpp
void triangulate_nonlinear(Graph& graph);
void triangulate_nonlinear(const vector<Mat34f>& poses, const vector<Vec2f>& pts, Vec3f& pt3d);
void triangulate_nonlinear(const vector<Mat34f>& poses, const vector<Keypoint>& keys, Vec3f& pt3d);
void triangulate_nonlinear(const vector<Mat34f>& poses, const Track& track, Structure_Point& struct_pts);
void triangulate_nonlinear(const vector<Mat34f>& poses, const vector<pair<Vec2f, Vec2f> >& pts, vector<Vec3f>& pts3d); // 2-view
void triangulate_nonlinear(const vector<Mat34f>& poses, const vector<Track>& tracks, vector<Structure_Point>& struct_pts);
```

### Midpoint triangulation

This methods finds the minimum midpoint of two 3D rays as the triangulated position. However, if the camera poses are retrieved from the Fundamental matrix, then as we know, the projection matrices are estimated up to a projective ambiguity. Thus the triangulated point is estimated up to a projective ambiguity. As we know, the notion of distance is invalid in the context of projective geometry, therefore, it makes no sense to find a point that minimizes the distance. Therefore, this method is known as projective variant. From my test results, it gives the largest reprojection error among the three implemented methods, thus I would not recommend using this method.
```cpp
void triangulate_midpoint(const vector<Vec3f>& centers, const vector<Vec3f>& directions, const vector<Vec2f>& pts, Vec3f& pt3d);
```