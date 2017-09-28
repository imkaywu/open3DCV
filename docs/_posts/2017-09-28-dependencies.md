---
layout: page
title: "Dependencies"
category: inst
order: 0
---

open3DCV depends on mostly header-only libraries, the complete dependencies are as follows:

1. [CImg](http://cimg.eu) is a header-only, open-source, and modern C++ toolkit for image processing.
2. [Eigen3](http://eigen.tuxfamily.org) is a header-only C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.
3. [VLFeat](http://www.vlfeat.org) implements popular computer vision algorithms specializing in image understanding and local features extraction and matching.
4. [nanoflann](https://github.com/jlblancoc/nanoflann) is a header-only C++ library for approximate nearest neighbor search.
5. [RpolyPlusPlus](https://github.com/sweeneychris/RpolyPlusPlus) is a header-only polynomial root-finding library. It implements a three-stage algorithm for finding roots of polynomials with real coefficients as outlined in: "A Three-Stage Algorithm for Real Polynomials Using Quadratic Iteration" by Jenkins and Traub, SIAM 1970.
6. [Ceres Solver](http://ceres-solver.org) is an open source C++ library for modeling and solving large, complicated optimization problems. In particular, open3DCV uses it for Bundle Adjustment.