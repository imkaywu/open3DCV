#ifndef bundle_adjust_h_
#define bundle_adjust_h_

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "math/numeric.h"
#include "transform/rodrigues.h"
#include "sfm/graph.h"

// This file is heavily adapted from `libmv_bundle_adjuster.cc`
namespace open3DCV
{
    // camera intrinsics to be bundled
    // BUNDLE_RADIAL implies bundling of k1 and k2 only, no bundling of k3 is possible at this moment.
    enum BundleIntrinsics {
        BUNDLE_NO_INTRINSICS = 0,  // 00000000
        BUNDLE_FOCAL_LENGTH = 1,   // 00000001
        BUNDLE_PRINCIPAL_POINT = 2,// 00000010
        BUNDLE_RADIAL_K1 = 4,      // 00000100
        BUNDLE_RADIAL_K2 = 8,      // 00001000
        BUNDLE_RADIAL = 12,        // 00001100
        BUNDLE_TANGENTIAL_P1 = 16, // 00010000
        BUNDLE_TANGENTIAL_P2 = 32, // 00100000
        BUNDLE_TANGENTIAL = 48,    // 00110000
    };
    
    // The intrinsics need to get combined into a single parameter block;
    // use these enums to index instead of numeric constants.
    enum {
        OFFSET_FOCAL_LENGTH = 0,
        OFFSET_PRINCIPAL_POINT_X,
        OFFSET_PRINCIPAL_POINT_Y,
        OFFSET_K1,
        OFFSET_K2,
        OFFSET_K3,
        OFFSET_P1,
        OFFSET_P2,
    };
    
    template <typename T>
    inline void apply_radio_distortion_camera_intrinsics(const T &focal_length_x,
                                                         const T &focal_length_y,
                                                         const T &principal_point_x,
                                                         const T &principal_point_y,
                                                         const T &k1,
                                                         const T &k2,
                                                         const T &k3,
                                                         const T &p1,
                                                         const T &p2,
                                                         const T &normalized_x,
                                                         const T &normalized_y,
                                                         T *image_x,
                                                         T *image_y)
    {
        T x = normalized_x;
        T y = normalized_y;
        
        // apply distortion to the normalized points to get (xd, yd)
        T r2 = x*x + y*y;
        T r4 = r2 * r2;
        T r6 = r4 * r2;
        T r_coeff = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
        T xd = x * r_coeff + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        T yd = y * r_coeff + 2.0 * p2 * x * y + p1 * (r2 + 2.0 * y * y);
        
        // apply focal length and principal point to get the final image coordinates
        *image_x = focal_length_x * xd + principal_point_x;
        *image_y = focal_length_y * yd + principal_point_y;
    }
    
    struct Open3DCVReprojectionError
    {
        Open3DCVReprojectionError(const double observed_x, const double observed_y)
            : observed_x_(observed_x), observed_y_(observed_y) {}
        
        template<typename T>
        bool operator()(const T* const intrinsics,
                        const T* const extrinsics,
                        const T* const point,
                        T* residules) const
        {
            const T& focal_length       = intrinsics[OFFSET_FOCAL_LENGTH];
            const T& principal_point_x  = intrinsics[OFFSET_PRINCIPAL_POINT_X];
            const T& principal_point_y  = intrinsics[OFFSET_PRINCIPAL_POINT_Y];
            const T& k1                 = intrinsics[OFFSET_K1];
            const T& k2                 = intrinsics[OFFSET_K2];
            const T& k3                 = intrinsics[OFFSET_K3];
            const T& p1                 = intrinsics[OFFSET_P1];
            const T& p2                 = intrinsics[OFFSET_P2];
            
            // compute projective coordinates: x = RX + t.
            // extrinsics[0, 1, 2]: axis-angle
            // extrinsics[3, 4, 5]: translation
            T x[3];
            ceres::AngleAxisRotatePoint(extrinsics, point, x);
            x[0] += extrinsics[3];
            x[1] += extrinsics[4];
            x[2] += extrinsics[5];
            
            // compute normalized coordinates
            T xn = x[0] / x[2];
            T yn = x[1] / x[2];
            
            T predicted_x, predicted_y;
            
            // apply distortion to the normalized points to get (xd, yd)
            // do something for zero distortion
//            apply_radio_distortion_camera_intrinsics(focal_length,
//                                                     focal_length,
//                                                     principal_point_x,
//                                                     principal_point_y,
//                                                     k1, k2, k3,
//                                                     p1, p2,
//                                                     xn, yn,
//                                                     &predicted_x,
//                                                     &predicted_y);
            predicted_x = focal_length * xn + principal_point_x;
            predicted_y = focal_length * yn + principal_point_y;
            
            residules[0] = predicted_x - T(observed_x_);
            residules[1] = predicted_y - T(observed_y_);
            return true;
        }
        
        // Factory to hide the construction of the CostFunction object from the client code
        static ceres::CostFunction* create(const float observed_x,
                                           const float observed_y)
        {
            return (new ceres::AutoDiffCostFunction<Open3DCVReprojectionError, 2, 8, 6, 3>(
                        new Open3DCVReprojectionError(observed_x, observed_y)));
        }
        
        double observed_x_;
        double observed_y_;
    };
    
    vector<Vec3> pack_3d_pts(const Graph&graph)
    {
        size_t npts = static_cast<size_t>(graph.structure_points_.size());
        vector<Vec3> pts3d(npts);
        for (int i = 0; i < npts; ++i)
        {
            pts3d[i] = graph.structure_points_[i].coords().cast<double>();
        }
        
        return pts3d;
    }
    
    void unpack_3d_pts(Graph &graph, const vector<Vec3>& pts3d)
    {
        for (int i = 0; i < graph.structure_points_.size(); ++i)
        {
            graph.structure_points_[i].coords() = pts3d[i].cast<float>();
        }
    }
    
    vector<Vec6> pack_camera_extrinsics(const Graph&graph)
    {
        vector<Vec6> extrinsics(graph.size());
        for (int i = 0; i < graph.extrinsics_mat_.size(); ++i)
        {
            Mat3 R = graph.extrinsics_mat_[i].block<3, 3>(0, 0).cast<double>();
            ceres::RotationMatrixToAngleAxis(&R(0, 0), &extrinsics[i](0));
            extrinsics[i].tail<3>() = graph.extrinsics_mat_[i].cast<double>().block<3, 1>(0, 3);
        }
        
        return extrinsics;
    }
    
    void unpack_camera_extrinsics(Graph& graph, const vector<Vec6> &extrinsics)
    {
        for (int i = 0; i < graph.extrinsics_mat_.size(); ++i)
        {
            Mat3 R;
            ceres::AngleAxisToRotationMatrix(&extrinsics[i](0), &R(0, 0));
            graph.extrinsics_mat_[i].block<3, 3>(0, 0) = R.cast<float>();
            graph.extrinsics_mat_[i].block<3, 1>(0, 3) = extrinsics[i].tail<3>().cast<float>();
        }
    }
    
    vector<Vec8> pack_camera_intrinsics(const Graph& graph)
    {
        vector<Vec8> intrinsics(graph.size());
        for (int i = 0; i < graph.intrinsics_mat_.size(); ++i)
        {
            Mat3 K = graph.intrinsics_mat_[i].cast<double>();
            intrinsics[i] << K(0, 0), K(0, 2), K(1, 2), 0, 0, 0, 0, 0;
        }
        
        return intrinsics;
    }
    
    void unpack_camera_intrinsics(Graph& graph, const vector<Vec8> &intrinsics)
    {
        for (int i = 0; i < graph.intrinsics_mat_.size(); ++i)
        {
            Mat3 K;
            K << intrinsics[i](OFFSET_FOCAL_LENGTH), 0,                                   intrinsics[i](OFFSET_PRINCIPAL_POINT_X),
                 0,                                  intrinsics[i](OFFSET_FOCAL_LENGTH),  intrinsics[i](OFFSET_PRINCIPAL_POINT_Y),
                 0,                                  0,                                   1;
            graph.intrinsics_mat_[i] = K.cast<float>();
        }
    }
    
    void Open3DCVBundleAdjustment(Graph& graph,
                                  const int bundle_intrinsics)
    {
        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);
        
        // convert camera rotation to angle axis and merge with translation
        vector<Vec6> extrinsics = pack_camera_extrinsics(graph);
        vector<Vec8> intrinsics = pack_camera_intrinsics(graph);
        vector<Vec3> pts3d = pack_3d_pts(graph);
        
        // construct the problem
        bool is_camera_locked = false;
        for (int m = 0; m < pts3d.size(); ++m)
        {
            Vec3& pt3d = pts3d[m];
            
            for (int n = 0; n < graph.tracks_[m].size(); ++n)
            {
                Keypoint key = graph.tracks_[m][n];
                double x = (double)key.coords().x();
                double y = (double)key.coords().y();
                ceres::CostFunction* cost_function = Open3DCVReprojectionError::create(x, y);
                
                int idx = graph.index(key.index());
                problem.AddResidualBlock(cost_function, NULL, &intrinsics[idx](0), &extrinsics[idx](0), &pt3d(0));
                
                // lock the first camera to better deal with scene orientation ambiguity
                if (!is_camera_locked)
                {
                    problem.SetParameterBlockConstant(&extrinsics[idx](0));
                    is_camera_locked = true;
                }
            }
        }
        
        // set part of parameters constant
        if (bundle_intrinsics == BUNDLE_NO_INTRINSICS)
        {
            for (int i = 0; i < intrinsics.size(); ++i)
                problem.SetParameterBlockConstant(&intrinsics[i](0));
        }
        else
        {
            vector<int> constant_intrinsics;
#define MAYBE_SET_CONSTANT(bundle_enum, offset) \
            if (!(bundle_intrinsics & bundle_enum)) { \
                constant_intrinsics.push_back(offset); \
            }
            MAYBE_SET_CONSTANT(BUNDLE_FOCAL_LENGTH,    OFFSET_FOCAL_LENGTH);
            MAYBE_SET_CONSTANT(BUNDLE_PRINCIPAL_POINT, OFFSET_PRINCIPAL_POINT_X);
            MAYBE_SET_CONSTANT(BUNDLE_PRINCIPAL_POINT, OFFSET_PRINCIPAL_POINT_Y);
            MAYBE_SET_CONSTANT(BUNDLE_RADIAL_K1,       OFFSET_K1);
            MAYBE_SET_CONSTANT(BUNDLE_RADIAL_K2,       OFFSET_K2);
            MAYBE_SET_CONSTANT(BUNDLE_TANGENTIAL_P1,   OFFSET_P1);
            MAYBE_SET_CONSTANT(BUNDLE_TANGENTIAL_P2,   OFFSET_P2);
#undef MAYBE_SET_CONSTANT
            
            // always set K3 constant, it's not used at the moment
            constant_intrinsics.push_back(OFFSET_K3);
            ceres::SubsetParameterization *subset_parameterizaiton =
                new ceres::SubsetParameterization(8, constant_intrinsics);
            
            for (int i = 0; i < intrinsics.size(); ++i)
                problem.SetParameterization(&intrinsics[i](0), subset_parameterizaiton);
        }
        
        // configure the solver
        ceres::Solver::Options options;
        options.use_nonmonotonic_steps = true;
        options.preconditioner_type = ceres::SCHUR_JACOBI;
        options.linear_solver_type = ceres::ITERATIVE_SCHUR;
        options.use_inner_iterations = true;
        options.max_num_iterations = 100;
        options.minimizer_progress_to_stdout = true;

        // solve
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << std::endl;
        
        // copy intrinsics and extrinsics back
        unpack_camera_extrinsics(graph, extrinsics);
        unpack_camera_intrinsics(graph, intrinsics);
        unpack_3d_pts(graph, pts3d);
    }
}

#endif
