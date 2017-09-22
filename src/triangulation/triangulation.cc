#include <cmath>
#include <limits>
#include "math/numeric.h"
#include "camera/camera.h"
#include "triangulation/triangulation.h"

using std::isnan;
using std::numeric_limits;

namespace open3DCV {
    
    // ---------------------------- Linear triangulation
    void triangulate_linear(const vector<Mat34f>& poses, const vector<Vec2f>& pts, Vec3f& pt3d)
    {
        Matf A(2 * pts.size(), 4);
        for (int i = 0; i < pts.size(); ++i)
        {
            float x = pts[i](0);
            float y = pts[i](1);
            const Mat34f pose = poses[i];
            Vec4f p1t = pose.row(0);
            Vec4f p2t = pose.row(1);
            Vec4f p3t = pose.row(2);
            Vec4f a = x * p3t - p1t;
            Vec4f b = y * p3t - p2t;
            a.normalize();
            b.normalize();
            A.block<1, 4>(2 * i, 0) = a.transpose();
            A.block<1, 4>(2 * i + 1, 0) = b.transpose();
        }
        Vec4f X;
        nullspace<Matf, Vec4f>(&A, &X);
        pt3d = X.head(3) / X(3);
    }
    
    void triangulate_linear(const vector<Mat34f>& poses, const vector<Keypoint>& keys, Vec3f& pt3d)
    {
        vector<Vec2f> pts(keys.size());
        for (int i = 0; i < keys.size(); ++i)
        {
            int ind_cam = keys[i].index();
            pts[i] = keys[i].coords();
        }
        triangulate_linear(poses, pts, pt3d);
    }
    
    void triangulate_linear(const vector<Mat34f>& poses, const Track& track, Structure_Point& structure_pt)
    {
        vector<Vec2f> pts(track.size());
        Vec3i color(0, 0, 0);
        for (int i = 0; i < track.size(); ++i)
        {
            pts[i] = track[i].coords();
            color += track[i].color();
        }
        Vec3f pt3d;
        triangulate_linear(poses, pts, pt3d);
        structure_pt.coords() = pt3d;
        structure_pt.color() = color;
    }
    
    // ---------------------------- Midpoint triangulation
    void triangulate_midpoint(const vector<Vec3f>& centers, const vector<Vec3f>& directions, const vector<Vec2f>& pts, Vec3f& pt3d)
    {
        double min_dist = numeric_limits<float>::max();
        Vec3f min_midpoint(0, 0, 0);
        int size_pts = static_cast<int>(pts.size());
        
        for (int i = 0; i < size_pts; ++i)
        {
            Vec3f center1 = centers[i];
            Vec3f direct1 = directions[i];
            for (int j = i + 1; j < size_pts; ++j)
            {
                Vec3f center2 = centers[j];
                Vec3f direct2 = directions[j];
                
                Vec3f direct_cross = direct1.cross(direct2);
                float denom = 1.0f / direct_cross.dot(direct_cross);
                float a1 = ((center2 - center1).cross(direct2)).dot(direct_cross) * denom;
                float a2 = ((center2 - center1).cross(direct1)).dot(direct_cross) * denom;
                Vec3f p1 = center1 + a1 * direct1;
                Vec3f p2 = center2 + a2 * direct2;
                
                Vec3f d = p1 - p2;
                float dist = sqrt(d.dot(d));
                Vec3f cd = center1 - center2;
                float cdist = sqrt(cd.dot(cd));
                Vec3f midpoint = (p1 + p2) * 0.5;
                min_midpoint = dist * cdist < min_dist ? (min_dist = dist * cdist), midpoint : min_midpoint;
                if (dist * cdist < 0.1f)
                {
                    pt3d = min_midpoint;
                }
            }
        }
        pt3d = min_midpoint;
    }
    
//    void triangulate_midpoint(const vector<Camera>& cameras, const vector<Keypoint>& keys, Vec3f& pt3d)
//    {
//        int sz = static_cast<int>(keys.size());
//        vector<Vec3f> centers(sz);
//        vector<Vec3f> directs(sz);
//        vector<Vec2f> pts(sz);
//        
//        for (int i = 0; i < sz; ++i)
//        {
//            int idx = keys[i].index();
//            centers[i] = cameras[idx].center();
//            directs[i] = cameras[idx].direction();
//            pts[i] = keys[i].coords();
//        }
//        triangulate_midpoint(centers, directs, pts, pt3d);
//    }
//    
//    void triangulate_midpoint(const vector<Camera>& cameras, const Track& track, Structure_Point& structure_point)
//    {
//        int sz = static_cast<int>(track.size());
//        vector<Vec3f> centers(sz);
//        vector<Vec3f> directs(sz);
//        vector<Vec2f> pts(sz);
//        Vec3i color(0, 0, 0);
//        for (int i = 0; i < sz; ++i)
//        {
//            int idx = track[i].index();
//            centers[i] = cameras[idx].center();
//            directs[i] = cameras[idx].direction();
//            pts[i] = track[i].coords();
//            color += track[i].color();
//        }
//        Vec3f pt3d;
//        triangulate_midpoint(centers, directs, pts, pt3d);
//        structure_point.coords() = pt3d;
//        structure_point.color() = color / sz;
//    }
    
    // ---------------------------- Nonlinear triangulation
    // adapted from vgg_X_from_xP_nonlin
    void triangulate_nonlinear(const vector<Mat34f>& poses, const vector<Vec2f>& pts, Vec3f& pt3d)
    {
        int sz = static_cast<int>(pts.size());
        triangulate_linear(poses, pts, pt3d);
        
        Mat4f T, Ttmp;
        // ------------ need improvement
        Eigen::Matrix<float, 1, 4> pt3d_homo = pt3d.transpose().homogeneous();
        Mat4f A;
        A.setZero();
        A.block<1, 4>(0, 0) = pt3d_homo;
        Eigen::JacobiSVD<Mat4f> asvd(A, Eigen::ComputeFullV);
        T = asvd.matrixV();
        Ttmp.block<4, 3>(0, 0) = T.block<4, 3>(0, 1);
        Ttmp.block<4, 1>(0, 3) = T.block<4, 1>(0, 0);
        T = Ttmp;
        // ------------ need improvement

        vector<Mat34f> Q(sz);
        for (int i = 0; i < sz; ++i)
        {
            Q[i] = poses[i] * T;
        }
        
        // Newton
        Vec3f Y(0, 0, 0);
        Vecf err_prev(2 * sz), err(2 * sz);
        Matf J(2 * sz, 3);
        err_prev.setOnes();
        err_prev *= numeric_limits<float>::max();
        const int niters = 10;
        for (int i = 0; i < niters; ++i)
        {
            residule(Y, pts, Q, err, J);
            if (1 - err.norm() / err_prev.norm() < 10e-8)
                break;
            
            err_prev = err;
            Y = Y - (J.transpose() * J).inverse() * (J.transpose() * err);
        }
        
        Vec4f X = T * Y.homogeneous();
        pt3d = X.block<3, 1>(0, 0).array() / X(3);
        
        // for debugging
        bool is_debug = false;
        if (is_debug)
        {
            float err = 0.0f;
            for (int i = 0; i < sz; ++i)
            {
                Vec3f x_homog = poses[i] * X;
                Vec2f dx = x_homog.head<2>() / x_homog(2) - pts[i];
                err += sqrtf(dx.dot(dx));
            }
            std::cout << "error (after refinement): " << err << std::endl;
        }
    }
    
    void triangulate_nonlinear(const vector<Mat34f>& poses, const vector<Keypoint>& keys, Vec3f& pt3d)
    {
        const int nkeys = static_cast<int>(keys.size());
        vector<Vec2f> pts(nkeys);
        for (int i = 0; i < nkeys; ++i)
        {
            pts[i] = keys[i].coords();
        }
        triangulate_nonlinear(poses, pts, pt3d);
    }
    
    void triangulate_nonlinear(const vector<Mat34f>& poses, const Track& track, Structure_Point& struct_pt)
    {
        const unsigned int nkeys = static_cast<unsigned int>(track.size());
        vector<Vec2f> pts(nkeys);
        for (unsigned int i = 0; i < nkeys; ++i)
        {
            pts[i] = track[i].coords();
        }
        Vec3f pt3d;
        triangulate_nonlinear(poses, pts, pt3d);
        struct_pt.coords() = pt3d;
    }
    
    void triangulate_nonlinear(const vector<Mat34f>& poses, const vector<pair<Vec2f, Vec2f> >& pts, vector<Vec3f>& pts3d)
    {
        vector<Vec2f> pts_pair(2);
        for (int i = 0; i < pts.size(); ++i)
        {
            pts_pair[0] = pts[i].first;
            pts_pair[1] = pts[i].second;
            triangulate_nonlinear(poses, pts_pair, pts3d[i]);
        }
    }
    
    void triangulate_nonlinear(const vector<Mat34f>& poses, const vector<Track>& tracks, vector<Structure_Point>& struct_pts)
    {
        const size_t ntracks = tracks.size();
        struct_pts.resize(ntracks);
        for (size_t i = 0; i < ntracks; ++i)
        {
            triangulate_nonlinear(poses, tracks[i], struct_pts[i]);
        }
    }
    
    void triangulate_nonlinear(Graph& graph)
    {
        vector<Mat34f> poses(graph.ncams_);
        for (int i = 0; i < graph.ncams_; ++i)
        {
            poses[i] = graph.K_[i] * graph.Rt_[i];
        }
        triangulate_nonlinear(poses, graph.tracks_, graph.structure_points_);
    }
    
    void residule(const Vec3f& pt3d, const vector<Vec2f>& pts, const vector<Mat34f>& Q, Vecf& e, Matf& J)
    {
        int sz = static_cast<int>(pts.size());
        Vec3f x;
        for (int i = 0; i < sz; ++i)
        {
            Mat3f q = Q[i].block<3, 3>(0, 0);
            Vec3f x0 = Q[i].block<3, 1>(0, 3);
            x = q * pt3d + x0;
            e.block<2, 1>(2 * i, 0) = x.block<2, 1>(0, 0) / x(2) - pts[i];
            J.block<1, 3>(2 * i, 0) = (x(2) * q.row(0) - x(0) * q.row(2)) / powf(x(2), 2.0);
            J.block<1, 3>(2 * i + 1, 0) = (x(2) * q.row(1) - x(1) * q.row(2)) / powf(x(2), 2.0);
        }
    }
    
    // ---------------------------- Angular triangulation
//    StructurePoint Triangulation::angular(const vector<Camera> &cameras,
//      const FeatureTrack &track) const
//    {
//      double precision = 1e-25;
//      StructurePoint point = midpoint(cameras, track);
//      Vector3d g_old;
//      Vector3d x_old;
//      Vector3d x_new = point.coords();
//      Vector3d grad = angular_gradient(cameras, track, x_new);
//      double epsilon = .001;
//      double diff;
//      int count = 150;
//      do {
//        x_old = x_new;
//        g_old = grad;
//        x_new = x_old - epsilon * g_old;
//        grad = angular_gradient(cameras, track, x_new);
//        Vector3d sk = x_new - x_old;
//        Vector3d yk = grad - g_old;
//        double skx = sk.x();
//        double sky = sk.y();
//        double skz = sk.z();
//        diff = skx*skx+sky*sky+skz*skz;
//        //Compute adaptive step size (sometimes get a divide by zero hence
//        //the subsequent check)
//        epsilon = diff/(skx*yk.x()+sky*yk.y()+skz*yk.z());
//        epsilon = (epsilon != epsilon) ||
//          (epsilon == numeric_limits<double>::infinity()) ? .001 : epsilon;
//        --count;
//      } while(diff > precision && count-- > 0);
//      if(isnan(x_new.x()) || isnan(x_new.y()) || isnan(x_new.z())) {
//        return point;
//      }
//      return StructurePoint(x_new, point.color());
//    }
//
//    Vector3d Triangulation::angular_gradient(const vector<Camera> &cameras,
//      const FeatureTrack &track, const Vector3d &point) const
//    {
//      Vector3d g = Vector3d(0,0,0);
//      for(unsigned int i = 0; i < track.size(); ++i) {
//        const Keypoint& f = track[i];
//        const Camera& cam = cameras[f.index()];
//        Vector3d w = cam.direction(f.coords());
//        Vector3d v = point - cam.position();
//        double denom2 = v.dot(v);
//        double denom = sqrt(denom2);
//        double denom15 = pow(denom2, 1.5);
//        double vdotw = v.dot(w);
//        g.x() += (-w.x()/denom) + ((v.x()*vdotw)/denom15);
//        g.y() += (-w.y()/denom) + ((v.y()*vdotw)/denom15);
//        g.z() += (-w.z()/denom) + ((v.z()*vdotw)/denom15);
//      }
//
//      return g;
//    }
    
    // --------------------------------------- reprojection error
    float reprojection_error(const Graph& graph)
    {
        typedef unsigned int uint;
        const int ncams = graph.ncams_;
        vector<Mat34f> poses(ncams);
        for (int i = 0; i < ncams; ++i)
        {
            poses[i] = graph.K_[i] * graph.Rt_[i];
        }
        const uint ntracks = static_cast<uint>(graph.tracks_.size());
        float error = 0.0f, err = 0.0f;
        
        for (uint i = 0; i < ntracks; ++i)
        {
            err = 0.0f;
            const uint nkeys = static_cast<uint>(graph.tracks_[i].size());
            for (uint j = 0; j < nkeys; ++j)
            {
                uint ind_cam_arry = graph.index(graph.tracks_[i][j].index());
                Vec3f x_homog;
                x_homog = poses[ind_cam_arry] * graph.structure_points_[i].coords().homogeneous();
                Vec2f dx = x_homog.head(2) / x_homog(2) - graph.tracks_[i][j].coords();
                err += sqrtf(dx.dot(dx));
            }
            error += err / nkeys;
        }
        error /= ntracks;
        std::cout << "triangulation error init: " << error << std::endl;
        
        return error;
    }
}
