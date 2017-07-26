#include <cmath>
using std::isnan;
#include <limits>
using std::numeric_limits;

#include <eigen3/Eigen/Dense>
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::Vector3i;
using Eigen::MatrixXd;
using Eigen::SelfAdjointEigenSolver;

#include "triangulation.h"

namespace SfMLibrary {
  namespace Triangulation {

    Triangulation::Triangulation() {
      //no-op
    }

    StructurePoint Triangulation::linear(const vector<Camera> &cameras,
      const FeatureTrack &track) const
    {
      MatrixXd A(2*track.size(), 4);
      Vector3i color(0,0,0);
      for(unsigned int k = 0; k < track.size(); ++k) {
        double x = track[k].coords().x();
        double y = track[k].coords().y();
        color += track[k].color();
        unsigned int index = track[k].index();
        const MatrixXd &Pi = cameras[index].projection();
        Vector4d pi1t(Pi(0,0),Pi(0,1),Pi(0,2),Pi(0,3));
        Vector4d pi2t(Pi(1,0),Pi(1,1),Pi(1,2),Pi(1,3));
        Vector4d pi3t(Pi(2,0),Pi(2,1),Pi(2,2),Pi(2,3));
        Vector4d a = x*pi3t - pi1t;
        a.normalize();
        Vector4d b = y*pi3t - pi2t;
        b.normalize();
        A(2*k,0) = a.x();A(2*k,1) = a(1);A(2*k,2) = a(2);A(2*k,3) = a(3);
        A(2*k+1,0) = b(0);A(2*k+1,1) = b(1);A(2*k+1,2) = b(2);A(2*k+1,3) = b(3);
      }
      SelfAdjointEigenSolver<MatrixXd> eigensolver(A.transpose()*A);
      MatrixXd evectors = eigensolver.eigenvectors();
      double denom = 1.0/evectors(3,0);
      Vector3d p(evectors(0,0)*denom, evectors(1,0)*denom, evectors(2,0)*denom);
      return StructurePoint(p, color);
    }

    StructurePoint Triangulation::midpoint(const vector<Camera> &cameras,
     const FeatureTrack &track) const
    {
      double min_dist = numeric_limits<double>::max();
      Vector3d min_mp;
      unsigned int size = track.size();
      Vector3i color(0,0,0);
      for(unsigned int i = 0; i < size; ++i) {
        unsigned int i1 = track[i].index();
        Vector3d pos1 = cameras[i1].position();
        Vector3d dir1 = cameras[i1].direction(track[i].coords());
        for(unsigned int j = i+1; j < size; ++j) {
          unsigned int i2 = track[j].index();
          Vector3d pos2 = cameras[i2].position();
          Vector3d dir2 = cameras[i2].direction(track[j].coords());
          Vector3d c = dir1.cross(dir2);
          double x = c.x(), y = c.y(), z = c.z();
          double denom = 1.0/(x*x + y*y + z*z);
          double a1 = ((pos2-pos1).cross(dir2)).dot(c)*denom;
          double a2 = ((pos2-pos1).cross(dir1)).dot(c)*denom;
          Vector3d p1 = pos1 + a1 * dir1;
          Vector3d p2 = pos2 + a2 * dir2;
          Vector3d d = p1-p2;
          double dist = sqrt(d.dot(d));
          Vector3d cd = pos1-pos2;
          double cdist = 1/sqrt(cd.dot(cd));
          Vector3d mp = (p1+p2)*0.5;
          min_mp = dist*cdist < min_dist ? (min_dist = dist*cdist), mp : min_mp;
          if(dist*cdist < .1) {
            color = track[i].color() + track[j].color();
            color /= 2;
            return StructurePoint(min_mp, color);
          }
        }
      }
      return StructurePoint(min_mp, track[0].color());
    }

    StructurePoint Triangulation::angular(const vector<Camera> &cameras,
      const FeatureTrack &track) const
    {
      double precision = 1e-25;
      StructurePoint point = midpoint(cameras, track);
      Vector3d g_old;
      Vector3d x_old;
      Vector3d x_new = point.coords();
      Vector3d grad = angular_gradient(cameras, track, x_new);
      double epsilon = .001;
      double diff;
      int count = 150;
      do {
        x_old = x_new;
        g_old = grad;
        x_new = x_old - epsilon * g_old;
        grad = angular_gradient(cameras, track, x_new);
        Vector3d sk = x_new - x_old;
        Vector3d yk = grad - g_old;
        double skx = sk.x();
        double sky = sk.y();
        double skz = sk.z();
        diff = skx*skx+sky*sky+skz*skz;
        //Compute adaptive step size (sometimes get a divide by zero hence
        //the subsequent check)
        epsilon = diff/(skx*yk.x()+sky*yk.y()+skz*yk.z());
        epsilon = (epsilon != epsilon) ||
          (epsilon == numeric_limits<double>::infinity()) ? .001 : epsilon;
        --count;
      } while(diff > precision && count-- > 0);
      if(isnan(x_new.x()) || isnan(x_new.y()) || isnan(x_new.z())) {
        return point;
      }
      return StructurePoint(x_new, point.color());
    }

    Vector3d Triangulation::angular_gradient(const vector<Camera> &cameras,
      const FeatureTrack &track, const Vector3d &point) const
    {
      Vector3d g = Vector3d(0,0,0);
      for(unsigned int i = 0; i < track.size(); ++i) {
        const Keypoint& f = track[i];
        const Camera& cam = cameras[f.index()];
        Vector3d w = cam.direction(f.coords());
        Vector3d v = point - cam.position();
        double denom2 = v.dot(v);
        double denom = sqrt(denom2);
        double denom15 = pow(denom2, 1.5);
        double vdotw = v.dot(w);
        g.x() += (-w.x()/denom) + ((v.x()*vdotw)/denom15);
        g.y() += (-w.y()/denom) + ((v.y()*vdotw)/denom15);
        g.z() += (-w.z()/denom) + ((v.z()*vdotw)/denom15);
      }

      return g;
    }
  }
}
