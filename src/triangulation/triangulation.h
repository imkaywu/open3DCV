#ifndef triangulation_h_
#define triangulation_h_

#include <vector>
using std::vector;
#include "sfm/track.h"
using open3DCV::Track;
#include "sfm/structure_point.h"
using open3DCV::Structure_Point;

namespace open3DCV {
    
    bool triangulate_dlt(const vector<Camera>& cameras, const Track& track, Structure_Point& struct_pts);
    
    bool triangulate_midpoint(const vector<Camera>& cameras, const Track& track, Structure_Point& struct_pts);
    
    bool triangulate_angular(const vector<Camera>& cameras, const Track& track, Structure_Point& struct_pts);
    
    
    
//! Triangulation class
/*!
 * This class contains several methods for computing the 3D scene structure
 * given camera parameters and feature track locations.
 */
class Triangulation {
public:
  //! Default Constructor
  Triangulation();
  //! Linear triangulation
  /*!
   * Computes the 3D scene structure using linear triangulation
   * \param cameras The collection of cameras
   * \param track The feature track for the 3D point
   * \return The 3D computed scene point
   */
  StructurePoint linear(const vector<Camera> &cameras,
    const FeatureTrack &track) const;
  //! Midpoint triangulation
  /*!
   * Computes the 3D scene structure using midpoint triangulation.  This
   * function computes the first midpoint that has a distance to baseline
   * of less than .1.  If the function does not find a midpoint that meets
   * the requirement then it returns the midpoint with the smallest ratio.
   * \param cameras The collection of cameras
   * \param track The feature track for the 3D point
   * \return The 3D computed scene point
   */
  StructurePoint midpoint(const vector<Camera> &cameras,
    const FeatureTrack &track) const;
  //! Recker et. al '12 angular triangulation
  /*!
   * Computes the 3D scene structure using the algorithm proposed in
   * WACV '12 by Recker et. al. This implementation uses a midpoint start
   * and also uses an adaptive gradient descent for optimization. This
   * implementation also does not compute the statistical sample used
   * in the paper.  The sample should be computed prior to calling this
   * function.
   * \param cameras The collection of cameras
   * \param track The feature track for the 3D point
   * \return The 3D computed scene point
   */
  StructurePoint angular(const vector<Camera> &cameras,
    const FeatureTrack &track) const;

private:
  //! Gradient computation for Recker et. al '12
  /*!
   * This function calculates the gradient of the cost function used in
   * WACV Recker et. al '12.
   * \param cameras The collection of cameras
   * \param track The feature track
   * \param point The current x,y,z point at which the gradient is to be
   *              evaluated.
   * \return The gradient at the specified point
   */
  Vector3d angular_gradient(const vector<Camera> &cameras, const FeatureTrack&
    track, const Vector3d &point) const;
};
}


#endif // triangulation_h_
