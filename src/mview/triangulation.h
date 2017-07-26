#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include <vector>
using std::vector;

#include "Core/camera.h"
using SfMLibrary::Core::Camera;
#include "Core/featuretrack.h"
using SfMLibrary::Core::FeatureTrack;
#include "Core/structurepoint.h"
using SfMLibrary::Core::StructurePoint;

namespace SfMLibrary {
  namespace Triangulation {
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
}


#endif // TRIANGULATION_H
