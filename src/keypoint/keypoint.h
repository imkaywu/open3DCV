#ifndef KEYPOINT_H
#define KEYPOINT_H

#include <eigen3/Eigen/Dense>
using Eigen::Vector2d;
using Eigen::Vector3i;

namespace SfMLibrary {
  namespace Core {
    //! Keypoint class
    /*!
     * This class encapsulates the data associated with a 2D image keypoint.
     * This class stores the x and y coordinates of the image location, along
     * with the index of the image (camera) it is found, and color value
     * (stored in a 0-255 format).
     */
    class Keypoint {
    public:
      //! Constructor
      /*!
       * Basic constructor sets the feature color to black (0,0,0)
       * \param x The image coordinates
       * \param i The index where the image is found
       */
      Keypoint(const Vector2d &x, unsigned int i);
      //! Constructor
      /*!
       * Full constructor
       * \param x The image coordinates
       * \param i The index where the image is found
       * \param c The color components
       */
      Keypoint(const Vector2d &x, unsigned int i, const Vector3i &c);
      //! Image coordinates (const reference)
      /*!
       * \return a const reference to the keypoint image coordinates
       */
      const Vector2d &coords() const;
      //! Image index
      /*!
       * \return the value of the image index
       */
      unsigned int index() const;
      //! Image color at keypoint (const reference)
      /*!
       * \return a const reference to the keypoint color
       */
      const Vector3i &color() const;

    protected:
      Vector2d p;     //!< Image coordinates
      unsigned int i; //!< Image index
      Vector3i c;     //!< Image color
    };

    inline const Vector2d &Keypoint::coords() const {
      return p;
    }

    inline unsigned int Keypoint::index() const {
      return i;
    }

    inline const Vector3i &Keypoint::color() const {
      return c;
    }

  }
}


#endif // KEYPOINT_H
