#ifndef KEYPOINT_H
#define KEYPOINT_H

#include "numeric.h"

namespace open3DCV {

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
      Keypoint(const Vec2 &x, unsigned int i);
      //! Constructor
      /*!
       * Full constructor
       * \param x The image coordinates
       * \param i The index where the image is found
       * \param c The color components
       */
      Keypoint(const Vec2 &x, unsigned int i, const Vec3i &c);
      //! Image coordinates (const reference)
      /*!
       * \return a const reference to the keypoint image coordinates
       */
      const Vec2 &coords() const;
      //! Image index
      /*!
       * \return the value of the image index
       */
      unsigned int index() const;
      //! Image color at keypoint (const reference)
      /*!
       * \return a const reference to the keypoint color
       */
      const Vec3i &color() const;

    protected:
      Vec2 p;     //!< Image coordinates
      unsigned int i; //!< Image index
      Vec3i c;     //!< Image color
    };

    inline const Vec2 &Keypoint::coords() const {
      return p;
    }

    inline unsigned int Keypoint::index() const {
      return i;
    }

    inline const Vec3i &Keypoint::color() const {
      return c;
    }

}


#endif // KEYPOINT_H
