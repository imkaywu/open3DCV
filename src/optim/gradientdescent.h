#ifndef GRADIENTDESCENT_H
#define GRADIENTDESCENT_H

#include <eigen3/Eigen/Dense>
using Eigen::VectorXd;

namespace SfMLibrary {
  namespace Optimization {
    //! Gradient Descent Optimization Class
    /*!
     * This class encapsulates a generic gradient descent procedure.  The
     * class requires both a gradient function and starting location to start
     * the procedure from.  The class will return the local minimum that can
     * be reached from the starting position.
     */
    template<typename GradientFunction>
    class GradientDescent {
    public:
      //! Constructor
      /*!
       * The basic constructor
       */
      GradientDescent();
      //! Optimization
      /*!
       * Executes the gradient descent algorithm starting from the given start
       * parameter.
       * \param start The starting location from which to run gradient descent
       */
      VectorXd operator()(const VectorXd& start) const;

    protected:
      GradientFunction gradient;
    };
  }
}

#endif // GRADIENTDESCENT_H
