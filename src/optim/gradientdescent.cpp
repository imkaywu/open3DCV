#include <limits>
using std::numeric_limits;

#include "gradientdescent.h"

namespace SfMLibrary {
  namespace Optimization {

    template<typename GradientFunction>
    GradientDescent<GradientFunction>::GradientDescent() {
      //no-op
    }

    template<typename GradientFunction>
    VectorXd GradientDescent<GradientFunction>::operator ()(const VectorXd& start) const {
      VectorXd xold = start;
      VectorXd xnew = start;
      VectorXd gold = gradient(start);
      VectorXd gnew = gold;
      double precision = 1e-15;
      double epsilon = .001;
      double diff;
      do {
        xold = xnew;
        gold = gnew;
        xnew = xold - epsilon * gold;
        gnew = gradient(xnew);
        VectorXd sk = xnew - xold;
        VectorXd yk = gnew - gold;
        diff = sk.squaredNorm();
        epsilon = diff/sk.dot(yk);
        epsilon = (epsilon != epsilon) ||
          (epsilon == numeric_limits<double>::infinity()) ? .001 : epsilon;
      } while(gnew.norm() > precision);
      return xnew;
    }

  }
}
