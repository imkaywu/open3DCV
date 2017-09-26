//
//  numeric_test.cpp
//  open3DCV_test
//
//  Created by KaiWu on Jul/5/17.
//  Copyright © 2017 KaiWu. All rights reserved.
//

#include <stdio.h>
#include "numeric.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "transform/rodrigues.h"

using namespace std;
using namespace open3DCV;

int main(int argc, const char * argv[]) {
    
    // Eigen matrix is column major
//    Mat3 A;
//    A << 1, 2, 3,
//         4, 5, 6,
//         7, 8, 9;
//    for (int i = 0; i < 9; ++i)
//        cout << A(i) << " ";
//    cout << endl;
//    
//    double *pA = &A[0];
//    for (int i = 0; i < 9; ++i)
//        cout << *(pA+i) << " ";
//    cout << endl;
    
    // test angle-axis rotation
    double om[3];
    om[0] = 0.2; om[1] = 0.3; om[2] = 0.4;
    Vec3f om_v(0.2, 0.3, 0.4);
    Mat3 R;
    ceres::AngleAxisToRotationMatrix(om, &R(0, 0));
    cout << R << endl;
    double om_est[3];
    ceres::RotationMatrixToAngleAxis(&R(0, 0), om_est);
    cout << "(" << om_est[0] << ", " << om_est[1] << ", " << om_est[2] << ")" << endl;
    
    Mat3f R_m;
    rodrigues(R_m, nullptr, om_v);
    cout << R_m << endl;
    
    return 0;
}
