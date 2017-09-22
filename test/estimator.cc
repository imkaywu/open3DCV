#include "estimator/est_Rt_from_E.h"

using namespace std;
using namespace open3DCV;

int main (int argc, char** argv)
{
    Vec3f t;
    Mat3f R;
    t << 0.343, 0.643, 2;
    R << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
    Mat3f E;
    E = cross_mat<Mat3f, Vec3f>(t) * R;
    
    vector<Mat3f> Rs;
    vector<Vec3f> ts;
    Rt_from_E(E, Rs, ts);
    cout << "R 1: " << Rs[0] << endl << "R 2: " << Rs[1] << endl;
    cout << "t 1: " << ts[0] << endl << "t 2: " << ts[1] << endl;
    
    return 0;
}
