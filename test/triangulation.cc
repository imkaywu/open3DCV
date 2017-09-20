#include "triangulation/triangulation.h"

#include "math/numeric.h"

using namespace std;
using namespace open3DCV;

int main(const int argc, const char** argv)
{
    int npts = 2;
    vector<Mat34f> poses(npts);
    Mat3f K, R;
    Vec3f t;
    K << 1520.400000, 0.000000, 302.320000,
         0.000000, 1525.900000, 246.870000,
         0.000000, 0.000000, 1.000000;
    R << 0.02187598221295043000, 0.98329680886213122000, -0.18068986436368856000,
         0.99856708067455469000, -0.01266114646423925600, 0.05199500709979997700,
         0.04883878372068499500, -0.18156839221560722000, -0.98216479887691122000;
    t << -0.0726637729648, 0.0223360353405, 0.614604845959;
    poses[0].block<3, 3>(0, 0) = K * R;
    poses[0].block<3, 1>(0, 3) = K * t;
    
    K << 1520.400000, 0.000000, 302.320000,
         0.000000, 1525.900000, 246.870000,
         0.000000, 0.000000, 1.000000;
    R << -0.03472199972816788400, 0.98429285136236500000, -0.17309524976677537000,
         0.93942192751145170000, -0.02695166652093134900, -0.34170169707277304000,
         -0.34099974317519038000, -0.17447403941185566000, -0.92373047190496216000;
    t << -0.0746307029819, 0.0338148092011, 0.600850565131;
    poses[1].block<3, 3>(0, 0) = K * R;
    poses[1].block<3, 1>(0, 3) = K * t;
    
    Vec2f xrange(-0.073568, 0.028855), yrange(0.021728, 0.181892), zrange(-0.012445, 0.062736);
    srand (time(NULL));
    
    vector<float> err(3);
    err[0] = err[1] = err[2] = 0.0f;
    for (int i = 0; i < 100; ++i)
    {
        float a = float(rand()) / RAND_MAX;
        Vec3f X;
        X << a * (xrange(1) - xrange(0)) + xrange(0),
        a * (yrange(1) - yrange(0)) + yrange(0),
        a * (zrange(1) - zrange(0)) + zrange(0);
        
        vector<Vec3f> centers(npts);
        vector<Vec3f> directs(npts);
        vector<Vec2f> pts(npts);
        for (int i = 0; i < npts; ++i)
        {
            Vec3f x = poses[i] * X.homogeneous();
            pts[i] = x.block<2, 1>(0, 0) / x(2);
            Mat4f M;
            M.block<3, 4>(0, 0) = poses[i];
            M.block<1, 4>(3, 0).setZero();
            Vec4f c;
            nullspace<Mat4f, Vec4f>(&M, &c);
            centers[i] = c.block<3, 1>(0, 0) / c(3);
            directs[i] = poses[i].block<1, 3>(2, 0);
            directs[i] /= directs[i].norm();
        }
        
        Vec3f X_est;
        // linear triangulation
        triangulate_linear(poses, pts, X_est);
        err[0] += (X - X_est).dot(X - X_est);
        
        // midpoint triangulation
        triangulate_midpoint(centers, directs, pts, X_est);
        err[1] += (X - X_est).dot(X - X_est);
        
        // nonlinear triangulation
        triangulate_nonlinear(poses, pts, X_est);
        err[2] += (X - X_est).dot(X - X_est);
        
    }
    cout << "linear triangulation: " << err[0] / 100 << endl;
    cout << "midpoint triangulation: " << err[1] / 100 << endl;
    cout << "nonlinear triangulation: " << err[2] / 100 << endl;
}
