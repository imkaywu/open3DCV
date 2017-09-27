#include "math/numeric.h"
#include "transform/rodrigues.h"
#include "triangulation/triangulation.h"
#include "io/match_io.h"

using namespace std;
using namespace open3DCV;

int main(const int argc, const char** argv)
{
    int npts = 2;
    vector<Mat34f> poses(npts);
    Mat3f K, R;
    Vec3f t;
    float f = 719.5459;
    const int w = 480, h = 640;
    K << f,   0.0, w/2.0,
         0.0, f,   h/2.0,
         0.0, 0.0, 1.0;
    R << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
    t << 0, 0, 0;
    poses[0].block<3, 3>(0, 0) = K * R;
    poses[0].block<3, 1>(0, 3) = K * t;
    
//    R << 0.98161, -0.0989498,   0.163251,
//    0.102381,   0.994664, -0.0127199,
//    -0.161121,  0.0291998,   0.986503;
//    t << -0.999372, 0.00983581, -0.0340434;
//    poses[1].block<3, 3>(0, 0) = K * R;
//    poses[1].block<3, 1>(0, 3) = K * t;
    
    poses[1] << 0.981858,  -0.0972401 ,   0.162788 ,  -0.999901,
    0.100072,    0.994937 ,-0.00926934, -0.00368404,
    -0.161062,   0.0253916,    0.986618,   0.0136145;
    poses[1] = K * poses[1];
    
//    Vec3f om(0.2, 0.3, 0.4);
//    rodrigues(R, nullptr, om);
//    t << 0.1, 0.1, 0.1;
//    poses[1].block<3,3>(0, 0) = K * R;
//    poses[1].block<3,1>(0, 3) = K * t;
    
    // read matching
    std::vector<std::pair<Vec2f, Vec2f> > matches;
    read_matches("matches.txt", matches);
    
    vector<Vec3f> pt3d(matches.size());
    triangulate_nonlinear(poses, matches, pt3d);
    float error = 0.0f;
    for (int i = 0; i < pt3d.size(); ++i)
    {
        Vec3f x;
        x = poses[0] * pt3d[i].homogeneous();
        x = x / x(2);
        Vec2f dx = x.head<2>() - matches[i].first;
        error += sqrt(dx.dot(dx));
        
        x = poses[1] * pt3d[i].homogeneous();
        x = x / x(2);
        dx = x.head<2>() - matches[i].second;
        
        error += sqrt(dx.dot(dx));
    }
    cout << "reprojection error: " << error / (2 * pt3d.size()) << endl;
    
    /* middlebury dataset
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
     */
}
