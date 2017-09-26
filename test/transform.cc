#include "math/numeric.h"
#include "transform/rodrigues.h"

using namespace std;
using namespace open3DCV;

int main (int argc, char** argv)
{
    // test 1, float pointer
    float om[3];
    float R[9];
    om[0] = 0.2, om[1] = 0.3, om[2] = 0.4;
    rodrigues<float>(R, nullptr, om);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            cout << R[i + j * 3] << " ";
        }
        cout << endl;
    }
    
    float om_est[3];
    irodrigues<float>(om_est, nullptr, R);
    for (int i = 0; i < 3; ++i)
        cout << om_est[i] << " " << endl;
    
    
    // test 2, vec3f
    Vec3f om1;
    Mat3f R1;
    om1[0] = 1.2; om1[1] = 0.3; om1[2] = 0.4;
    rodrigues(R1, nullptr, om1);
    cout << R1 << endl;
    
    Vec3f om1_est;
    irodrigues(om1_est, nullptr, R1);
    cout << om1_est << endl;
    
    return 0;
}
