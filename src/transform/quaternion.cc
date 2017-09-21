#include "transform/quaternion.h"

namespace open3DCV
{
    void proj2quat(Mat4f& proj, float q[6]) {
        float s;
        
        q[3] = proj(0, 3);
        q[4] = proj(1, 3);
        q[5] = proj(2, 3);
        q[0] = 0;
        q[1] = 0;
        q[2] = 0;
        if (proj(2, 0) == 1.0f) {
            q[1] = -M_PI / 2.0f;
            q[2] = 0.0f;
            q[0] = atan2f(-proj(0, 1), proj(1, 1));
        }
        else {
            if (proj(2, 0) == -1.0f) {
                q[1] = M_PI / 2.0f;
                q[2] = 0.0f;
                q[0] = atan2f(proj(0, 1), proj(1, 1));
            }
            else {
                q[1] = asinf(-proj(2, 0));
                if (cosf(q[1]) > 0.0f) {
                    s = 1.0f;
                }
                else {
                    s = -1.0f;
                }
                q[0] = atan2f(proj(2, 1) * s, proj(2, 2) * s);
                q[2] = atan2f(proj(1, 0) * s, proj(0, 0) * s);
            }
        }
        q[0] = q[0] * 180.0f / M_PI;//RadInDeg;
        q[1] = q[1] * 180.0f / M_PI;//RadInDeg;
        q[2] = q[2] * 180.0f / M_PI;//RadInDeg;
        for(int i = 0; i < 3; i++){
            if (fabsf(q[i]) > 180.0f){
                q[i] = (q[i] > 0) ? q[i] - 360.0f : q[i] + 360.0f;
            }
        }
    }
    
    void quat2proj(const float q[6], Mat4f &proj) {
        const float a = q[0] * M_PI / 180.0;
        const float b = q[1] * M_PI / 180.0;
        const float g = q[2] * M_PI / 180.0;
        
        const float s1 = sinf(a);  const float s2 = sinf(b);  const float s3 = sinf(g);
        const float c1 = cosf(a);  const float c2 = cosf(b);  const float c3 = cosf(g);
        
        /*   Premiere colonne*/	/*   Seconde colonne	*/
        proj(0, 0) = c2 * c3; 		proj(0, 1) = c3 * s2 * s1 - s3 * c1;
        proj(1, 0) = s3 * c2; 		proj(1, 1) = s3 * s2 * s1 + c3 * c1;
        proj(2, 0) = -s2;   		proj(2, 1) = c2 * s1;
        
        /*   Troisieme colonne*/	/*  Quatrieme colonne	*/
        proj(0, 2) = c3 * s2 * c1 + s3 * s1; 	proj(0, 3) = q[3];
        proj(1, 2) = s3 * s2 * c1 - c3 * s1; 	proj(1, 3) = q[4];
        proj(2, 2) = c2 * c1;                   proj(2, 3) = q[5];
        
        proj(3, 0) = proj(3, 1) = proj(3, 2) = 0.0f;
        proj(3, 3) = 1.0f;
    }
}
