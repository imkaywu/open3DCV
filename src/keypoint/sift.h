#ifndef sift_h
#define sift_h

#include "vl/sift.h"
#include "image.h"
#include "keypoint.h"

namespace open3DCV {
    
    class Sift : public Keypoint {
        
    public:
        
        Sift(const Vec2 x, unsigned int i);
        Sift(const Vec2 x, unsigned int i, const Vec3i c);
        ~Sift();
        
        int convert(Image &img, vl_sift_pix *fdata);
        int detect();
        int descript();
    };
}

#endif // sift_h
