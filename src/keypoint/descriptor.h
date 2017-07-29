#ifndef descriptor_h_
#define descriptor_h_

#include "math/numeric.h"
#include "image/image.h"
#include "keypoint/keypoint.h"

namespace open3DCV {
    class Descriptor {
    public:
        Descriptor();
        virtual ~Descriptor();
        
        virtual int extract_descriptor(const Image& image, vector<Keypoint>& keypoints, Mat* descriptor);
    };
} // namespace open3DCV

#endif
