#ifndef descriptor_h_
#define descriptor_h_

#include "math/numeric.h"
#include "image/image.h"
#include "keypoint/keypoint.h"

namespace open3DCV {
    class Descriptor {
    public:
        Descriptor() { };
        virtual ~Descriptor() { };
        
        virtual int extract_descriptor(const Image& image, const Keypoint& keypoint, Vec& descriptor) = 0;
        virtual int extract_descriptors(const Image& image, vector<Keypoint>& keypoints, vector<Vec>& descriptors);
    };
    
    inline int Descriptor::extract_descriptors(const Image& image, vector<Keypoint>& keypoints, vector<Vec>& descriptors)
    {
        descriptors.reserve(keypoints.size());
        
        auto keypoint_it = keypoints.begin();
        while (keypoint_it != keypoints.end())
        {
            Vec descriptor;
            if (!extract_descriptor(image, *keypoint_it, descriptor))
            {
                keypoint_it = keypoints.erase(keypoint_it);
                continue;
            }
            descriptors.push_back(descriptor);
        }
        return 0;
    }
} // namespace open3DCV

#endif
