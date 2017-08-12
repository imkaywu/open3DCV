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
        
        virtual int extract_descriptor(const Image& image, const Keypoint& keypoint, Vecf& descriptor) = 0;
        virtual int extract_descriptors(const Image& image, vector<Keypoint>& keypoints, vector<Vecf>& descriptors);
    };
    
    // If any descriptors could not be extracted at a given keypoint, that keypoint would be removed from the container
    inline int Descriptor::extract_descriptors(const Image& image, vector<Keypoint>& keypoints, vector<Vecf>& descriptors)
    {
        descriptors.reserve(keypoints.size());
        
        auto keypoint_it = keypoints.begin();
        while (keypoint_it != keypoints.end())
        {
            Vecf descriptor;
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
