#ifndef descriptor_h_
#define descriptor_h_

#include <fstream>
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
        static int read_descriptors(vector<Vecf>& descriptors, const string fname);
        static int write_descriptors(const vector<Vecf>& descriptors, const string fname);
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
    
    inline int Descriptor::read_descriptors(vector<Vecf>& desc, const string fname)
    {
        std::ifstream ifstr;
        ifstr.open(fname, std::ifstream::in);
        
        if (!ifstr.is_open())
        {
            std::cerr << "Cannot open file." << std::endl;
            return 1;
        }
        
        while (!ifstr.eof())
        {
            Vecf d(128);
            for (int i = 0; i < 128; ++i)
            { ifstr >> d(i); }
            desc.push_back(d);
        }
        ifstr.close();
        
        return 0;
    }
    
    inline int Descriptor::write_descriptors(const vector<Vecf>& desc, const string fname)
    {
        std::ofstream ofstr;
        ofstr.open(fname, std::ofstream::out);
        
        if (!ofstr.is_open())
        {
            std::cerr << "Cannot create file." << std::endl;
            return 1;
        }
        
        for (int i = 0; i < desc.size(); ++i)
        {
            for (int d = 0; d < desc[0].size(); ++d)
            { ofstr << desc[i](d) << " "; }
            ofstr << std::endl;
        }
        ofstr.close();
        
        return 0;
    }
} // namespace open3DCV

#endif
