#ifndef track_h_
#define track_h_

#include "keypoint/keypoint.h"

namespace open3DCV
{
    class Track
    {
    public:
        Track() { };
        
        void add_keypoint(const Keypoint &k);
        void rm_keypoint(unsigned int index);
        const Keypoint &operator[](unsigned int index) const;
        unsigned int size() const; // length of the feature track
        
    protected:
        std::vector<Keypoint> keypoints; // the collection of keypoints
        
    };
    
    inline void Track::add_keypoint(const Keypoint &k) {
        keypoints.push_back(k);
    }
    
    inline void Track::rm_keypoint(unsigned int index) {
        keypoints.erase(keypoints.begin() + index);
    }
    
    inline const Keypoint &Track::operator [](unsigned int index) const {
        return keypoints[index];
    }
    
    inline unsigned int Track::size() const {
        return static_cast<unsigned int>(keypoints.size());
    }
}

#endif // track_h_
