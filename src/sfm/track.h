#ifndef track_h_
#define track_h_

#include "keypoint/keypoint.h"

namespace open3DCV
{
    class Track
    {
    public:
        Track();
        Track(const Track& track);
        Track& operator=(const Track& track);
        virtual ~Track();
        
        int size() const; // length of the feature track
        const Keypoint &operator[](int index) const;
        void add_keypoint(const Keypoint &k);
        void rm_keypoint(int index);
        std::vector<Keypoint>::iterator key_begin();
        std::vector<Keypoint>::iterator key_end();
        static int has_overlapping_keypoints(const Track& track1, const Track& track2);
        static void find_overlapping_keypoints(const Track& track1, const Track& track2, std::vector<std::pair<int, int> >& ind_key);
        
    protected:
        std::vector<Keypoint> keys_; // the collection of keypoint correspondences
        
    };
}

#endif // track_h_
