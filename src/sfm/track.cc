#include "sfm/track.h"

using std::vector;
using std::pair;

namespace open3DCV
{
    Track::Track()
    {
        // no op
    }
    
    Track::Track(const Track& track)
    {
        keys_ = track.keys_;
    }
    
    Track& Track::operator=(const Track& track)
    {
        keys_ = track.keys_;
        return *this;
    }
    
    void Track::add_keypoint(const Keypoint &k) {
        keys_.push_back(k);
    }
    
    void Track::rm_keypoint(unsigned int index) {
        keys_.erase(keys_.begin() + index);
    }
    
    const Keypoint &Track::operator[](unsigned int index) const {
        return keys_[index];
    }
    
    unsigned int Track::size() const {
        return static_cast<unsigned int>(keys_.size());
    }
    
    int Track::has_overlapping_keypoints(const Track& track1, const Track& track2)
    {
        for (int i = 0; i < track1.size(); ++i)
        {
            for (int j = 0; j < track2.size(); ++i)
            {
                if (Keypoint::is_identical(track1[i], track2[j]))
                    return 1;
            }
        }
        return 0;
    }
    
    void Track::find_overlapping_keypoints(const Track& track1, const Track& track2, vector<pair<int, int> >& ind_key)
    {
        ind_key.clear();
        for (int i = 0; i < track1.size(); ++i)
            for (int j = 0; j < track2.size(); ++i)
                if (Keypoint::is_identical(track1[i], track2[j]))
                {
                    ind_key.push_back(pair<int, int>(i, j));
                }
    }
}
