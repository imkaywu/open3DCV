#ifndef detector_h_
#define detector_h_

#include <fstream>
#include "image/image.h"
#include "keypoint/keypoint.h"

namespace open3DCV {
    class Detector {
    public:
        Detector() { };
        virtual ~Detector() { };
        
        virtual int detect_keypoints(const Image& image, vector<Keypoint>& keypoints, int verbose = 0) = 0;
        static int read_keypoints(vector<Keypoint>& keypoints, const string fname);
        static int write_keypoints(const vector<Keypoint>& keypoints, const string fname);
    };
    
    inline int Detector::read_keypoints(vector<Keypoint> &keypoints, const string fname)
    {
        std::ifstream ifstr;
        ifstr.open(fname, std::ifstream::in);
        if (!ifstr.is_open())
        {
            std::cerr << "Cannot open the file." << std::endl;
            return 1;
        }
        
        while (!ifstr.eof())
        {
            float x, y, s, o;
            ifstr >> x >> y >> s >> o;
            Keypoint key(Vec2f(x, y), s, o);
            keypoints.push_back(key);
        }
        ifstr.close();
        
        return 0;
    }
    inline int Detector::write_keypoints(const vector<Keypoint> &keypoints, const string fname)
    {
        std::ofstream ofstr;
        ofstr.open(fname, std::ofstream::out);
        if (!ofstr.is_open())
        {
            std::cerr << "Cannot create the file." << std::endl;
            return 1;
        }
        
        // use operator overload
        for (int i = 0; i < keypoints.size(); ++i)
        {
            const Keypoint& key = keypoints[i];
            ofstr << key.coords()(0) << " " << key.coords()(1) << " " << key.scale() << " " << key.orientation() << endl;
        }
        ofstr.close();
        
        return 0;
    }
    
} // namespace open3DCV

#endif
