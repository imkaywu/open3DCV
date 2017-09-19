#include <fstream>
#include "math/numeric.h"
#include "keypoint_io.h"

using std::ifstream;
using std::ofstream;
using std::vector;
using std::string;

namespace open3DCV
{
    int read_keypoints(vector<Keypoint> &keypoints, const string fname)
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
    int write_keypoints(const vector<Keypoint> &keypoints, const string fname)
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
            ofstr << key.coords()(0) << " " << key.coords()(1) << " " << key.scale() << " " << key.orientation() << std::endl;
        }
        ofstr.close();
        
        return 0;
    }
}

