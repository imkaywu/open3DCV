#include <fstream>
#include "math/numeric.h"
#include "keypoint_io.h"

using std::ifstream;
using std::ofstream;
using std::vector;
using std::string;

namespace open3DCV
{
    int read_keypoints(const string fname, vector<Keypoint> &keypoints)
    {
        std::ifstream ifstr;
        ifstr.open(fname, std::ifstream::in);
        if (!ifstr.is_open())
        {
            std::cerr << "Cannot open the file." << std::endl;
            return 1;
        }
        
        string line;
        while (std::getline(ifstr, line))
        {
            float x, y, s, o;
            std::istringstream iss(line);
            if (!(iss >> x >> y >> s >> o)) { break; } // error
//            ifstr >> x >> y >> s >> o;
            Keypoint key(Vec2f(x, y), s, o);
            keypoints.push_back(key);
        }
        ifstr.close();
        
        return 0;
    }
    
    int read_keypoints(const string fname, vector<Vec2f>& keys)
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
            float s, o;
            Vec2f key;
            ifstr >> key(0) >> key(1) >> s >> o;
            keys.push_back(key);
        }
        ifstr.close();
        
        return 0;
    }
    
    int write_keypoints(const string fname, const vector<Keypoint> &keypoints)
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
            ofstr << key.index() << " " << key.coords()(0) << " " << key.coords()(1) << " " << key.scale() << " " << key.orientation() << std::endl;
        }
        ofstr.close();
        
        return 0;
    }

}

