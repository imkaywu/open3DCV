#include <iostream>
#include <fstream>
#include "match_io.h"

using std::cerr;
using std::endl;
using std::ifstream;
using std::ofstream;
using std::vector;
using std::string;

namespace open3DCV
{
    int read_matches(const string fname, const std::vector<Keypoint>& keys1, const std::vector<Keypoint>& keys2, vector<DMatch>& matches)
    {
        std::ifstream ifstr;
        ifstr.open(fname);
        
        if (!ifstr.is_open())
        {
            cerr << "Cannot open file." << endl;
            return 1;
        }
        
        while (!ifstr.eof())
        {
            DMatch match;
            ifstr >> match.ikey1_ >> match.ikey2_ >> match.dist_;
            matches.push_back(match);
        }
        ifstr.close();
        
        return 0;
    }
    
    int read_matches(const string fname, std::vector<std::pair<Vec2f, Vec2f> >& matches)
    {
        std::ifstream ifstr;
        ifstr.open(fname);
        
        if (!ifstr.is_open())
        {
            cerr << "Cannot open file." << endl;
            return 1;
        }
        
        while (!ifstr.eof())
        {
            std::pair<Vec2f, Vec2f> match;
            
            ifstr >> match.first(0) >> match.first(1) >> match.second(0) >> match.second(1);
            matches.push_back(match);
        }
        ifstr.close();
        
        return 0;
        
    }
    
    int write_matches(const string fname, const std::vector<Keypoint>& keys1, const std::vector<Keypoint>& keys2, const vector<DMatch>& matches)
    {
        std::ofstream ofstr;
        ofstr.open(fname);
        
        if (!ofstr.is_open())
        {
            std::cerr << "Cannot create file." << endl;
            return 1;
        }
        
        for (int i = 0; i < matches.size(); ++i)
        {
            const Vec2f& key1 = keys1[matches[i].ikey1_].coords();
            const Vec2f& key2 = keys2[matches[i].ikey2_].coords();
            ofstr << key1(0) << " " << key1(1) << " " << key2(0) << " " << key2(1) <<std::endl;
        }
        ofstr.close();
        
        return 0;
    }
    
    int write_matches(const std::string fname, const std::vector<std::pair<Vec2f, Vec2f> >& matches)
    {
        std::ofstream ofstr;
        ofstr.open(fname);
        
        if (!ofstr.is_open())
        {
            std::cerr << "Cannot create file." << endl;
            return 1;
        }
        
        for (int i = 0; i < matches.size(); ++i)
        {
            const Vec2f& key1 = matches[i].first;
            const Vec2f& key2 = matches[i].second;
            ofstr << key1(0) << " " << key1(1) << " " << key2(0) << " " << key2(1) <<std::endl;
        }
        ofstr.close();
        
        return 0;
    }
}
