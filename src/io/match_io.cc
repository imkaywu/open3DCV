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
    int read_matches(const string fname, std::vector<DMatch>& matches)
    {
        std::ifstream ifstr;
        ifstr.open(fname, std::ifstream::in);
        if (!ifstr.is_open())
        {
            std::cerr << "Cannot open the file." << std::endl;
            return 1;
        }
        
        std::string line;
        while (std::getline(ifstr, line))
        {
            DMatch match;
            Vec2f& key1 = match.point_.first;
            Vec2f& key2 = match.point_.second;
//            ifstr >> match.ind_key_.first >> match.ind_key_.second
//                  >> key1(0) >> key1(1) >> key2(0) >> key2(1)
//                  >> match.dist_;
            std::istringstream iss(line);
            if (!(iss >> match.ind_key_.first >> match.ind_key_.second >> key1(0) >> key1(1) >> key2(0) >> key2(1) >> match.dist_)) { break; } // error
            matches.push_back(match);
        }
        ifstr.close();
        
        return 0;
    }
    
    int read_matches(const string fname, const std::vector<Keypoint>& keys1, const std::vector<Keypoint>& keys2, vector<DMatch>& matches)
    {
        std::ifstream ifstr;
        ifstr.open(fname, std::ifstream::in);
        
        if (!ifstr.is_open())
        {
            cerr << "Cannot open file." << endl;
            return 1;
        }
        
        while (!ifstr.eof())
        {
            DMatch match;
            ifstr >> match.ind_key_.first >> match.ind_key_.second >> match.dist_;
            matches.push_back(match);
        }
        ifstr.close();
        
        return 0;
    }
    
    int read_matches(const string fname, std::vector<std::pair<Vec2f, Vec2f> >& matches)
    {
        std::ifstream ifstr;
        ifstr.open(fname, std::ifstream::in);
        
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
    
    int write_matches(const string fname, const vector<DMatch>& matches)
    {
        std::ofstream ofstr;
        ofstr.open(fname, std::ifstream::out);
        
        if (!ofstr.is_open())
        {
            cerr << "Cannot open file." << endl;
            return 1;
        }
        
        for (int i = 0; i < matches.size(); ++i)
        {
            const DMatch& match = matches[i];
            const Vec2f& key1 = match.point_.first;
            const Vec2f& key2 = match.point_.second;
            ofstr << match.ind_key_.first << " " << match.ind_key_.second << " "
                  << key1(0) << " " << key1(1) << " " << key2(0) << " " << key2(1) << " "
                  << match.dist_ << endl;
        }
        
        return 0;
    }
    
    int write_matches(const string fname, const std::vector<Keypoint>& keys1, const std::vector<Keypoint>& keys2, const vector<DMatch>& matches)
    {
        std::ofstream ofstr;
        ofstr.open(fname, std::ifstream::out);
        
        if (!ofstr.is_open())
        {
            std::cerr << "Cannot create file." << endl;
            return 1;
        }
        
        for (int i = 0; i < matches.size(); ++i)
        {
            const Vec2f& key1 = keys1[matches[i].ind_key_.first].coords();
            const Vec2f& key2 = keys2[matches[i].ind_key_.second].coords();
            ofstr << key1(0) << " " << key1(1) << " " << key2(0) << " " << key2(1) <<std::endl;
        }
        ofstr.close();
        
        return 0;
    }
    
    int write_matches(const std::string fname, const std::vector<std::pair<Vec2f, Vec2f> >& matches)
    {
        std::ofstream ofstr;
        ofstr.open(fname, std::ifstream::out);
        
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
