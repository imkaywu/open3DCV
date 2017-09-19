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
    int read_matches(vector<Match>& matches, const string fname)
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
            Match match;
            ifstr >> match.ikey1_ >> match.ikey2_ >> match.dist_;
            matches.push_back(match);
        }
        ifstr.close();
        
        return 0;
    }
    
    int write_matches(const vector<Match>& matches, const string fname)
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
            ofstr << matches[i].ikey1_ << " " << matches[i].ikey2_ << " " << matches[i].dist_ << std::endl;
        }
        ofstr.close();
        
        return 0;
    }
}
