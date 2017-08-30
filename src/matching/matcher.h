#ifndef matcher_h_
#define matcher_h_

#include <fstream>
#include "keypoint/keypoint.h"
#include "keypoint/descriptor.h"
#include "matching/matcher_param.h"
#include "matching/match.h"
#include "flann/nanoflann.h"

namespace open3DCV
{
    
    class Matcher
    {
    public:
        Matcher() { };
        Matcher(Matcher_Param r_matcher_param);
        virtual ~Matcher() { };
        
        virtual void init_param(Matcher_Param r_matcher_param) = 0;
        virtual int match(const vector<Vecf>& desc1, const vector<Vecf>& desc2, vector<Match>& matches) = 0;
        static int read_matches(vector<Match>& matches, const string fname);
        static int write_matches(const vector<Match>& matches, const string fname);
        
    protected:
        Matcher_Param matcher_param_;
    };
    
    inline int Matcher::read_matches(vector<Match>& matches, const string fname)
    {
        std::ifstream ifstr;
        ifstr.open(fname);
        
        if (!ifstr.is_open())
        {
            std::cerr << "Cannot open file." << std::endl;
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
    
    inline int Matcher::write_matches(const vector<Match>& matches, const string fname)
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
    
} // namespace open3DCV

#endif
