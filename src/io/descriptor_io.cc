#include "descriptor.h"

namespace open3DCV
{
    int read_descriptors(const string fname, vector<Vecf>& desc)
    {
        std::ifstream ifstr;
        ifstr.open(fname, std::ifstream::in);
        
        if (!ifstr.is_open())
        {
            std::cerr << "Cannot open file." << std::endl;
            return 1;
        }
        
        while (!ifstr.eof())
        {
            Vecf d(128);
            for (int i = 0; i < 128; ++i)
            { ifstr >> d(i); }
            desc.push_back(d);
        }
        ifstr.close();
        
        return 0;
    }
    
    int write_descriptors(const string fname, const vector<Vecf>& desc)
    {
        std::ofstream ofstr;
        ofstr.open(fname, std::ofstream::out);
        
        if (!ofstr.is_open())
        {
            std::cerr << "Cannot create file." << std::endl;
            return 1;
        }
        
        for (int i = 0; i < desc.size(); ++i)
        {
            for (int d = 0; d < desc[0].size(); ++d)
            { ofstr << desc[i](d) << " "; }
            ofstr << std::endl;
        }
        ofstr.close();
        
        return 0;
    }
}
