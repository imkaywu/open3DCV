#ifndef match_h_
#define match_h_

namespace open3DCV
{
class Match
{
public:
    Match () { };
    Match (int r_ikey1, int r_ikey2, float dist) :
        ikey1_(r_ikey1), ikey2_(r_ikey2), dist_(dist) { };
    
    int ikey1_;
    int ikey2_;
    float dist_;
};
    
} // namespace open3DCV

#endif
