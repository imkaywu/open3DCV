#ifndef match_h_
#define match_h_

namespace open3DCV
{
class Match
{
public:
    Match () { };
    Match (int r_ind1, int r_ind2, float dist) :
        key_ind1_(r_ind1), key_ind2_(r_ind2), dist_(dist) { };
    
    int key_ind1_;
    int key_ind2_;
    float dist_;
};
    
} // namespace open3DCV

#endif
