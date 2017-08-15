#include <fstream>
#include <sstream>
#include "estimator/fundamental.h"
#include "estimator/param_estimator.h"
#include "estimator/ransac.h"

using std::string;
using std::ifstream;
using std::istringstream;
using open3DCV::Vec2f;
using open3DCV::Mat3f;

void load_pts(const string fname, vector<Vec2f>& x1, vector<Vec2f>& x2)
{
    x1.clear();
    x2.clear();
    ifstream ifstr(fname.c_str());
    bool good = ifstr.is_open();
    string line;
    while (getline(ifstr, line))
    {
        istringstream iss(line);
        Vec2f y1, y2;
        iss >> y1(0) >> y1(1) >> y2(0) >> y2(1);
        x1.push_back(y1);
        x2.push_back(y2);
    }
    if (!good)
    {
        std::cerr << "Error loading points." << std::endl;
    }
}

int main(int argc, const char* argv[])
{
    string fname = "/Users/BlacKay/Documents/Projects/open3DCV/test/fundamental/matches.txt";
    vector<Vec2f> x1, x2;
    load_pts(fname, x1, x2);
    
    vector<std::pair<Vec2f, Vec2f> > data;
    for (int i = 0; i < x1.size(); ++i)
    {
        std::pair<Vec2f, Vec2f> pair_data;
        pair_data.first = x1[i];
        pair_data.second = x2[i];
        data.push_back(pair_data);
    }
    
    vector<float> params(9);
    open3DCV::Param_Estimator<std::pair<Vec2f, Vec2f>, float>* fund_esti = new open3DCV::Fundamental_Estimator(0.5f);
    open3DCV::Ransac<std::pair<Vec2f, Vec2f>, float>::estimate(fund_esti, data, params, 0.95);
    
    Mat3f F;
    F << params[0], params[1], params[2],
         params[3], params[4], params[5],
         params[6], params[7], params[8];
    std::cout << F << std::endl;
    
    return 0;
}
