//#include "io/camera_io.h"
//
//namespace open3DCV
//{
//    void read_camera(const std::string fname, const int param_type)
//    {
//        cname_ = cname;
//        
//        intrinsics_.resize(6);
//        extrinsics_.resize(6);
//        projection_.resize(3, 4);
//        
//        ifstream ifstr;
//        ifstr.open(cname.c_str());
//        string header;
//        ifstr >> header;
//        if (header == "CONTOUR")
//            param_type_ = 0;
//        else if (header == "CONTOUR2")
//            param_type_ = 2;
//        else if (header == "CONTOUR3")
//            param_type_ = 3;
//        else
//        {
//            cerr << "Unrecognizable text format" << endl;
//            exit(1);
//        }
//        switch (param_type_)
//        {
//            case 0:
//                for (int i = 0; i < 6; ++i)
//                    ifstr >> intrinsics_[i];
//                for (int i = 0; i < 6; ++i)
//                    ifstr >> extrinsics_[i];
//                break;
//            case 2:
//                // read file
//                //            projection_.block(0, 0, 3, 3) = getK() * getR();
//                //            projection_.block(0, 3, 1, 3) = -getR() * getC();
//                break;
//            default:
//                break;
//        }
//        ifstr.close();
//        
//        updateProjection();
//    }
//    
//    void write_camera(const std::string fname, const int param_type)
//    {
//        ofstream ofstr;
//        ofstr.open(cname_.c_str());
//        
//        switch(param_type_)
//        {
//            case 0:
//                ofstr << "CONTOUR\n"
//                << projection_(0, 0) << " " << projection_(0, 1) << " " << projection_(0, 2) << " " << projection_(0, 3) << endl
//                << projection_(1, 0) << " " << projection_(1, 1) << " " << projection_(1, 2) << " " << projection_(1, 3) << endl
//                << projection_(2, 0) << " " << projection_(2, 1) << " " << projection_(2, 2) << " " << projection_(2, 3);
//                break;
//            case 1:
//                break;
//            default:
//                break;
//        }
//        ofstr.close();
//    }
//    
//}
