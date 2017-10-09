#include "sift.h"
#define _USE_MATH_DEFINES
extern "C" {
#include <math.h>
}
#include <algorithm>

using std::cout;
using std::endl;

namespace open3DCV {
    static const int kMaXScaledDim = 3600;
    
    Sift::Sift() : data_(nullptr), sift_filter_(nullptr)
    {
        // no op
    }
    
    Sift::Sift(const SiftParam& sift_param) : sift_param_(sift_param), data_(nullptr), sift_filter_(nullptr)
    {
        // no op
    }
    
    Sift::Sift(Image& image) : data_(nullptr), sift_filter_(nullptr)
    {
        convert(image);
    }
    
    Sift::~Sift()
    {
        if (data_ != nullptr)
            { delete [] data_; }
        if(sift_filter_ != nullptr)
            { vl_sift_delete(sift_filter_); }
    }
    
    void Sift::clear()
    {
        if (data_ != nullptr)
        {
            delete [] data_;
            data_ = nullptr;
        }
        
        if (sift_filter_ != nullptr)
        {
            vl_sift_delete(sift_filter_);
            sift_filter_ = nullptr;
        }
    }
    
    int Sift::convert(const Image& image)
    {
        const int width_ = image.width();
        const int height_ = image.height();
        const int channel_ = image.channel();
        
        // the inner storage ORDER of VlPgmImage is exactly the same as that of our Image class
        data_ = new vl_sift_pix[width_ * height_];
        
        if (channel_ != 1)
        {
            Image gimage;
            gimage.rgb2grey(image);
            for (int i = 0; i < width_ * height_; ++i)
                { data_[i] = (vl_sift_pix)gimage.m_image[i]; }
        }
        else
        {
            for (int i = 0; i < width_ * height_; ++i)
                { data_[i] = (vl_sift_pix)image.m_image[i]; }
        }
        
        return 0;
    }
    
    void Sift::set_params(const SiftParam& r_params)
    {
        sift_param_ = r_params;
    }
    
    int Sift::detect_keypoints(const Image& image, vector<Keypoint> &keypoints, int verbose)
    {
        if (sift_filter_ == nullptr || (sift_filter_->width != image.width() ||
                                        sift_filter_->height != image.height()))
        {
            vl_sift_delete(sift_filter_);
            const int first_octave = get_valid_first_octave(sift_param_.first_octave_, image.width(), image.height());
            sift_filter_ = vl_sift_new(image.width(), image.height(),
                                       sift_param_.num_octaves_,
                                       sift_param_.num_levels_, first_octave);
            vl_sift_set_edge_thresh(sift_filter_, sift_param_.edge_thresh_);
            vl_sift_set_peak_thresh(sift_filter_, sift_param_.peak_thresh_);
        }
        
        if (data_ == nullptr)
            { convert(image); }
        
        int vl_status = vl_sift_process_first_octave(sift_filter_, data_);
        
        keypoints.reserve(2000);
        
        while (vl_status != VL_ERR_EOF)
        {
            vl_sift_detect(sift_filter_);
            
            const VlSiftKeypoint* vl_keypoints = vl_sift_get_keypoints(sift_filter_);
            int nkeys = vl_sift_get_nkeypoints(sift_filter_);
            
            for (int i = 0; i < nkeys; ++i)
            {
                double angles[4];
                int nangles = vl_sift_calc_keypoint_orientations(sift_filter_, angles, &vl_keypoints[i]);
                
                for (int j = 0; j < nangles; ++j)
                {
                    Keypoint keypoint(Vec2f(vl_keypoints[i].x + 1, vl_keypoints[i].y + 1), SIFT);
                    keypoint.scale() = vl_keypoints[i].sigma;
                    keypoint.orientation() = angles[j];
                    keypoints.push_back(keypoint);
                }
            }
            vl_status = vl_sift_process_next_octave(sift_filter_);
        }
        
        return 0;
    }
    
    int Sift::extract_descriptor(const Image& image, const Keypoint& keypoint, Vecf& descriptor)
    {
        if (!keypoint.has_scale() || !keypoint.has_orientation())
        {
            cerr << "keypoint must have scale and orientation to compute SIFT descriptor." << endl;
            return 1;
        }
        
        if (sift_filter_ == nullptr ||
            sift_filter_->width == image.width() ||
            sift_filter_->height == image.height())
        {
            vl_sift_delete(sift_filter_);
            const int first_octave = get_valid_first_octave(sift_param_.first_octave_, image.width(), image.height());
            sift_filter_ = vl_sift_new(image.width(), image.height(),
                                       sift_param_.num_octaves_,
                                       sift_param_.num_levels_, first_octave);
        }
        // sift keypoint
        VlSiftKeypoint sift_keypoint;
        vl_sift_keypoint_init(sift_filter_, &sift_keypoint, keypoint.coords()(0) - 1, keypoint.coords()(1) - 1, keypoint.scale());
        
        // process the first octave
        if (data_ == nullptr)
            { convert(image); }
        int vl_status = vl_sift_process_first_octave(sift_filter_, data_);
        while (sift_keypoint.o != sift_filter_->o_cur)
            { vl_status = vl_sift_process_next_octave(sift_filter_); }
        
        if (vl_status == VL_ERR_EOF)
        {
            cerr << "Cannot extract SIFT descriptor" << endl;
            return 1;
        }
        
        descriptor.resize(128);
        vl_sift_calc_keypoint_descriptor(sift_filter_, descriptor.data(), &sift_keypoint, keypoint.orientation());
        
        if(sift_param_.root_sift_)
        {
            convert_root_sift(descriptor);
        }
        
        return 0;
    }
    
    int Sift::extract_descriptors(const Image& image, vector<Keypoint>& keypoints, vector<Vecf>& descriptors)
    {
        if (sift_filter_ == nullptr ||
            sift_filter_->width == image.width() ||
            sift_filter_->height == image.height())
        {
            vl_sift_delete(sift_filter_);
            const int first_octave = get_valid_first_octave(sift_param_.first_octave_, image.width(), image.height());
            sift_filter_ = vl_sift_new(image.width(), image.height(),
                                       sift_param_.num_octaves_,
                                       sift_param_.num_levels_, first_octave);
        }
        // sift keypoints
        vector<VlSiftKeypoint> sift_keypoints(keypoints.size());
        for (int i = 0; i < keypoints.size(); ++i)
        {
            if (!keypoints[i].has_scale() || !keypoints[i].has_orientation())
            {
                cerr << "keypoint must have scale and orientation to compute SIFT descriptor." << endl;
                return 1;
            }
            vl_sift_keypoint_init(sift_filter_, &sift_keypoints[i], keypoints[i].coords()(0) - 1, keypoints[i].coords()(1) - 1, keypoints[i].scale());
        }
        
        // process the first octave
        if (data_ == nullptr)
            { convert(image); }
        int vl_status = vl_sift_process_first_octave(sift_filter_, data_);
        
        descriptors.resize(keypoints.size(), Vecf(128));
        while (vl_status != VL_ERR_EOF)
        {
            for (int i = 0; i < sift_keypoints.size(); ++i)
            {
                if (sift_keypoints[i].o != sift_filter_->o_cur)
                    { continue; }
                vl_sift_calc_keypoint_descriptor(sift_filter_, descriptors[i].data(), &sift_keypoints[i], keypoints[i].orientation());
            }
            vl_status = vl_sift_process_next_octave(sift_filter_);
        }
        
        if (sift_param_.root_sift_)
        {
            for (int i = 0; i < descriptors.size(); ++i)
            {
                convert_root_sift(descriptors[i]);
            }
        }
        
        return 0;
    }
    
    double Sift::get_valid_first_octave(const int first_octave, const int width, const int height)
    {
        const int max_dim = std::max(width, height);
        int valid_first_octave = first_octave;
        double scale_factor = std::pow(2.0, -1 * valid_first_octave);
        while (max_dim * scale_factor >= kMaXScaledDim)
        {
            scale_factor /= 2.0;
            ++valid_first_octave;
        }
        return valid_first_octave;
    }
    
    // convert to RootSIFT descriptor, which is proven to provide better matches
    // "Three things everyone should know to improve object retrieval" by
    // Arandjelovic and Zisserman.
    void Sift::convert_root_sift(Vecf &descriptor)
    {
        const double tol = 1e-8;
        const double l1_norm = descriptor.lpNorm<1>();
        if (l1_norm > tol)
        {
            descriptor /= l1_norm;
            descriptor = descriptor.array().sqrt();
        }
    }
    
    // ----------------------------------------------------- methods not used
    void Sift::transpose_descriptor(vl_sift_pix* dst, vl_sift_pix* src)
    {
        int const BO = 8;  /* number of orientation bins */
        int const BP = 4;  /* number of spatial bins     */
        int i, j, t;
        
        for (j = 0; j < BP; ++j) {
            int jp = BP - 1 - j;
            for (i = 0; i < BP; ++i) {
                int o  = BO * i + BP*BO * j;
                int op = BO * i + BP*BO * jp;
                dst [op] = src[o];
                for (t = 1; t < BO; ++t)
                    dst [BO - t + op] = src [t + o];
            }
        }
    }
    
    bool Sift::ksort(const Keypoint &a, const Keypoint &b)
    {
        return a.scale() < b.scale();
    }
    
    vl_bool Sift::check_sorted(const vector<Keypoint> &keys, vl_size nkeys)
    {
        vl_uindex k;
        for (k = 0; k + 1 < nkeys; ++k)
        {
            if (ksort(keys[k], keys[k + 1]))
                return VL_FALSE;
            k++;
        }
        return VL_TRUE;
    }
    
    // still has some bugs
    // check out the original source code in 'toolbox/sift/vl_sift'
    int Sift::detect_keypoints_unfinished(const Image &image, vector<Keypoint> &keypoints, int verbose)
    {
        
        // sift setting
        int O = 3;
        int S = 3;
        int o_min = 0;
        double edge_thresh = 10;
        double peak_thresh = 0;
        double norm_thresh = -INFINITY;
        double magnif = 3;
        double window_size = 2;
        
        vl_bool force_orientations = 1;
        vl_bool float_descriptors = 0;
        vl_bool compute_descriptors = 0;
        
        vector<Keypoint> ikeys;
        ikeys.assign(keypoints.begin(), keypoints.end());
        int nikeys = static_cast<int>(ikeys.size());
        if (nikeys != 0)
        {
            if (check_sorted(keypoints, nikeys))
                std::sort(keypoints.begin(), keypoints.end(), ksort);
        }
        keypoints.clear();
        
        /* -----------------------------------------------------------------
         *                                                            Do job
         * -------------------------------------------------------------- */
        {
        VlSiftFilt        *filt;
        vl_bool            first;
        int                nframe = 0, i, q; // nframes: number of features
        
        /* create a filter to process the image */
        filt = vl_sift_new (width_, height_, O, S, o_min);
        
        if (peak_thresh >= 0) vl_sift_set_peak_thresh (filt, peak_thresh);
        if (edge_thresh >= 0) vl_sift_set_edge_thresh (filt, edge_thresh);
        if (norm_thresh >= 0) vl_sift_set_norm_thresh (filt, norm_thresh);
        if (magnif      >= 0) vl_sift_set_magnif      (filt, magnif);
        if (window_size >= 0) vl_sift_set_window_size (filt, window_size);
        
        if (verbose) {
            cout << "vl_sift: filter settings:\n" << endl;
            cout << "vl_sift:   octaves      (O)      = "
                 << vl_sift_get_noctaves(filt) << endl;
            cout << "vl_sift:   levels       (S)      = "
                 << vl_sift_get_nlevels(filt) << endl;
            cout << "vl_sift:   first octave (o_min)  = "
                 << vl_sift_get_octave_first(filt) << endl;
            cout << "vl_sift:   edge thresh           = "
                 << vl_sift_get_edge_thresh(filt) << endl;
            cout << "vl_sift:   peak thresh           = "
                 << vl_sift_get_peak_thresh(filt) << endl;
            cout << "vl_sift:   norm thresh           = "
                 << vl_sift_get_norm_thresh(filt) << endl;
            cout << "vl_sift:   window size           = "
                 << vl_sift_get_window_size(filt) << endl;
            cout << "vl_sift:   float descriptor      = "
                 << float_descriptors << endl;
            
            printf((nikeys >= 0) ?
                   "vl_sift: will source frames? yes (%d read)\n" :
                   "vl_sift: will source frames? no\n", nikeys) ;
            cout << "vl_sift: will force orientations? "
                 << (force_orientations ? "yes" : "no") << endl;
        }
        
        /* ...............................................................
         *                                             Process each octave
         * ............................................................ */
        i     = 0;
        first = 1;
        while (1) {
            int                   err;
            VlSiftKeypoint const *keys  = 0;
            int                   nkeys = 0;
            
            if (verbose)
            {
                cout << "vl_sift: processing octave %d\n"
                     << vl_sift_get_octave_index (filt) << endl;
            }
            
            /* Calculate the GSS for the next octave .................... */
            if (first) {
                err   = vl_sift_process_first_octave (filt, data_);
                first = 0;
            } else {
                err   = vl_sift_process_next_octave  (filt);
            }
            
            if (err) break;
            
            if (verbose > 1)
            {
                cout << "vl_sift: GSS octave %d computed\n"
                     << vl_sift_get_octave_index(filt) << endl;
            }
            
            /* Run detector ............................................. */
            if (nikeys <= 0) {
                vl_sift_detect (filt);
                
                keys  = vl_sift_get_keypoints  (filt);
                nkeys = vl_sift_get_nkeypoints (filt);
                i     = 0;
                
                if (verbose > 1) {
                    printf ("vl_sift: detected %d (unoriented) keypoints\n", nkeys);
                }
            } else {
                nkeys = nikeys;
            }
            
            /* For each keypoint ........................................ */
            for (; i < nkeys; ++i) {
                double                angles [4];
                int                   nangles;
                VlSiftKeypoint        ik;
                VlSiftKeypoint const *k;
                
                /* Obtain keypoint orientations ........................... */
                if (nikeys <= 0)
                {
                    k = keys + i;
                    nangles = vl_sift_calc_keypoint_orientations(filt, angles, k);
                }
                else
                {
                    vl_sift_keypoint_init (filt, &ik,
                                           ikeys[i].coords()(1) - 1,
                                           ikeys[i].coords()(2) - 1,
                                           ikeys[i].scale());
                    
                    if (ik.o != vl_sift_get_octave_index (filt)) {
                        break;
                    }
                    
                    k = &ik;
                    
                    /* optionally compute orientations too */
                    if (force_orientations) {
                        nangles = vl_sift_calc_keypoint_orientations(filt, angles, k);
                    } else {
                        angles [0] = M_PI / 2 - keypoints[i].orientation();
                        nangles    = 1;
                    }
                }
                // Q: how come multiple orientation?
                // A: the gradient histogram is smoothed and the maximum is selected. In addition,
                //    up to other three modes whose amplitude is within the 80% of the biggest mode
                //    are retained and returned as additional orientations.
                /* For each orientation ................................... */
                for (q = 0; q < nangles; ++q) {
                    vl_sift_pix  buf [128];
                    vl_sift_pix rbuf [128];
                    
                    /* compute descriptor (if necessary) */
                    if (!compute_descriptors) {
                        vl_sift_calc_keypoint_descriptor(filt, buf, k, angles[q]);
                        transpose_descriptor (rbuf, buf);
                    }
                    
                    KeypointType type = SIFT;
                    Keypoint keypoint(Vec2f(k->x + 1, k->y + 1), type);
                    keypoint.scale() = (float)k->sigma;
                    keypoint.orientation() = M_PI / 2 - angles [q];
                    keypoints.push_back(keypoint);
                    
                    ++nframe;
                } /* next orientation */
            } /* next keypoint */
        } /* next octave */
        
        if (verbose)
            { cout << "vl_sift: found " << keypoints.size() << " keypoints."  << endl; }
        
        /* cleanup */
        vl_sift_delete (filt);
        ikeys.clear();
        
        }/* end: do job */
        
        return 0;
    }
    
}
