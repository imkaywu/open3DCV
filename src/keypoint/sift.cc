#include "sift.h"
#define _USE_MATH_DEFINES
extern "C" {
#include <math.h>
}

using std::cout;
using std::endl;

namespace open3DCV {
    
    int Sift::convert(Image& img)
    {
        const int width_ = img.width();
        const int height_ = img.height();
        const int channel_ = img.channel();
        
        // the inner storage ORDER of VlPgmImage is exactly the same as that of our Image class
        data_ = (vl_sift_pix*)malloc(width_ * height_ * channel_ * sizeof(vl_sift_pix));
        
        if (channel_ != 1)
        {
            if (img.m_gimage.empty())
                { img.rgb2grey(); }
            for (int i = 0; i < width_ * height_; ++i)
                { data_[i] = img.m_gimage[i]; }
        }
        else
        {
            for (int i = 0; i < width_ * height_; ++i)
                { data_[i] = img.m_image[i]; }
        }
        
        return 0;
    }
    
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
    
    int Sift::detect()
    {
        int verbose = 0;
        // sift setting
        int O = -1;
        int S = 3;
        int o_min = 0;
        double edge_thresh = -1;
        double peak_thresh = -1;
        double norm_thresh = -1;
        double magnif = -1;
        double window_size = -1;
        
        Mat4X ikeys_array;
        double *ikeys = 0;
        int nikeys = -1;
        vl_bool force_orientations = 0;
        vl_bool float_descriptors = 0;
        vl_bool compute_descriptors = 0;
        
        /* -----------------------------------------------------------------
         *                                                            Do job
         * -------------------------------------------------------------- */
        
        VlSiftFilt        *filt;
        vl_bool            first;
        double            *frames = 0;
        void              *descr  = 0;
        int                nframes = 0, reserved = 0, i,j,q; // nframes: number of keypoints
        
        /* create a filter to process the image */
        filt = vl_sift_new (width_, height_, O, S, o_min);
        
        if (peak_thresh >= 0) vl_sift_set_peak_thresh (filt, peak_thresh);
        if (edge_thresh >= 0) vl_sift_set_edge_thresh (filt, edge_thresh);
        if (norm_thresh >= 0) vl_sift_set_norm_thresh (filt, norm_thresh);
        if (magnif      >= 0) vl_sift_set_magnif      (filt, magnif);
        if (window_size >= 0) vl_sift_set_window_size (filt, window_size);
        /*
        if (verbose) {
            cout << "vl_sift: filter settings:\n" << endl;
            cout << "vl_sift:   octaves      (O)      = %d\n"
                 << vl_sift_get_noctaves(filt) << endl;
            mexPrintf("vl_sift:   levels       (S)      = %d\n",
                      vl_sift_get_nlevels       (filt));
            mexPrintf("vl_sift:   first octave (o_min)  = %d\n",
                      vl_sift_get_octave_first  (filt));
            mexPrintf("vl_sift:   edge thresh           = %g\n",
                      vl_sift_get_edge_thresh   (filt));
            mexPrintf("vl_sift:   peak thresh           = %g\n",
                      vl_sift_get_peak_thresh   (filt));
            mexPrintf("vl_sift:   norm thresh           = %g\n",
                      vl_sift_get_norm_thresh   (filt));
            mexPrintf("vl_sift:   window size           = %g\n",
                      vl_sift_get_window_size   (filt));
            mexPrintf("vl_sift:   float descriptor      = %d\n",
                      floatDescriptors);
            
            mexPrintf((nikeys >= 0) ?
                      "vl_sift: will source frames? yes (%d read)\n" :
                      "vl_sift: will source frames? no\n", nikeys);
            mexPrintf("vl_sift: will force orientations? %s\n",
                      force_orientations ? "yes" : "no");
        }
        */
        
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
            if (nikeys < 0) {
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
                if (nikeys >= 0) {
                    vl_sift_keypoint_init (filt, &ik,
                                           ikeys [4 * i + 1] - 1,
                                           ikeys [4 * i + 0] - 1,
                                           ikeys [4 * i + 2]);
                    
                    if (ik.o != vl_sift_get_octave_index (filt)) {
                        break;
                    }
                    
                    k = &ik;
                    
                    /* optionally compute orientations too */
                    if (force_orientations) {
                        nangles = vl_sift_calc_keypoint_orientations
                        (filt, angles, k);
                    } else {
                        angles [0] = M_PI / 2 - ikeys [4 * i + 3];
                        nangles    = 1;
                    }
                } else {
                    k = keys + i;
                    nangles = vl_sift_calc_keypoint_orientations
                    (filt, angles, k);
                }
                
                /* For each orientation ................................... */
                for (q = 0; q < nangles; ++q) {
                    vl_sift_pix  buf [128];
                    vl_sift_pix rbuf [128];
                    
                    /* compute descriptor (if necessary) */
                    if (!compute_descriptors) {
                        vl_sift_calc_keypoint_descriptor (filt, buf, k, angles [q]);
                        transpose_descriptor (rbuf, buf);
                    }
                    
                    /* make enough room for all these keypoints and more */
                    if (reserved < nframes + 1) {
                        reserved += 2 * nkeys;
                        //frames = mxRealloc (frames, 4 * sizeof(double) * reserved);
                        frames = new double[4 * reserved];
                        if (!compute_descriptors) {
                            if (! float_descriptors) {
                                //descr  = mxRealloc (descr,  128 * sizeof(vl_uint8) * reserved);
                                descr = new vl_uint8[128 * reserved];
                            } else {
                                //descr  = mxRealloc (descr,  128 * sizeof(float) * reserved);
                                descr = new float[128 * reserved];
                            }
                        }
                    }
                    
                    /* Save back with MATLAB conventions. Notice tha the input
                     * image was the transpose of the actual image. */
                    frames [4 * nframes + 0] = k -> y + 1;
                    frames [4 * nframes + 1] = k -> x + 1;
                    frames [4 * nframes + 2] = k -> sigma;
                    frames [4 * nframes + 3] = M_PI / 2 - angles [q];
                    
                    if (!compute_descriptors) {
                        if (! float_descriptors) {
                            for (j = 0; j < 128; ++j) {
                                float x = 512.0F * rbuf [j];
                                x = (x < 255.0F) ? x : 255.0F;
                                ((vl_uint8*)descr) [128 * nframes + j] = (vl_uint8) x;
                            }
                        } else {
                            for (j = 0; j < 128; ++j) {
                                float x = 512.0F * rbuf [j];
                                ((float*)descr) [128 * nframes + j] = x;
                            }
                        }
                    }
                    
                    ++nframes;
                } /* next orientation */
            } /* next keypoint */
        } /* next octave */
        
        if (verbose)
            { cout << "vl_sift: found %d keypoints\n" << nframes << endl; }
        
        /* ...............................................................
         *                                                       Save back
         * ............................................................ */
        
        {
            keys_.resize(4, nframes);
            descr_.resize(128, nframes);
            for (int c = 0; c < nframes; ++c)
            {
                for (int r = 0; r < 4; ++r)
                {
                    keys_(r, c) = frames[4 * c + r];
                }
                if (!compute_descriptors)
                {
                    for (int r = 0; r < 128; ++r)
                    {
                        descr_(r, c) = ((float*)descr) [128 * c + r];
                    }
                }
                
            }
            /*
            int dims [2];
            
            // create an empty array
            dims [0] = 0;
            dims [1] = 0;
            out[OUT_FRAMES] = mxCreateNumericArray
            (2, dims, mxDOUBLE_CLASS, mxREAL);
            
            // set array content to be the frames buffer
            dims [0] = 4;
            dims [1] = nframes;
            mxSetPr         (out[OUT_FRAMES], frames);
            mxSetDimensions (out[OUT_FRAMES], dims, 2);
            
            if (!compute_descriptors) {
                
                // create an empty array
                dims [0] = 0;
                dims [1] = 0;
                out[OUT_DESCRIPTORS]= mxCreateNumericArray
                (2, dims,
                 floatDescriptors ? mxSINGLE_CLASS : mxUINT8_CLASS,
                 mxREAL);
                
                // set array content to be the descriptors buffer
                dims [0] = 128;
                dims [1] = nframes;
                mxSetData       (out[OUT_DESCRIPTORS], descr);
                mxSetDimensions (out[OUT_DESCRIPTORS], dims, 2);
            }
            */
        }
        
        /* cleanup */
        vl_sift_delete (filt);
        /*
        if (ikeys_array)
            mxDestroyArray(ikeys_array);
        */
        ikeys_array.resize(0, 0);
        /* end: do job */
        
        return 0;
    }
    
    int Sift::descript()
    {
        
        return 0;
    }
}
