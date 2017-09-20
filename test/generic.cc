extern "C" {
#include <stdlib.h>
#include <stdio.h>

#include "vl/generic.h"
#include "vl/pgm.h"
#include "vl/sift.h"
}

#include <iostream>
#include "image.h"


int main(int argc, const char * argv[]) {
    std::string fname = "/Users/BlacKay/Documents/Projects/Images/test/mandrill.pgm";
    FILE *in = 0;
    vl_uint8 *data = 0;
    vl_sift_pix *fdata = 0;
    VlPgmImage pim;
    
    vl_bool  err    = VL_ERR_OK ;
    char err_msg[1024];
    int verbose;
    
    in = fopen(fname.c_str(), "rb");
    
    // read PGM header
    err = vl_pgm_extract_head (in, &pim);
    
    
    if (err) {
        switch (vl_get_last_error()) {
            case  VL_ERR_PGM_IO :
                snprintf(err_msg, sizeof(err_msg),
                         "Cannot read from '%s'.", fname.c_str()) ;
                err = VL_ERR_IO ;
                break ;
                
            case VL_ERR_PGM_INV_HEAD :
                snprintf(err_msg, sizeof(err_msg),
                         "'%s' contains a malformed PGM header.", fname.c_str()) ;
                err = VL_ERR_IO ;
                //goto done ;
        }
    }
    
    if (verbose)
        printf ("sift: image is %" VL_FMT_SIZE " by %" VL_FMT_SIZE " pixels\n",
                pim. width,
                pim. height) ;
    
    // allocate buffer
    data  = (vl_uint8*)malloc(vl_pgm_get_npixels (&pim) *
                   vl_pgm_get_bpp       (&pim) * sizeof (vl_uint8)   ) ;
    fdata = (vl_sift_pix*)malloc(vl_pgm_get_npixels (&pim) *
                   vl_pgm_get_bpp       (&pim) * sizeof (vl_sift_pix)) ;
    std::cout << "number of pixel:" << vl_pgm_get_npixels(&pim) << std::endl;
    std::cout << "types per pixel:" << vl_pgm_get_bpp(&pim) << std::endl;
    
    if (!data || !fdata) {
        err = VL_ERR_ALLOC ;
        snprintf(err_msg, sizeof(err_msg),
                 "Could not allocate enough memory.");
        //goto done;
    }
    
    // read PGM body
    err  = vl_pgm_extract_data (in, &pim, data) ;
    
    if (err) {
        snprintf(err_msg, sizeof(err_msg), "PGM body malformed.") ;
        err = VL_ERR_IO ;
        //goto done ;
    }
    
    open3DCV::Image img;
    img.init(fname);
    vector<unsigned char> image;
    int w, h;
    img.readPGMImage(fname, image, w, h);
    
    std::cout << "Start: VlPgmImage" << std::endl;
    for (int i = 0; i < 100; ++i)
        std::cout << (int)data[i] << " " << (int)image[i] << std::endl;
    std::cout << "End: VlPgmImage" << std::endl;
    
}
