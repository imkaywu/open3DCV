#include "image.hpp"
#include <fstream>

#define cimg_display 0          // don't have X11 installed or don't want display capabilities of CImg
#if defined(PMVS_HAVE_PNG)		// See CMakeLists.txt
#	define cimg_use_png
#endif
#define cimg_use_jpeg
#if defined(PMVS_HAVE_TIFF)		// See CMakeLists.txt
#	define cimg_use_tiff
#endif
#include "CImg.h"

using namespace cimg_library;
using cimg_library::CImg;
using std::ifstream;
using std::ofstream;
using Eigen::Matrix4f;

namespace open3DCV {

Image::Image() {
    m_alloc = 0;
}

Image::~Image() {
}

void Image::init(const string name) {
    m_alloc = 0;
    
    if (!name.empty()) {
        completeName(name, m_iname, 1);
    }
}

void Image::completeName(const string& lhs, string& rhs, const int isColor) {
    if (5 <= lhs.length() && lhs[lhs.length() - 4] == '.') {
        rhs = lhs;
        return;
    }
    
    // support ppm jpg
    if (isColor) {
        string stmp0 = lhs + ".ppm";    string stmp1 = lhs + ".jpg";
        string stmp2 = lhs + ".png";    string stmp3 = lhs + ".tiff";
        
        if (ifstream(stmp0.c_str()))
            rhs = stmp0;
        else if (ifstream(stmp1.c_str()))
            rhs = stmp1;
#if defined(PMMVPS_HAVE_PNG)
        else if (ifstream(stmp2.c_str()))
            rhs = stmp2;
#endif
#if defined(PMMVPS_HAVE_TIFF)
        else if (ifstream(stmp3.c_str()))
            rhs = stmp3;
#endif
        else
            rhs = lhs;
    }
    // support pgm pbm
    else {
        string stmp0 = lhs + ".pgm";    string stmp1 = lhs + ".pbm";
        
        if (ifstream(stmp0.c_str()))
            rhs = stmp0;
        else if (ifstream(stmp1.c_str()))
            rhs = stmp1;
        else
            rhs = lhs;
    }
}

void Image::free() {
    if(m_alloc != 0)
        m_alloc = 1;
    else
        m_alloc = 0;
        vector<unsigned char>().swap(m_image);
}

Vector3f Image::getColor(const int ix, const int iy) const {
#ifdef PMMVPS_DEBUG
    if (m_alloc != 2) {
        cerr << "First allocate" << std::endl;
        exit (1);
    }
#endif
    const int index = (iy * m_width + ix) * 3;
    
    return Vector3f(m_image[index],
                    m_image[index+1],
                    m_image[index+2]);
}

Vector3f Image::getColor(const float x, const float y) const {
#ifdef PMMVPS_DEBUG
    if (m_alloc != 2) {
        std::cerr << "First allocate" << std::endl;
        exit (1);
    }
#endif
    
#ifdef PMMVPS_IMAGE_BICUBIC  // Bicubic case
    const int x1 = (int)floor(x);      const int y1 = (int)floor(y);
    const float p = x - x1;      const float q = y - y1;
    
    float f = 1+p;
    const float wx0 = (((-1) * f + 5) * f - 8) * f + 4;
    f = 2-p;
    const float wx3 = (((-1) * f + 5) * f - 8) * f + 4;
    f = p;
    const float wx1 = (((1) * f - 2) * f) * f + 1;
    f = 1 - p;
    const float wx2 = (((1) * f - 2) * f) * f + 1;
    
    f = 1+q;
    const float wy0 = (((-1) * f + 5) * f - 8) * f + 4;
    f = 2-q;
    const float wy3 = (((-1) * f + 5) * f - 8) * f + 4;
    f = q;
    const float wy1 = (((1) * f - 2) * f) * f + 1;
    f = 1 - q;
    const float wy2 = (((1) * f - 2) * f) * f + 1;
    
    const int offset = m_widths[level] * 3;
    const int index0 = ((y1 - 1) * m_widths[level] + x1 - 1) * 3;
    const int index1 = index0 + offset;
    const int index2 = index1 + offset;
    const int index3 = index2 + offset;
    
#ifdef PMMVPS_IMAGE_GAMMA
    // to be written
#else
    const unsigned char& r00 = m_images[level][index0];
    const unsigned char& g00 = m_images[level][index0+1];
    const unsigned char& b00 = m_images[level][index0+2];
    const unsigned char& r01 = m_images[level][index0+3];
    const unsigned char& g01 = m_images[level][index0+4];
    const unsigned char& b01 = m_images[level][index0+5];
    const unsigned char& r02 = m_images[level][index0+6];
    const unsigned char& g02 = m_images[level][index0+7];
    const unsigned char& b02 = m_images[level][index0+8];
    const unsigned char& r03 = m_images[level][index0+9];
    const unsigned char& g03 = m_images[level][index0+10];
    const unsigned char& b03 = m_images[level][index0+11];
    
    const unsigned char& r10 = m_images[level][index1];
    const unsigned char& g10 = m_images[level][index1+1];
    const unsigned char& b10 = m_images[level][index1+2];
    const unsigned char& r11 = m_images[level][index1+3];
    const unsigned char& g11 = m_images[level][index1+4];
    const unsigned char& b11 = m_images[level][index1+5];
    const unsigned char& r12 = m_images[level][index1+6];
    const unsigned char& g12 = m_images[level][index1+7];
    const unsigned char& b12 = m_images[level][index1+8];
    const unsigned char& r13 = m_images[level][index1+9];
    const unsigned char& g13 = m_images[level][index1+10];
    const unsigned char& b13 = m_images[level][index1+11];
    
    const unsigned char& r20 = m_images[level][index2];
    const unsigned char& g20 = m_images[level][index2+1];
    const unsigned char& b20 = m_images[level][index2+2];
    const unsigned char& r21 = m_images[level][index2+3];
    const unsigned char& g21 = m_images[level][index2+4];
    const unsigned char& b21 = m_images[level][index2+5];
    const unsigned char& r22 = m_images[level][index2+6];
    const unsigned char& g22 = m_images[level][index2+7];
    const unsigned char& b22 = m_images[level][index2+8];
    const unsigned char& r23 = m_images[level][index2+9];
    const unsigned char& g23 = m_images[level][index2+10];
    const unsigned char& b23 = m_images[level][index2+11];
    
    const unsigned char& r30 = m_images[level][index3];
    const unsigned char& g30 = m_images[level][index3+1];
    const unsigned char& b30 = m_images[level][index3+2];
    const unsigned char& r31 = m_images[level][index3+3];
    const unsigned char& g31 = m_images[level][index3+4];
    const unsigned char& b31 = m_images[level][index3+5];
    const unsigned char& r32 = m_images[level][index3+6];
    const unsigned char& g32 = m_images[level][index3+7];
    const unsigned char& b32 = m_images[level][index3+8];
    const unsigned char& r33 = m_images[level][index3+9];
    const unsigned char& g33 = m_images[level][index3+10];
    const unsigned char& b33 = m_images[level][index3+11];
#endif // PMMVPS_IMAGE_GAMMA
    // separate x and y
    const float row0[3] = {wx0 * r00 + wx1 * r01 + wx2 * r02 + wx3 * r03,
        wx0 * g00 + wx1 * g01 + wx2 * g02 + wx3 * g03,
        wx0 * b00 + wx1 * b01 + wx2 * b02 + wx3 * b03};
    const float row1[3] = {wx0 * r10 + wx1 * r11 + wx2 * r12 + wx3 * r13,
        wx0 * g10 + wx1 * g11 + wx2 * g12 + wx3 * g13,
        wx0 * b10 + wx1 * b11 + wx2 * b12 + wx3 * b13};
    const float row2[3] = {wx0 * r20 + wx1 * r21 + wx2 * r22 + wx3 * r23,
        wx0 * g20 + wx1 * g21 + wx2 * g22 + wx3 * g23,
        wx0 * b20 + wx1 * b21 + wx2 * b22 + wx3 * b23};
    const float row3[3] = {wx0 * r30 + wx1 * r31 + wx2 * r32 + wx3 * r33,
        wx0 * g30 + wx1 * g31 + wx2 * g32 + wx3 * g33,
        wx0 * b30 + wx1 * b31 + wx2 * b32 + wx3 * b33};
    
    float r = wy0 * row0[0] + wy1 * row1[0] + wy2 * row2[0] + wy3 * row3[0];
    float g = wy0 * row0[1] + wy1 * row1[1] + wy2 * row2[1] + wy3 * row3[1];
    float b = wy0 * row0[2] + wy1 * row1[2] + wy2 * row2[2] + wy3 * row3[2];
    
    return Vector3f(r, g, b);
#else // Bilinear case
    const int lx = static_cast<int>(x);
    const int ly = static_cast<int>(y);
    const int index = 3 * (ly * m_width + lx);
    
    const float dx1 = x - lx;  const float dx0 = 1.0f - dx1;
    const float dy1 = y - ly;  const float dy0 = 1.0f - dy1;
    
    const float f00 = dx0 * dy0;  const float f01 = dx0 * dy1;
    const float f10 = dx1 * dy0;  const float f11 = dx1 * dy1;
    const int index2 = index + 3 * m_width;
    
#ifdef PMMVPS_IMAGE_GAMMA
    // to be written
#else
    const unsigned char* ucp0 = &m_image[index] - 1;
    const unsigned char* ucp1 = &m_image[index2] - 1;
    float r = 0.0f;  float g = 0.0f;  float b = 0.0f;
    r += *(++ucp0) * f00 + *(++ucp1) * f01;
    g += *(++ucp0) * f00 + *(++ucp1) * f01;
    b += *(++ucp0) * f00 + *(++ucp1) * f01;
    r += *(++ucp0) * f10 + *(++ucp1) * f11;
    g += *(++ucp0) * f10 + *(++ucp1) * f11;
    b += *(++ucp0) * f10 + *(++ucp1) * f11;
    return Vector3f(r, g, b);
#endif // PMMVPS_IMAGE_GAMMA
    
#endif // PMMVPS_IMAGE_BICUBIC
}

int Image::getWidth() const {
    return m_width;
};

int Image::getHeight() const {
    return m_height;
};

//-----------------------------------------------
// any image
//-----------------------------------------------
int Image::readAnyImage(const string file,
                        vector<unsigned char>& image,
                        int& width, int& height,
                        int& channel)
{
    CImg<unsigned char> cimage;
    try
    {
        cimage.load(file.c_str());
        if (!cimage.is_empty())
        {
            if (cimage.spectrum() != 1 && cimage.spectrum() != 3)
            {
                cerr << "Cannot handle the components. Component num is " << cimage.spectrum() << endl;
                return 1;
            }
            width = cimage.width();
            height = cimage.height();
            channel = cimage.spectrum();
            
            // CImg holds the image internally in a different order, so we need to reorder it here
            image.resize(cimage.size());
            int i = 0;
            for (int y = 0; y < height; ++y)
            {
                for (int x = 0; x < width; ++x)
                {
                    for (int c = 0; c < channel; ++c)
                    {
                        image[i] = cimage(x, y, 0, c);
                        i++;
                    }
                }
            }
        }
    }
    catch (cimg_library::CImgException& e)
    {
        cerr << "Cannot read image " << file << endl;
        return 1;
    }
    return 0;
}

//-----------------------------------------------
// PBM
//-----------------------------------------------
int Image::readPBMImage(const string file,
                        vector<unsigned char>& image,
                        int& width, int& height)
{
    
    if (file.substr(file.length() - 3, file.length()) != "pbm")
    {
        return 1;
    }
    
    ifstream ifstr;
    ifstr.open(file.c_str());
    if (!ifstr.is_open())
    {
        return 1;
    }
    string header;
    unsigned char uctmp;
    
    ifstr >> header;
    ifstr.read((char*)&uctmp, sizeof(unsigned char));
    
    if (header != "P4")
    {
        cerr << "Only accept binary pbm format: " << file << endl;
        return 1;
    }
    
    while (1)
    {
        ifstr.read((char*)&uctmp, sizeof(unsigned char));
        ifstr.putback(uctmp);
        if (uctmp == '#')
        {
            char buffer[1024];
            ifstr.getline(buffer, 1024);
        }
        else
        {
            break;
        }
    }
    ifstr >> width >> height;
    ifstr.read((char*)&uctmp, sizeof(unsigned char));
    
    image.clear();
    
    int reso = width * height;
    if (reso % 8 != 0)
    {
        reso++;
    }
    
    int count = 0;
    for (int i = 0; i < reso; ++i)
    {
        ifstr.read((char*)&uctmp, sizeof(unsigned char));
        for (int j = 0; j < 8; ++j)
        {
            if (uctmp >> 7)
            {
                image.push_back((unsigned char)0);
            }
            else
            {
                image.push_back((unsigned char)255);
            }
            count++;
            uctmp <<= 1;
            if (count == width * height)
            {
                break;
            }
        }
    }
    ifstr.close();
    return 0;
}

int Image::writePBMImage(const string file,
                         vector<unsigned char>& image,
                         const int width, const int height)
{
    ofstream ofstr;
    ofstr.open(file.c_str());
    if(!ofstr.is_open())
    {
        cerr << "Cannot write to a file" << file << endl;
        return 1;
    }
    ofstr << "P4" << endl << width << ' ' << height << endl;
    
    unsigned char uctmp = 0;
    for (int i = 0; i < width * height; ++i)
    {
        uctmp <<= 1;
        if(image[i] < 127)
            uctmp |= 0x0001;
        else
            uctmp &= 0x0001;
        
        if (i % 8 == 7)
            ofstr.write((char*)&uctmp, sizeof(char));
    }
    const int itmp = (width * height) % 8;
    if (itmp != 0)
    {
        uctmp <<= 8 - itmp;
        ofstr.write((char*)&uctmp, sizeof(char));
    }
    
    ofstr.close();
    return 0;
}

//-----------------------------------------------
// PGM
//-----------------------------------------------
int Image::readPGMImage(const string file,
                        vector<unsigned char>& image,
                        int& width, int& height)
{
    if (file.substr(file.length() - 3, file.length()) != "pgm")
    {
        return 1;
    }
    
    ifstream ifstr;
    ifstr.open(file.c_str());
    if (!ifstr.is_open())
    {
        cerr << "Cannot open a file: " << file << endl;
        return 1;
    }
    
    string header;
    unsigned char uctmp;
    
    ifstr >> header;
    ifstr.read((char*)&uctmp, sizeof(unsigned char));
    if (header != "P5")
    {
        cerr << "Only accept binary pgm format" << file << " " << header << endl;
        return 1;
    }
    
    while (1)
    {
        ifstr.read((char*)&uctmp, sizeof(unsigned char));
        ifstr.putback(uctmp);
        if (uctmp == '#')
        {
            char buffer[1024];
            ifstr.getline(buffer, 1024);
        }
        else
        {
            break;
        }
    }
    int itmp;
    ifstr >> width >> height >> itmp;
    ifstr.read((char*)&uctmp, sizeof(unsigned char));
    
    image.clear();
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            ifstr.read((char*)&uctmp, sizeof(unsigned char));
            image.push_back(uctmp);
        }
    }
    ifstr.close();
    
    return 0;
}

int Image::writePGMImage(const string file,
                         const vector<unsigned char>& image,
                         const int width, const int height)
{
    std::ofstream ofstr;
    ofstr.open(file.c_str());
    if (!ofstr.is_open()) {
        cerr << "Cannot write to a file: " << file << endl;
        return 1;
    }
    
    ofstr << "P5" << endl
          << width << ' ' << height << endl
          << 255 << endl;
    
    for (int i = 0; i < width * height; ++i) {
        unsigned char uctmp = image[i];
        ofstr.write((char*)&uctmp, sizeof(unsigned char));
    }
    
    ofstr.close();
    return 0;
}

//----------------------------------------------------------------------
// PPM
//----------------------------------------------------------------------
int Image::readPPMImage(const string file,
                        vector<unsigned char>& image,
                        int& width, int& height)
{
    if (file.substr(file.length() - 3, file.length()) != "ppm")
    {
        return 1;
    }
    
    CImg<unsigned char> cimage;
    try
    {
        cimage.load_pnm(file.c_str());
        if(!cimage.is_empty())
        {
            if(cimage.spectrum() != 1 && cimage.spectrum() != 3)
            {
                cerr << "Cannot handle this component. Component num is" << cimage.spectrum() << endl;
                return 1;
            }
            width = cimage.width();
            height = cimage.height();
            image.resize(cimage.size());
            
            int i = 0;
            for (int y = 0; y < height; ++y)
            {
                for (int x = 0; x < width; ++x)
                {
                    for (int c = 0; c < cimage.spectrum(); ++c)
                    {
                        image[i] = cimage(x, y, 0, c);
                        ++i;
                    }
                }
            }
        }
    }
    catch(cimg_library::CImgException &e)
    {
        cerr << "Cannot read image " << file << endl;
        return 1;
    }
    return 0;
}

int Image::writePPMImage(const string file,
                         const vector<unsigned char>& image,
                         const int width, const int height)
{
    ofstream ofstr;
    ofstr.open(file.c_str());
    if(!ofstr.is_open())
    {
        cerr << "Cannot write to a file: " << file << endl;
        return 1;
    }
    ofstr << "P6" << endl << width << ' ' << height << endl << 255 << endl;
    
    for (int i = 0; i < 3 * width * height; ++i)
    {
        unsigned char uctmp = image[i];
        ofstr.write((char*)&uctmp, sizeof(unsigned char));
    }
    ofstr.close();
    return 0;
}

//----------------------------------------------------------------------
// Jpeg
//----------------------------------------------------------------------
struct my_error_mgr {
    struct jpeg_error_mgr pub;	/* "public" fields */
    
    jmp_buf setjmp_buffer;	/* for return to caller */
};
typedef struct my_error_mgr * my_error_ptr;

METHODDEF(void)
my_error_exit (j_common_ptr cinfo)
{
    /* cinfo->err really points to a my_error_mgr struct, so coerce pointer */
    my_error_ptr myerr = (my_error_ptr) cinfo->err;
    
    /* Always display the message. */
    /* We could postpone this until after returning, if we chose. */
    (*cinfo->err->output_message) (cinfo);
    
    /* Return control to the setjmp point */
    longjmp(myerr->setjmp_buffer, 1);
}

int Image::readJpegImage(const std::string file, vector<unsigned char>& image, int& width, int& height, int& channel) {
    if (file.substr(file.length() - 3, file.length()) != "jpg") {
        cerr << file.substr(file.length() - 3, file.length()) << endl;
        return -1;
    }
    
    // Use CImg for image loading
    cimg::imagemagick_path("/opt/local/bin/convert");
    CImg<unsigned char> cimage;
    try {
        cimage.load_jpeg(file.c_str());
        if(!cimage.is_empty()) {
            if(cimage.spectrum() != 1 && cimage.spectrum() != 3) {
                cerr << "Cannot handle this component. Component num is " << cimage.spectrum() << endl;
                return 1;
            }
            width = cimage.width();
            height = cimage.height();
            channel = cimage.spectrum();
            
            // CImg holds the image internally in a different order, so we need to reorder it here
            int i = 0;
            if (channel == 1) {
                image.resize(3 * cimage.size());
                
                for(int y = 0; y < cimage.height(); y++) {
                    for(int x = 0; x < cimage.width(); x++) {
                        image[i] = image[i + 1] = image[i + 2] = cimage(x, y, 0, 0);
                        i += 3;
                    }
                }
            }
            else if (channel == 3) {
                image.resize(cimage.size());
                
                for(int y = 0; y < cimage.height(); y++) {
                    for(int x = 0; x < cimage.width(); x++) {
                        for(int c = 0; c < cimage.spectrum(); c++) {
                            image[i] = cimage(x, y, 0, c);
                            i++;
                        }
                    }
                }
            }
        }
    }
    catch(cimg_library::CImgException& e) {
        std::cerr << "Couldn't read image " << file.c_str() << std::endl;
        return -1;
    }
    
    return 0;
}

void Image::writeJpegImage(const string filename, vector<unsigned char>& buffer, const int width, const int height, const int flip) {
    const int quality = 100;
    
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    /* More stuff */
    FILE * outfile;		/* target file */
    JSAMPROW row_pointer[1];	/* pointer to JSAMPLE row[s] */
    int row_stride;		/* physical row width in image buffer */
    
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    
    if ((outfile = fopen(filename.c_str(), "wb")) == NULL) {
        fprintf(stderr, "can't open %s\n", filename.c_str());
        exit(1);
    }
    jpeg_stdio_dest(&cinfo, outfile);
    
    cinfo.image_width = width;
    cinfo.image_height = height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE /* limit to baseline-JPEG values */);
    
    jpeg_start_compress(&cinfo, TRUE);
    
    row_stride = width * 3;	/* JSAMPLEs per row in image_buffer */
    
    while (cinfo.next_scanline < cinfo.image_height) {
        if (flip)
            row_pointer[0] = (JSAMPROW)& buffer[(cinfo.image_height - 1 - cinfo.next_scanline) * row_stride];
        else
            row_pointer[0] = (JSAMPROW)& buffer[cinfo.next_scanline * row_stride];
        (void) jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }
    jpeg_finish_compress(&cinfo);
    fclose(outfile);
    
    jpeg_destroy_compress(&cinfo);
}
    
} // end of namespace open3DCV
