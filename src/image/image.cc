#include "image.h"
#include <fstream>

#define cimg_display 0          // 0 if don't have X11 installed or don't want display capabilities of CImg, 1 if use X11
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

Image::Image() { }

Image::Image(const string r_name) { init(r_name); }
    
Image::Image(const Image& img)
{
    m_image = img.m_image;
    width_ = img.width();
    height_ = img.height();
    channel_ = img.channel();
}
// not tested
Image::Image(const int h, const int w, const int c)
{
    width_ = w;
    height_ = h;
    channel_ = c;
    m_image.resize(h * w * c);
}

Image::~Image() { }

int Image::init(const string r_name)
{
    alloc_ = 0;
    
    if (!ifstream(r_name.c_str()))
    {
        cerr << "not such file" << endl;
        return 1;
    }
    name_ = r_name;
    
    return 0;
}
    
int Image::check_format(const string r_fmt)
{
    int fmt = 0;
    if (r_fmt == "pbm")
        { fmt = 0; }
    else if (r_fmt == "pgm")
        { fmt = 1; }
    else if(r_fmt == "ppm")
        { fmt = 2; }
    else if (r_fmt == "jpg")
        { fmt = 3; }
    else if (r_fmt == "jpeg")
        { fmt = 4; }
    else if (r_fmt == "png")
        { fmt = 5; }
    else if (r_fmt == "tiff")
        { fmt = 6; }
    else
        { fmt = -1; }
    return fmt;
}

void Image::free()
{
    if(alloc_ != 0)
        { alloc_ = 1; }
    else
        { alloc_ = 0; }
    
    vector<unsigned char>().swap(m_image);
}

const int& Image::width() const { return width_; }

const int& Image::height() const { return height_; }
    
const int& Image::channel() const { return channel_; }
    
const string& Image::name() const { return name_; }

// rgb2gray: https://www.mathworks.com/help/matlab/ref/rgb2gray.html
void Image::rgb2grey(const Image& img)
{
    const size_t sz = img.width() * img.height();
    m_image.resize(sz);
    for (size_t i = 0; i < sz; ++i)
    {
        m_image[i] = (char)(0.2989f * img.m_image[3 * i + 0] + 0.5870f * img.m_image[3 * i + 1] + 0.114 * img.m_image[3 * i + 2]);
    }
}
    
void Image::combine_images(const Image& img1, const Image& img2)
{
    if (img1.channel() != img2.channel())
    {
        cerr << "Image should have the same number of channels." << endl;
    }
    int w = img1.width() + img2.width();
    int h = std::max(img1.height(), img2.height());
    int ch = img1.channel();
    m_image.resize(w * h * ch);
    alloc_ = 1;
    width_ = w;
    height_ = h;
    channel_ = ch;
    
    int ind0, ind1;
    // write img1
    for (int y = 0; y < img1.height(); ++y)
    {
        for (int x = 0; x < img1.width(); ++x)
        {
            for (int c = 0; c < ch; ++c)
            {
                ind0 = ch * (y * img1.width() + x) + c;
                ind1 = ch * (y * w + x) + c;
                m_image[ind1] = img1.m_image[ind0];
            }
        }
    }
    
    // write img2
    for (int y = 0; y < img2.height(); ++y)
    {
        for (int x = 0; x < img2.width(); ++x)
        {
            for (int c = 0; c < ch; ++c)
            {
                ind0 = ch * (y * img2.width() + x) + c;
                ind1 = ch * (y * w + x + img1.width()) + c;
                m_image[ind1] = img2.m_image[ind0];
            }
        }
    }
}

int Image::read(const string r_name)
{
    init(r_name);
    string ext;
    int pos = (int)r_name.find(".");
    if (pos != std::string::npos)
        { ext = r_name.substr(pos + 1, r_name.length()); }
    int fmt = check_format(ext);
    
    switch(fmt)
    {
    case 0:
        read_pbm(r_name, m_image, width_, height_);
        channel_ = 1;
        break;
    case 1:
        read_pgm(r_name, m_image, width_, height_);
        channel_ = 1;
        break;
    case 2:
        read_ppm(r_name, m_image, width_, height_, channel_);
        break;
    case 3:
    case 4:
        read_jpeg(r_name, m_image, width_, height_, channel_);
        break;
    case 5:
        break;
    case 6:
        break;
    default:
        cerr << "wrong image format" << endl;
        return 1;
    }
    return 0;
}
    
int Image::write(const string r_name)
{
    string ext;
    int pos = (int)r_name.find(".");
    if (pos != std::string::npos)
        { ext = r_name.substr(pos + 1, r_name.length()); }
    int fmt = check_format(ext);
    
    switch(fmt)
    {
    case 0:
        write_pbm(r_name, m_image, width_, height_);
        break;
    case 1:
        write_pgm(r_name, m_image, width_, height_);
        break;
    case 2:
        write_ppm(r_name, m_image, width_, height_, channel_);
        break;
    case 3:
    case 4:
        write_jpeg(r_name, m_image, width_, height_, channel_);
        break;
    case 5:
        break;
    case 6:
        break;
    default:
        cerr << "wrong image format" << endl;
        return 1;
    }
    return 0;
}
//-----------------------------------------------
// any image
//-----------------------------------------------
int Image::read_any_image(const string r_name, vector<unsigned char>& r_image, int& r_width, int& r_height, int& r_channel)
{
    CImg<unsigned char> cimage;
    try
    {
        cimage.load(r_name.c_str());
        if (!cimage.is_empty())
        {
            if (cimage.spectrum() != 1 && cimage.spectrum() != 3)
            {
                cerr << "Cannot handle the components. Component num is " << cimage.spectrum() << endl;
                return 1;
            }
            r_width = cimage.width();
            r_height = cimage.height();
            r_channel = cimage.spectrum();
            r_image.resize(cimage.size());
            
            // CImg holds the image internally in a different order, so we need to reorder it here
            int i = 0;
            for (int y = 0; y < r_height; ++y)
            {
                for (int x = 0; x < r_width; ++x)
                {
                    for (int c = 0; c < r_channel; ++c)
                    {
                        r_image[i] = cimage(x, y, 0, c);
                        i++;
                    }
                }
            }
        }
    }
    catch (cimg_library::CImgException& e)
    {
        cerr << "Cannot read image " << r_name << endl;
        return 1;
    }
    return 0;
}

//-----------------------------------------------
// PBM
//-----------------------------------------------
int Image::read_pbm(const string r_name, vector<unsigned char>& r_image, int& r_width, int& r_height)
{
    if (r_name.substr(r_name.length() - 3, r_name.length()) != "pbm")
        { return 1; }
    
    ifstream ifstr;
    ifstr.open(r_name.c_str());
    if (!ifstr.is_open()) { return 1; }
    
    string header;
    unsigned char uctmp;
    
    ifstr >> header;
    ifstr.read((char*)&uctmp, sizeof(unsigned char));
    
    if (header != "P4")
    {
        cerr << "Only accept binary pbm format: " << r_name << endl;
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
        { break; }
    }
    ifstr >> r_width >> r_height;
    ifstr.read((char*)&uctmp, sizeof(unsigned char));
    
    r_image.clear();
    
    int reso = r_width * r_height;
    if (reso % 8 != 0)
    { reso++; }
    
    int count = 0;
    for (int i = 0; i < reso; ++i)
    {
        ifstr.read((char*)&uctmp, sizeof(unsigned char));
        for (int j = 0; j < 8; ++j)
        {
            if (uctmp >> 7)
            { r_image.push_back((unsigned char)0); }
            else
            { r_image.push_back((unsigned char)255); }
            count++;
            uctmp <<= 1;
            if (count == r_width * r_height)
            { break; }
        }
    }
    ifstr.close();
    return 0;
}

int Image::write_pbm(const string r_name, vector<unsigned char>& r_image, int& r_width, int& r_height)
{
    ofstream ofstr;
    ofstr.open(r_name.c_str());
    if(!ofstr.is_open())
    {
        cerr << "Cannot write to a file" << r_name << endl;
        return 1;
    }
    ofstr << "P4" << endl << r_width << ' ' << r_height << endl;
    
    unsigned char uctmp = 0;
    for (int i = 0; i < r_width * r_height; ++i)
    {
        uctmp <<= 1;
        if(r_image[i] < 127)
            uctmp |= 0x0001;
        else
            uctmp &= 0x0001;
        
        if (i % 8 == 7)
            ofstr.write((char*)&uctmp, sizeof(char));
    }
    const int itmp = (r_width * r_height) % 8;
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
int Image::read_pgm(const string r_name, vector<unsigned char>& r_image, int& r_width, int& r_height)
{
    if (r_name.substr(r_name.length() - 3, r_name.length()) != "pgm")
        { return 1; }
    
    ifstream ifstr;
    ifstr.open(r_name.c_str());
    if (!ifstr.is_open())
    {
        cerr << "Cannot open a file: " << r_name << endl;
        return 1;
    }
    
    string header;
    unsigned char uctmp;
    
    ifstr >> header;
    ifstr.read((char*)&uctmp, sizeof(unsigned char));
    if (header != "P5")
    {
        cerr << "Only accept binary pgm format" << r_name << " " << header << endl;
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
    ifstr >> r_width >> r_height >> itmp;
    ifstr.read((char*)&uctmp, sizeof(unsigned char));
    
    r_image.clear();
    for (int y = 0; y < r_height; ++y)
    {
        for (int x = 0; x < r_width; ++x)
        {
            ifstr.read((char*)&uctmp, sizeof(unsigned char));
            r_image.push_back(uctmp);
        }
    }
    ifstr.close();
    
    return 0;
}

int Image::write_pgm(const string r_name, vector<unsigned char>& r_image, int& r_width, int& r_height)
{
    std::ofstream ofstr;
    ofstr.open(r_name.c_str());
    if (!ofstr.is_open()) {
        cerr << "Cannot write to a file: " << r_name << endl;
        return 1;
    }
    
    ofstr << "P5" << endl
          << r_width << ' ' << r_height << endl
          << 255 << endl;
    
    for (int i = 0; i < r_width * r_height; ++i) {
        unsigned char uctmp = r_image[i];
        ofstr.write((char*)&uctmp, sizeof(unsigned char));
    }
    
    ofstr.close();
    return 0;
}

//----------------------------------------------------------------------
// PPM
//----------------------------------------------------------------------
int Image::read_ppm(const string r_name, vector<unsigned char>& r_image, int& r_width, int& r_height, int& r_channel)
{
    if (r_name.substr(r_name.length() - 3, r_name.length()) != "ppm")
        { return 1; }
    
    CImg<unsigned char> cimage;
    try
    {
        cimage.load_pnm(r_name.c_str());
        if(!cimage.is_empty())
        {
            if(cimage.spectrum() != 1 && cimage.spectrum() != 3)
            {
                cerr << "Cannot handle this component. Component num is" << cimage.spectrum() << endl;
                return 1;
            }
            r_width = cimage.width();
            r_height = cimage.height();
            r_channel = cimage.spectrum();
            r_image.resize(cimage.size());
            
            int i = 0;
            for (int y = 0; y < r_height; ++y)
            {
                for (int x = 0; x < r_width; ++x)
                {
                    for (int c = 0; c < r_channel; ++c)
                    {
                        r_image[i] = cimage(x, y, 0, c);
                        ++i;
                    }
                }
            }
        }
    }
    catch(cimg_library::CImgException &e)
    {
        cerr << "Cannot read image " << r_name << endl;
        return 1;
    }
    return 0;
}

int Image::write_ppm(const string r_name, vector<unsigned char>& r_image, int& r_width, int& r_height, int& r_channel)
{
    ofstream ofstr;
    ofstr.open(r_name.c_str());
    if(!ofstr.is_open())
    {
        cerr << "Cannot write to a file: " << r_name << endl;
        return 1;
    }
    ofstr << "P6" << endl << r_width << ' ' << r_height << endl << 255 << endl;
    
    for (int i = 0; i < r_channel * r_width * r_height; ++i)
    {
        unsigned char uctmp = r_image[i];
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

int Image::read_jpeg(const string r_name, vector<unsigned char>& r_image, int& r_width, int& r_height, int& r_channel) {
    if (r_name.substr(r_name.length() - 3, r_name.length()) != "jpg" &&
        r_name.substr(r_name.length() - 4, r_name.length()) != "jpeg")
    {
        cerr << r_name.substr(r_name.length() - 4, r_name.length()) << endl;
        return 1;
    }
    
    // Use CImg for image loading
    cimg::imagemagick_path("/opt/local/bin/convert");
    CImg<unsigned char> cimage;
    try {
        cimage.load_jpeg(r_name.c_str());
        if(!cimage.is_empty())
        {
            if(cimage.spectrum() != 1 && cimage.spectrum() != 3)
            {
                cerr << "Cannot handle this component. Component num is " << cimage.spectrum() << endl;
                return 1;
            }
            r_width = cimage.width();
            r_height = cimage.height();
            r_channel = cimage.spectrum();
            r_image.resize(cimage.size());
            
            // CImg holds the image internally in a different order, so we need to reorder it here
            int i = 0;
            for(int y = 0; y < r_height; y++) {
                for(int x = 0; x < r_width; x++) {
                    for(int c = 0; c < r_channel; c++) {
                        r_image[i] = cimage(x, y, 0, c);
                        i++;
                    }
                }
            }
        }
    }
    catch(cimg_library::CImgException& e) {
        cerr << "Couldn't read image " << r_name.c_str() << endl;
        return 1;
    }
    
    return 0;
}

void Image::write_jpeg(const string r_name, vector<unsigned char>& r_image, int& r_width, int& r_height, int& r_channel, const int flip) {
    const int quality = 100;
    
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    /* More stuff */
    FILE * outfile;		/* target file */
    JSAMPROW row_pointer[1];	/* pointer to JSAMPLE row[s] */
    int row_stride;		/* physical row width in image buffer */
    
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    
    if ((outfile = fopen(r_name.c_str(), "wb")) == NULL) {
        fprintf(stderr, "can't open %s\n", r_name.c_str());
        exit(1);
    }
    jpeg_stdio_dest(&cinfo, outfile);
    
    cinfo.image_width = r_width;
    cinfo.image_height = r_height;
    cinfo.input_components = r_channel;
    cinfo.in_color_space = JCS_RGB;
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE /* limit to baseline-JPEG values */);
    
    jpeg_start_compress(&cinfo, TRUE);
    
    row_stride = r_width * r_channel;	/* JSAMPLEs per row in image_buffer */
    
    while (cinfo.next_scanline < cinfo.image_height) {
        if (flip)
            row_pointer[0] = (JSAMPROW)& r_image[(cinfo.image_height - 1 - cinfo.next_scanline) * row_stride];
        else
            row_pointer[0] = (JSAMPROW)& r_image[cinfo.next_scanline * row_stride];
        (void) jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }
    jpeg_finish_compress(&cinfo);
    fclose(outfile);
    
    jpeg_destroy_compress(&cinfo);
}

    
// need to re-examing
Vector3f Image::color(const int ix, const int iy) const {
#ifdef PMMVPS_DEBUG
    if (alloc != 2)
    {
        cerr << "First allocate" << endl;
        exit (1);
    }
#endif
    const int index = (iy * width_ + ix) * channel_;
    
    return Vector3f(m_image[index],
                    m_image[index+1],
                    m_image[index+2]);
}

Vector3f Image::color(const float x, const float y) const {
#ifdef PMMVPS_DEBUG
    if (alloc != 2) {
        cerr << "First allocate" << endl;
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
    
    const int offset = width_ * 3;
    const int index0 = ((y1 - 1) * width_ + x1 - 1) * 3;
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
    const int index = 3 * (ly * width_ + lx);
    
    const float dx1 = x - lx;  const float dx0 = 1.0f - dx1;
    const float dy1 = y - ly;  const float dy0 = 1.0f - dy1;
    
    const float f00 = dx0 * dy0;  const float f01 = dx0 * dy1;
    const float f10 = dx1 * dy0;  const float f11 = dx1 * dy1;
    const int index2 = index + 3 * width_;
    
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
} // end of namespace open3DCV
