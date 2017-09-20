#include "image/image.h"
#include "viz/plot.h"

using namespace open3DCV;
int main (const int argc, const char** argv)
{
    string iname = "/Users/BlacKay/Documents/Projects/Images/test/1.jpg";
    Image image(iname);
    image.read(iname);
    gl_init();
}
