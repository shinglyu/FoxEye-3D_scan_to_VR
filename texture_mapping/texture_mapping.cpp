#include <boost/lexical_cast.hpp>

#include <pcl/console/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/conversions.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int, char **argv)
{
  std::cout << std::endl;
  std::cout << "****************************************************************************" << std::endl;
  std::cout << "*                                                                          *" << std::endl;
  std::cout << "*                       PCD 2 PNG CONVERTER - Usage Guide                  *" << std::endl;
  std::cout << "*                                                                          *" << std::endl;
  std::cout << "****************************************************************************" << std::endl;
  std::cout << std::endl;
  std::cout << "Usage: " << argv[0] << " [Options] input.pcd output.png" << std::endl;
  std::cout << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << std::endl;
  std::cout << "     --help   : Show this help"                                               << std::endl;
  std::cout << "     --no-nan : Paint NaN (infinite) points with black color regardless of"   << std::endl;
  std::cout << "                field contents"                                               << std::endl;
  std::cout << "     --field  : Set the field to extract data from. Supported fields:"        << std::endl;
  std::cout << "                - normal"                                                     << std::endl;
  std::cout << "                * rgb (default)"                                              << std::endl;
  std::cout << "                - label"                                                      << std::endl;
  std::cout << "                - z"                                                          << std::endl;
  std::cout << "                - curvature"                                                  << std::endl;
  std::cout << "                - intensity"                                                  << std::endl;
  std::cout << "     --scale  : Apply scaling to extracted data (only for z, curvature, and"  << std::endl;
  std::cout << "                intensity fields). Supported options:"                        << std::endl;
  std::cout << "                - <float> : Scale by a fixed number"                          << std::endl;
  std::cout << "                - auto    : Auto-scale to the full range"                     << std::endl;
  std::cout << "                - no      : No scaling"                                       << std::endl;
  std::cout << "                If the option is omitted then default scaling (depends on"    << std::endl;
  std::cout << "                the field type) will be used."                                << std::endl;
  std::cout << "     --colors : Choose color mapping mode for labels (only for label field)." << std::endl;
  std::cout << "                Supported options:"                                           << std::endl;
  std::cout << "                - mono    : Shades of gray"                                   << std::endl;
  std::cout << "                - rgb     : Randomly generated RGB colors"                    << std::endl;
  std::cout << "                * glasbey : Fixed colors from the Glasbey table¹ (default)"   << std::endl;
  std::cout << std::endl;
  std::cout << "Notes:"                                                                       << std::endl;
  std::cout << std::endl;
  std::cout << "¹) The Glasbey lookup table is a color table structured in a maximally"       << std::endl;
  std::cout << "   discontinuous manner. Adjacent color bins are chosen to be as distinct"    << std::endl;
  std::cout << "   from one another as possible (see https://github.com/taketwo/glasbey)."    << std::endl;
  std::cout << "   The label with the smallest id will be assigned the first color from the"  << std::endl;
  std::cout << "   table, the second smallest will have the second color, and so on. Thus,"   << std::endl;
  std::cout << "   if you have several clouds with the same labels, you will get repetitive"  << std::endl;
  std::cout << "   consistently colored PNG images."                                          << std::endl;
}

bool
loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

void
saveImage (const std::string &filename, const pcl::PCLImage& image)
{
  TicToc tt;
  tt.tic ();
  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
  savePNGFile (filename, image);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", image.width * image.height); print_info (" points]\n");
}

template<typename T> bool
parseScaleOption (int argc, char** argv, T& pcie)
{
  std::string scaling = "default";
  pcl::console::parse_argument (argc, argv, "--scale", scaling);
  print_info ("Scaling: "); print_value ("%s\n", scaling.c_str());
  if (scaling == "default")
  {
    // scaling option omitted, use whatever defaults image extractor has
  }
  else if (scaling == "no")
  {
    pcie.setScalingMethod(pcie.SCALING_NO);
  }
  else if (scaling == "auto")
  {
    pcie.setScalingMethod(pcie.SCALING_FULL_RANGE);
  }
  else
  {
    try
    {
      float factor = boost::lexical_cast<float> (scaling);
      pcie.setScalingMethod(pcie.SCALING_FIXED_FACTOR);
      pcie.setScalingFactor(factor);
    }
    catch (const boost::bad_lexical_cast&)
    {
      print_error ("The value of --scale option should be \"no\", \"auto\", or a floating point number.\n");
      return false;
    }
  }
  return true;
}

template<typename T> bool
parseColorsOption (int argc, char** argv, T& pcie)
{
  std::string colors = "glasbey";
  pcl::console::parse_argument (argc, argv, "--colors", colors);
  print_info ("Colors: "); print_value ("%s\n", colors.c_str());
  if (colors == "mono")
  {
    pcie.setColorMode(pcie.COLORS_MONO);
  }
  else if (colors == "rgb")
  {
    pcie.setColorMode(pcie.COLORS_RGB_RANDOM);
  }
  else if (colors == "glasbey")
  {
    pcie.setColorMode(pcie.COLORS_RGB_GLASBEY);
  }
  else
  {
    return false;
  }
  return true;
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Convert a PCD file to PNG format.\nFor more information, use: %s --help\n", argv[0]);

  //if (argc < 3 || pcl::console::find_switch (argc, argv, "--help"))
  if (argc < 2 || pcl::console::find_switch (argc, argv, "--help"))
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd and .png files
  //std::vector<int> pcd_file_index = parse_file_extension_argument (argc, argv, ".pcd");
  std::vector<int> png_file_index = parse_file_extension_argument (argc, argv, ".png");

  if (png_file_index.size () != 1)
  {
    print_error ("Need one input PCD file and one output PNG file.\n");
    return (-1);
  }

  //std::string pcd_filename = argv[pcd_file_index[0]];
  std::string png_filename = argv[png_file_index[0]];

  pcl::PCLImage img;

  img.encoding = "rgb8";
  //img.width = cloud.width;
  //img.height = cloud.height;
  img.width = 4;
  img.height = 4;
  img.step = img.width * sizeof (unsigned char) * 3;
  img.data.resize (img.step * img.height);

  std::srand(std::time(0));
  for (size_t i = 0; i < img.width * img.height; ++i)
  {
    /*
    uint32_t val;
    //pcl::getFieldValue<PointT, uint32_t> (cloud.points[i], offset, val);
    val = 256;
    img.data[i * 3 + 0] = (val >> 16) & 0x0000ff;
    img.data[i * 3 + 1] = (val >> 8) & 0x0000ff;
    img.data[i * 3 + 2] = (val) & 0x0000ff;
    */
    img.data[i * 3 + 0] = static_cast<uint8_t> ((std::rand () % 256));
    img.data[i * 3 + 1] = static_cast<uint8_t> ((std::rand () % 256));
    img.data[i * 3 + 2] = static_cast<uint8_t> ((std::rand () % 256));
  }

  /*
  std::srand(std::time(0));
  std::map<uint32_t, size_t> colormap;

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    uint32_t val;
    //pcl::getFieldValue<PointT, uint32_t> (cloud.points[i], offset, val);
    val = 0;
    if (colormap.count (val) == 0)
    {
      colormap[val] = i * 3;
      img.data[i * 3 + 0] = static_cast<uint8_t> ((std::rand () % 256));
      img.data[i * 3 + 1] = static_cast<uint8_t> ((std::rand () % 256));
      img.data[i * 3 + 2] = static_cast<uint8_t> ((std::rand () % 256));

    }
    else
    {
      memcpy (&img.data[i * 3], &img.data[colormap[val]], 3);

    }
                    }
                    */

  saveImage (png_filename, img);

  return (0);
}
