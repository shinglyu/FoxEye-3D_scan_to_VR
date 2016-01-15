/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2013-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief Vertex Color Tranferer
 *
 * This transfers a point cloud's vertex color to a mesh, using the nearest K neighbor serach
 *
 * \author Shing Lyu
 *
 */

#include <boost/lexical_cast.hpp>

#include <pcl/console/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
//#include <stdint.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int, char **argv)
{
  std::cout << std::endl;
  std::cout << "Transfer the vertex color from a colored point cloud to its corresponding colorless polygon mesh." << std::endl;
  std::cout << "Usage: " << argv[0] << " colored_point_cloud.pcd colorless_mesh.vtk output.ply" << std::endl;
  std::cout << std::endl;
}

bool
loadPCDCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud)
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

bool
loadVTKMesh(const std::string &filename, pcl::PolygonMesh &mesh) {
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPolygonFileVTK(filename, mesh) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", mesh.cloud.width * mesh.cloud.height); print_info (" points : "); print_value("%d", mesh.polygons.size()); print_info (" polygons]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (mesh.cloud).c_str ());

  return (true);
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
  print_info ("Transfers the vertex color of a point colud to a mesh.\nFor more information, use: %s --help\n", argv[0]);

  if (argc < 4 || pcl::console::find_switch (argc, argv, "--help"))
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd and .png files
  std::vector<int> pcd_file_index = parse_file_extension_argument (argc, argv, ".pcd");
  std::vector<int> vtk_in_file_index = parse_file_extension_argument (argc, argv, ".vtk");
  std::vector<int> ply_out_file_index = parse_file_extension_argument (argc, argv, ".ply");

  /*
  if (pcd_file_index.size () != 1 || png_file_index.size () != 1)
  {
    print_error ("Need one input PCD file and one output PNG file.\n");
    return (-1);
  }
  */

  std::string pcd_filename = argv[pcd_file_index[0]];
  std::string vtk_in_filename = argv[vtk_in_file_index[0]];

  // Load the input file
  pcl::PCLPointCloud2::Ptr origColorCloud2 (new pcl::PCLPointCloud2);
  if (!loadPCDCloud (pcd_filename, *origColorCloud2))
  {
    print_error ("Unable to load PCD file.\n");
    return (-1);
  }

  PolygonMesh mesh;
  //loadPolygonFileVTK (argv[vtk_in_file_index[0]], mesh);
  loadVTKMesh(argv[vtk_in_file_index[0]], mesh);

  PolygonMesh coloredMesh;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::copyPointCloud(mesh.cloud, *coloredCloud);
  pcl::fromPCLPointCloud2(mesh.cloud, *coloredCloud);


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr origColorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(*origColorCloud2, *origColorCloud);
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud (origColorCloud);

  /*
  float resolution = 128.0f;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree (resolution);
  octree.setInputCloud (*origColorCloud);
  octree.addPointsFromInputCloud ();
  */


  //Ref: http://pointclouds.org/documentation/tutorials/kdtree_search.php#kdtree-search
  for (PointCloud<pcl::PointXYZRGB>::iterator cloud_it (coloredCloud->begin()); cloud_it != coloredCloud->end(); ++cloud_it)
  {
    int K = 5;

    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;

    /*
    std::cout << "K nearest neighbor search at (" << searchPoint.x 
      << " " << searchPoint.y 
      << " " << searchPoint.z
      << ") with K=" << K << std::endl;
    */

    uint8_t r255, g255, b255;
    if (kdtree.nearestKSearch (*cloud_it, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
      float r = 0;
      float g = 0;
      float b = 0;
      for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
        //TODO: smartly detect if rgb or rgba
        r += ((uint32_t)origColorCloud->points[ pointIdxNKNSearch[i] ].rgba >> 16) & 0x0000ff;
        g += ((uint32_t)origColorCloud->points[ pointIdxNKNSearch[i] ].rgba >> 8 ) & 0x0000ff;
        b += ((uint32_t)origColorCloud->points[ pointIdxNKNSearch[i] ].rgba      ) & 0x0000ff;
        //std::cout << (uint32_t)origColorCloud->points[ pointIdxNKNSearch[i] ].rgba << std::endl;
        /*
        std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i]  ].x 
          << " " << cloud->points[ pointIdxNKNSearch[i]  ].y 
          << " " << cloud->points[ pointIdxNKNSearch[i]  ].z 
          << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
          */
      }
      //std::cout << (int)r << " " << (int)g << " " << (int)b << std::endl;
      r255 = (int)(r / pointIdxNKNSearch.size () + 0.5);
      g255 = (int)(g / pointIdxNKNSearch.size () + 0.5);
      b255 = (int)(b / pointIdxNKNSearch.size () + 0.5);
      //std::cout << (int)r255 << " " << (int)g255 << " " << (int)b255 << std::endl;

    }

    /*
    uint8_t r = rand() % (int)(255);
    uint8_t g = rand() % (int)(255);
    uint8_t b = rand() % (int)(255);
    */
    uint32_t rgb = ((uint32_t)r255 << 16 | (uint32_t)g255 << 8 | (uint32_t)b255);
    //std::cout << r << std::endl;
    //std::cout << g << std::endl;
    //std::cout << b << std::endl;
    //std::cout << *reinterpret_cast<float*>(&rgb) << std::endl;
    cloud_it->rgb = *reinterpret_cast<float*>(&rgb);
  } 

  //savePCDFileASCII ("debug_colored.pcd", *coloredCloud);
  //std::cerr << "Saved " << coloredCloud.points.size () << " data points to debug_colored.pcd." << std::endl;
  //
  //PCLPointCloud2 coloredCloud2;
  //pcl::toPCLPointCloud2(*coloredCloud, coloredCloud2);
  pcl::toPCLPointCloud2(*coloredCloud, mesh.cloud);
  //mesh.cloud = coloredCloud2;
  //saveVTKFile("debug_colored.vtk", mesh); //No color
  //saveOBJFile("debug_colored.obj", mesh); //No color
  savePLYFile(argv[ply_out_file_index[0]], mesh);




  return (0);
}
