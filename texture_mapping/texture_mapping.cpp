#include <boost/lexical_cast.hpp>

#include <pcl/console/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/png_io.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Core>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

/** \brief Save a textureMesh object to obj file */
int
saveOBJFile (const std::string &file_name,
             const pcl::TextureMesh &tex_mesh, unsigned precision)
{
  if (tex_mesh.cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no data!\n");
    return (-1);
  }

  // Open file
  std::ofstream fs;
  fs.precision (precision);
  fs.open (file_name.c_str ());

  // Define material file
  std::string mtl_file_name = file_name.substr (0, file_name.find_last_of (".")) + ".mtl";
  // Strip path for "mtllib" command
  std::string mtl_file_name_nopath = mtl_file_name;
  mtl_file_name_nopath.erase (0, mtl_file_name.find_last_of ('/') + 1);

  /* Write 3D information */
  // number of points
  int nr_points  = tex_mesh.cloud.width * tex_mesh.cloud.height;
  int point_size = tex_mesh.cloud.data.size () / nr_points;

  // mesh size
  int nr_meshes = tex_mesh.tex_polygons.size ();
  // number of faces for header
  int nr_faces = 0;
  for (int m = 0; m < nr_meshes; ++m)
    nr_faces += tex_mesh.tex_polygons[m].size ();

  // Write the header information
  fs << "####" << std::endl;
  fs << "# OBJ dataFile simple version. File name: " << file_name << std::endl;
  fs << "# Vertices: " << nr_points << std::endl;
  fs << "# Faces: " <<nr_faces << std::endl;
  fs << "# Material information:" << std::endl;
  fs << "mtllib " << mtl_file_name_nopath << std::endl;
  fs << "####" << std::endl;

  // Write vertex coordinates
  fs << "# Vertices" << std::endl;
  for (int i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    // "v" just be written one
    bool v_written = false;
    for (size_t d = 0; d < tex_mesh.cloud.fields.size (); ++d)
    {
      int count = tex_mesh.cloud.fields[d].count;
      if (count == 0)
        count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
      int c = 0;
      // adding vertex
      if ((tex_mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
                tex_mesh.cloud.fields[d].name == "x" ||
                tex_mesh.cloud.fields[d].name == "y" ||
                tex_mesh.cloud.fields[d].name == "z"))
      {
        if (!v_written)
        {
            // write vertices beginning with v
            fs << "v ";
            v_written = true;
        }
        float value;
        memcpy (&value, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fs << value;
        if (++xyz == 3)
            break;
        fs << " ";
      }
    }
    if (xyz != 3)
    {
      PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no XYZ data!\n");
      return (-2);
    }
    fs << std::endl;
  }
  fs << "# "<< nr_points <<" vertices" << std::endl;

  // Write vertex normals
  for (int i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    // "vn" just be written one
    bool v_written = false;
    for (size_t d = 0; d < tex_mesh.cloud.fields.size (); ++d)
    {
      int count = tex_mesh.cloud.fields[d].count;
      if (count == 0)
      count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
      int c = 0;
      // adding vertex
      if ((tex_mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
      tex_mesh.cloud.fields[d].name == "normal_x" ||
      tex_mesh.cloud.fields[d].name == "normal_y" ||
      tex_mesh.cloud.fields[d].name == "normal_z"))
      {
        if (!v_written)
        {
          // write vertices beginning with vn
          fs << "vn ";
          v_written = true;
        }
        float value;
        memcpy (&value, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fs << value;
        if (++xyz == 3)
          break;
        fs << " ";
      }
    }
    if (xyz != 3)
    {
    PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no normals!\n");
    return (-2);
    }
    fs << std::endl;
  }
  // Write vertex texture with "vt" (adding latter)

  for (int m = 0; m < nr_meshes; ++m)
  {
    if(tex_mesh.tex_coordinates.size() == 0)
      continue;

    PCL_INFO ("%d vertex textures in submesh %d\n", tex_mesh.tex_coordinates[m].size (), m);
    fs << "# " << tex_mesh.tex_coordinates[m].size() << " vertex textures in submesh " << m <<  std::endl;
    for (size_t i = 0; i < tex_mesh.tex_coordinates[m].size (); ++i)
    {
      fs << "vt ";
      fs <<  tex_mesh.tex_coordinates[m][i][0] << " " << tex_mesh.tex_coordinates[m][i][1] << std::endl;
    }
  }

  int f_idx = 0;

  // int idx_vt =0;
  PCL_INFO ("Writting faces...\n");
  for (int m = 0; m < nr_meshes; ++m)
  {
    if (m > 0) 
      f_idx += tex_mesh.tex_polygons[m-1].size ();

    if(tex_mesh.tex_materials.size() !=0)
    {
      fs << "# The material will be used for mesh " << m << std::endl;
      //TODO pbl here with multi texture and unseen faces
      fs << "usemtl " <<  tex_mesh.tex_materials[m].tex_name << std::endl;
      fs << "# Faces" << std::endl;
    }
    for (size_t i = 0; i < tex_mesh.tex_polygons[m].size(); ++i)
    {
      // Write faces with "f"
      fs << "f";
      size_t j = 0;
      // There's one UV per vertex per face, i.e., the same vertex can have
      // different UV depending on the face.
      for (j = 0; j < tex_mesh.tex_polygons[m][i].vertices.size (); ++j)
      {
        unsigned int idx = tex_mesh.tex_polygons[m][i].vertices[j] + 1;
        fs << " " << idx
        << "/" << 3*(i+f_idx) +j+1
        << "/" << idx; // vertex index in obj file format starting with 1
      }
      fs << std::endl;
    }
    PCL_INFO ("%d faces in mesh %d \n", tex_mesh.tex_polygons[m].size () , m);
    fs << "# "<< tex_mesh.tex_polygons[m].size() << " faces in mesh " << m << std::endl;
  }
  fs << "# End of File";

  // Close obj file
  PCL_INFO ("Closing obj file\n");
  fs.close ();

  /* Write material defination for OBJ file*/
  // Open file
  PCL_INFO ("Writing material files\n");
  //dont do it if no material to write
  if(tex_mesh.tex_materials.size() ==0)
    return (0);

  std::ofstream m_fs;
  m_fs.precision (precision);
  m_fs.open (mtl_file_name.c_str ());

  // default
  m_fs << "#" << std::endl;
  m_fs << "# Wavefront material file" << std::endl;
  m_fs << "#" << std::endl;
  for(int m = 0; m < nr_meshes; ++m)
  {
    m_fs << "newmtl " << tex_mesh.tex_materials[m].tex_name << std::endl;
    m_fs << "Ka "<< tex_mesh.tex_materials[m].tex_Ka.r << " " << tex_mesh.tex_materials[m].tex_Ka.g << " " << tex_mesh.tex_materials[m].tex_Ka.b << std::endl; // defines the ambient color of the material to be (r,g,b).
    m_fs << "Kd "<< tex_mesh.tex_materials[m].tex_Kd.r << " " << tex_mesh.tex_materials[m].tex_Kd.g << " " << tex_mesh.tex_materials[m].tex_Kd.b << std::endl; // defines the diffuse color of the material to be (r,g,b).
    m_fs << "Ks "<< tex_mesh.tex_materials[m].tex_Ks.r << " " << tex_mesh.tex_materials[m].tex_Ks.g << " " << tex_mesh.tex_materials[m].tex_Ks.b << std::endl; // defines the specular color of the material to be (r,g,b). This color shows up in highlights.
    m_fs << "d " << tex_mesh.tex_materials[m].tex_d << std::endl; // defines the transparency of the material to be alpha.
    m_fs << "Ns "<< tex_mesh.tex_materials[m].tex_Ns  << std::endl; // defines the shininess of the material to be s.
    m_fs << "illum "<< tex_mesh.tex_materials[m].tex_illum << std::endl; // denotes the illumination model used by the material.
    // illum = 1 indicates a flat material with no specular highlights, so the value of Ks is not used.
    // illum = 2 denotes the presence of specular highlights, and so a specification for Ks is required.
    m_fs << "map_Kd " << tex_mesh.tex_materials[m].tex_file << std::endl;
    m_fs << "###" << std::endl;
  }
  m_fs.close ();
  return (0);
}
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
loadPLYCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPLYFile (filename, cloud) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

//TODO: include this function from ../tansfer_vertex_color
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
  //std::vector<int> ply_file_index = parse_file_extension_argument (argc, argv, ".ply");
  std::vector<int> vtk_file_index = parse_file_extension_argument (argc, argv, ".vtk");
  std::vector<int> png_file_index = parse_file_extension_argument (argc, argv, ".png");

  if (vtk_file_index.size () != 1 || png_file_index.size () != 1)
  {
    print_error ("Need one input PCD file and one output PNG file.\n");
    return (-1);
  }

  //std::string ply_filename = argv[ply_file_index[0]];
  std::string vtk_filename = argv[vtk_file_index[0]];
  std::string png_filename = argv[png_file_index[0]];

  pcl::PCLImage img;

  img.encoding = "rgb8";
  //img.width = cloud.width;
  //img.height = cloud.height;
  //TODO: dynamically detect width and height
  img.width = 2048; 
  img.height = 2048;
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
    /*
    img.data[i * 3 + 0] = static_cast<uint8_t> ((std::rand () % 256));
    img.data[i * 3 + 1] = static_cast<uint8_t> ((std::rand () % 256));
    img.data[i * 3 + 2] = static_cast<uint8_t> ((std::rand () % 256));
    */
    img.data[i * 3 + 0] = static_cast<uint8_t> (255);
    img.data[i * 3 + 1] = static_cast<uint8_t> (0);
    img.data[i * 3 + 2] = static_cast<uint8_t> (0);
  }


  saveImage (png_filename, img);

  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  //pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
  //if (!loadPLYCloud (ply_filename, *cloud))
  //if (!loadPLYCloud (ply_filename, *cloud))
  PolygonMesh mesh;
  if (!loadVTKMesh (vtk_filename, mesh))
  //if (pcl::io::loadPLYFile<pcl::PointXYZRGB> (ply_filename, *cloud) == -1) 
  {
    print_error ("Unable to load PLY file.\n");
    return (-1);
  }

  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::toPCLPointCloud2(*coloredCloud, mesh.cloud);


  TextureMesh tex_mesh;
  tex_mesh.cloud = mesh.cloud;
  //tex_mesh.tex_polygons[0] = mesh.polygons;
  std::vector< pcl::Vertices> polygon_1;

  // push faces into the texturemesh object
  polygon_1.resize (mesh.polygons.size ());
  for(size_t i =0; i < mesh.polygons.size (); ++i)
  {
    polygon_1[i] = mesh.polygons[i];
  }
  tex_mesh.tex_polygons.push_back(polygon_1);

  // mesh information
  int nr_points = tex_mesh.cloud.width * tex_mesh.cloud.height;
  int point_size = static_cast<int> (tex_mesh.cloud.data.size ()) / nr_points;

  // temporary PointXYZ
  float x, y, z;
  // temporary face
  //Eigen::Vector3f facet[3];

  // texture coordinates for each mesh
  std::vector<std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > >texture_map;

  for (size_t m = 0; m < tex_mesh.tex_polygons.size (); ++m)
  {
    // texture coordinates for each mesh
    //std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > texture_map_tmp;
    std::vector<Eigen::Vector2f> texture_map_tmp;

    // processing for each face
    for (size_t i = 0; i < tex_mesh.tex_polygons[m].size (); ++i)
    {
      size_t idx;

      // get facet information
      /*
      for (size_t j = 0; j < tex_mesh.tex_polygons[m][i].vertices.size (); ++j)
      {
        idx = tex_mesh.tex_polygons[m][i].vertices[j];
        memcpy (&x, &tex_mesh.cloud.data[idx * point_size + tex_mesh.cloud.fields[0].offset], sizeof(float));
        memcpy (&y, &tex_mesh.cloud.data[idx * point_size + tex_mesh.cloud.fields[1].offset], sizeof(float));
        memcpy (&z, &tex_mesh.cloud.data[idx * point_size + tex_mesh.cloud.fields[2].offset], sizeof(float));
        facet[j][0] = x;
        facet[j][1] = y;
        facet[j][2] = z;
      }

      // get texture coordinates of each face
      std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > tex_coordinates = mapTexture2Face (facet[0], facet[1], facet[2]);
      */
      Eigen::Vector2f tp1, tp2, tp3;
      size_t col = i % (img.width / 2);
      size_t row = i / (img.height/ 2);
      //printf("%zu\n",col);
      //printf("%zu\n",row);
      tp1[0] = (col * 2 + 0.0) / img.width;
      tp1[1] = (row * 2 + 0.0) / img.height;

      tp2[0] = (col * 2 + 1.0) / img.width;
      tp2[1] = (row * 2 + 0.0) / img.height;

      tp3[0] = (col * 2 + 0.0) / img.width;
      tp3[1] = (row * 2 + 1.0) / img.height;
      std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > tex_coordinates;
      tex_coordinates.push_back (tp1);
      tex_coordinates.push_back (tp2);
      tex_coordinates.push_back (tp3);
      for (size_t n = 0; n < tex_coordinates.size (); ++n)
        texture_map_tmp.push_back (tex_coordinates[n]);
    }// end faces
    //https://github.com/PointCloudLibrary/pcl/blob/master/surface/include/pcl/surface/impl/texture_mapping.hpp#L181
    tex_mesh.tex_coordinates.push_back (texture_map_tmp);

    //texture materials
    pcl::TexMaterial mesh_material;
    mesh_material.tex_Ka.r = 0.2f;
    mesh_material.tex_Ka.g = 0.2f;
    mesh_material.tex_Ka.b = 0.2f;

    mesh_material.tex_Kd.r = 0.8f;
    mesh_material.tex_Kd.g = 0.8f;
    mesh_material.tex_Kd.b = 0.8f;

    mesh_material.tex_Ks.r = 1.0f;
    mesh_material.tex_Ks.g = 1.0f;
    mesh_material.tex_Ks.b = 1.0f;

    mesh_material.tex_d = 1.0f;
    mesh_material.tex_Ns = 75.0f;
    mesh_material.tex_illum = 2;

    std::stringstream tex_name;
    //tex_name << "material_" << i;
    tex_name << "material_" << "0";
    tex_name >> mesh_material.tex_name;

    //FIXME: hardcoded
    mesh_material.tex_file = "texture.png";

    tex_mesh.tex_materials.push_back (mesh_material);

  }// end meshes
  //TODO: hardcoded
  //
  // compute normals for the mesh
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

  PCL_INFO ("\nEstimating normals...\n");
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);

  // Concatenate XYZ and normal fields
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  PCL_INFO ("...Done.\n");

  pcl::toPCLPointCloud2 (*cloud_with_normals, tex_mesh.cloud);

  PCL_INFO ("\nSaving mesh to textured_mesh.obj\n");
  saveOBJFile("output.obj", tex_mesh, 5);


  return (0);
}
