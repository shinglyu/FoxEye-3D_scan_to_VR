#===Downsampling===
#pcl_voxel_grid learn1.pcd learn1_downsample.pcd -leaf 0.005
pcl_voxel_grid learn1.pcd learn1_downsample.pcd -leaf 0.001
#cp learn1.pcd learn1_downsample.pcd
#----------------------
#pcl_viewer learn1.pcd
pcl_viewer learn1_downsample.pcd
#----------------------

#===Normal Estimation===
pcl_normal_estimation learn1_downsample.pcd learn1_downsample_normal.pcd -radius 0.01
#----------------------
#pcl_pcd2ply learn1_downsample_normal.pcd learn1_downsample_normal.ply
pcl_viewer -normals 5 learn1_downsample_normal.pcd
#----------------------


#===Marching Cube Surface Reconstruction===
#pcl_marching_cubes_reconstruction learn1_downsample_normal.pcd learn1_downsample_normal_marchingcube.vtk 
#----------------------
#pcl_vtk2ply learn1_downsample_normal_marchingcube.vtk learn1_downsample_normal_marchingcube.ply
#meshlab learn1_downsample_normal_marchingcube.ply learn1_downsample_normal.ply
#pcl_viewer learn1_downsample_normal_marchingcube.vtk
#----------------------

#===Poisson Surface Reconstruction===
#----------------------
pcl_poisson_reconstruction learn1_downsample_normal.pcd learn1_downsample_normal_poisson.vtk
pcl_vtk2ply learn1_downsample_normal_poisson.vtk learn1_downsample_normal_poisson.ply
meshlab learn1_downsample_normal_poisson.ply
#----------------------

#===Hull Surface Reconstruction===
#----------------------
#pcl_compute_hull -alpha 0.01 learn1_downsample_normal.pcd learn1_downsample_normal_hull.vtk
#pcl_vtk2ply learn1_downsample_normal_hull.vtk learn1_downsample_normal_hull.ply
#meshlab learn1_downsample_normal_hull.ply
#----------------------

#===Transfer color from point cloud to mesh
../transfer_vertex_color/transfer_vertex_color learn1_downsample.pcd learn1_downsample_normal_poisson.vtk learn1_downsample_normal_poisson_color.ply
meshlab learn1_downsample_normal_poisson_color.ply

#===Export to obj===
pcl_vtk2obj learn1_downsample_normal_poisson.vtk learn1_downsample_normal_poisson.obj
