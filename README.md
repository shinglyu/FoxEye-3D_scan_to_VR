# FoxEye Project
## 3D scanned point cloud to WebVR 

# Build
Prerequisite: CMake 

```
build.sh
```
It will install PCL and MeshLab

# Usage

```
cd point_cloud_to_mesh
./run.sh
```

This will transform the `learn1.pcd` example file from point cloud to colored mesh (`learn1_downsample_normal_poisson_color.ply`)

#Screenshots
Source point cloud
![Point Cloud](https://raw.githubusercontent.com/shinglyu/FoxEye-3D_scan_to_VR/master/doc/point_cloud.png)
Source point cloud closeup
![Point Cloud](https://raw.githubusercontent.com/shinglyu/FoxEye-3D_scan_to_VR/master/doc/point_cloud_closeup.png)
Estimated Normals
![Normals](https://raw.githubusercontent.com/shinglyu/FoxEye-3D_scan_to_VR/master/doc/normals.png)
Smooth reconstructed surface using Poisson reconstruction
![Poisson Reconstruction](https://raw.githubusercontent.com/shinglyu/FoxEye-3D_scan_to_VR/master/doc/colorless_mesh.png)
Reconstructed mesh with vertex color
![Color Transfered Mesh](https://raw.githubusercontent.com/shinglyu/FoxEye-3D_scan_to_VR/master/doc/colored_mesh.png)
Smoothed rendering of the colored mesh
![Smooth Render](https://raw.githubusercontent.com/shinglyu/FoxEye-3D_scan_to_VR/master/doc/colored_mesh_smooth.png)
