Point Cloud Tabletop Object Segmentation and Projection

This software uses Point Cloud Library (PCL) to filter a point cloud of objects on a tabletop.
The goal is to segment the object from the tabletop and project to object's points onto the removed tabletop.

Operating System: Ubuntu 16.04
PCL Version: 1.8

Point Cloud data sets are zipped in dataSets.zip, so first unzip this:
unzip dataSets.zip

To compile:
cd Point-Cloud-Ground-Segmentation/src/
mkdir build/
cd build/
CMake
make

The executable is located in Point-Cloud-Ground-Segmentation/src/build/planar_segmentation
To run from the build directory:
./planar_segmentation ../../dataSets/[Point Cloud of Objects on Tabletop].pcd
Do not run on *_obj.pcd or *_proj.pcd as these are the outputs of the software.

Inputs:
learn0.pcd
learn22.pcd
test18.pcd
table_scene_mug_stereo_textured.pcd

Outputs:
Outputs are saved in dataSets/
*_obj.pcd: Isolated object after tabletop Point Cloud is removed.
*_proj.pcd: Projection of isolated object onto plane defined by the tabletop.
