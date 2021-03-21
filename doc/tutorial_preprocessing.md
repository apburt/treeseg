# Preprocessing

## The larger-area point cloud

treeseg requires some larger-area point cloud that was collected in a forest scene (e.g., in this tutorial, the point cloud representing the 1 ha tropical rainforest plot in French Guiana). It is expected that this point cloud is stored in binary or ascii [PCD format](https://pcl.readthedocs.io/projects/tutorials/en/latest/pcd_file_format.html). By default, the expected point fields are the Cartesian coordinates (x, y, z) of each point, but additional fields can be included as described in the following subsection. It is also expected that the larger-area point cloud is broken into N tiles (minimum of one), using the following strict naming convention:

```
mycloud.tile.0.pcd
mycloud.tile.1.pcd
...
mycloud.tile.N.pcd
```

Where 'mycloud' can be replaced with some meaningful identifier of the larger-area point cloud. Depending on the computing environment and resources, it will often be necessary for each tile to have a maximum size of a few gigabytes, to circumvent issues that potentially arise from requiring large contiguous blocks of unoccupied RAM to be available when loading large point clouds.

## Parsing additional point attributes

treeseg implements a custom point type, PointTreeseg, to provide the user with flexibility over the point fields. By default PointTreeseg = [pcl::PointXYZ](https://pointclouds.org/documentation/structpcl_1_1_point_x_y_z.html). However, the definition of PointTreeseg in [treeseg_pointtype.h](../include/treeseg_pointtype.h#L33) can be modified if the user wishes to include other fields (e.g., reflectance).  

An example of how this can be implemented is provided by the preprocessor macro XYZRRDRS in [treeseg_pointtype.h](../include/treeseg_pointtype.h#L29). If this macro is defined as true (default: false) and treeseg recompiled, then the point fields become: x (m), y (m), z (m), range (m), reflectance (dB), deviation (#), return number (#) and scan number (#). These additional attributes will then be preserved through the various processing steps of treeseg.

## Converting RXP to PCD

In this tutorial we only consider conversion of lidar data stored in RIEGL's RXP data stream format. However, various open-source tools are available for converting lidar data in other formats (e.g., LAS) to PCD, in preparation for using treeseg.

The lidar data downloaded during the previous step of this tutorial are converted into the required tiled PCD format using the rxp2pcd executable. The overall x-y limits of the tiles are defined by the coordinates of a rectangle, which here, we want dictated by the plot boundaries, which can be approximated from the scan locations via the transformation matrices in ./treeseg_tutorial/data/matrix/ using plotcoords:

```
cd ./treeseg_tutorial/processing/;
plotcoords ../data/matrix/ > NOU11_coords.dat;
```

Whereby the resulting ASCII file NOU11_coords.dat contains:
 
```
-101.261 4.48606 -104.789 0
```

I.e., the minimum and maximum coordinates of the 121 scan locations (xmin, xmax, ymin, ymax), in meters.

rxp2pcd can then be called as:

```
rxp2pcd ../data/ NOU11_coords.dat 25 15 NOU11;
```

Where the input parameters are: 1) the top directory containing the lidar data; 2) the coordinates of the plot boundaries; 3) the area of each output tile (m2); 4) the maximum permitted value of point deviation (used to remove noisy returns); and 5) the larger-area point cloud identifier. 

This command will output the larger-area point cloud into 870 PCD tiles: NOU11.tile.0.pcd to NOU11.tile.869.pcd. The overall x-y limits of the tiles are increased by 20 m in each direction to ensure data relating to the trees of interest are not truncated (e.g., the crowns of large trees at the edges of the plot). 

These tiles can be viewed using pcl_viewer, e.g.,:

```
pcl_viewer NOU11.tile.342.pcd;
```

## [Next: Downsampling](tutorial_downsample.md)
