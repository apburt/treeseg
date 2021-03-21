# Downsampling

Downsampling the data is optional, but provides two potential benefits: i) some of the algorithms used in treeseg (e.g., Euclidean clustering) will provide more consistent results, and ii) reduces computation. However, determining the appropriate level of downsampling is a complicated subject, requiring careful consideration (e.g., the downstream implications of downsampling, such as the impacts on above-ground biomass estimation), which is briefly discussed in the following subsection.

treeseg implements two methods for downsampling, both employing octrees. The first, and the one used in this tutorial, replaces the points inside each occupied voxel (leaf-node), with a single aggregate point:  

```
downsample 0.04 NOU11.tile.*.pcd 
```

Where 0.04 is the edge length of each voxel, in meters. This will output downsampled point clouds for each tile, e.g., NOU11.tile.downsample.342.pcd, whose point count has reduced by approximately 92% compared with its original counterpart.

The alternative approach, thin, preserves the original data, rather than replacing it, by keeping the original point in each voxel that is closest to the would-be aggregate point. This might be useful when [additional fields](tutorial_preprocessing.md#Parsing-additional-point-attributes) are parsed (e.g., reflectance), and their aggregation would not make sense. thin can be similarly called as:

```
thin 0.04 NOU11.tile.*.pcd 
```

Note: if the preprocessor macro [XYZRRDRS](../include/treeseg_pointtype.h#L29) is defined as true, the behaviour of thin is modified. Instead, the point with the smallest deviation value (i.e., a proxy for measurement goodness) will be preserved (if multiple points with the same deviation exist in a voxel, the nearest to the would-be aggregate is selected). This behaviour can be changed in [treeseg.cpp](../src/treeseg.cpp#L343).

## Downsampling resolution

A potential place to start when determining the downsampling resolution, is to consider the characteristics of the lidar instrument (i.e., beam divergence, diameter of the beam at emission, and ranging accuracy), and to compare them with the observed distances between individual points in the point cloud. For the data considered in this tutorial, this exercise was undertaken in figure 2 of our [paper](https://doi.org/10.1111/2041-210X.13121), where it can be seen that a voxel edge length of 0.04 m provided some balance between removing redundant points close to the ground (e.g., points on the stems where the beam overlapped), whilst conserving important data from the canopy.

treeseg includes the executable nearestneighbour, which may help in making this decision, by outputting the mean vertically-resolved nearest neighbour distances through a particular point cloud (i.e., the cloud is partitioned into height bins, and the mean distance of each point to its N nearest neighbours is calculated and averaged for each bin): 

```
nearestneighbour 2 6 NOU11.tile.342.pcd 
```

Where 2 is the width of the bins, in meters, and 6 is the number of nearest neighbours to be considered. 

## [Next: Digital terrain model](tutorial_getdtmslice.md)
