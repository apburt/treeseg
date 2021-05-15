# Stem segmentation

Next, the full stem of each identified tree is segmented from the larger-area point cloud as follows:

1. Shape fitting: RANSAC methods are used to fit a cylinder to each set of inliers output from `findstems`, to describe the orientation and radius of each stem.
2. Spatial filtering: pass-through filters (infinite cylinder) are applied to the larger-area point cloud, as driven by expanded versions of these cylinders, to generate sections of point cloud containing each stem and neighbouring vegetation.
3. Plane fitting: points are classified as ground returns and segmented, if they are inliers of the plane fitted to the bottom of each of these sections via [SACMODEL_NORMAL_PARALLEL_PLANE](https://pointclouds.org/documentation/classpcl_1_1_sample_consensus_model_normal_parallel_plane.html) (n.b., this model imposes conditions on both the inliers, and the plane itself).
4. Region-based segmentation: the remaining sections are organised into regions, based on the properties of their underlying surfaces, in order to classify each element of vegetation.

These steps are implemented in the `segmentstem` executable, which can be called:

```
cd ../stems/;
segmentstem 4 ../clusters/NOU11.cluster.?.pcd ../NOU11.tile.downsample.*.pcd;
```

Where the inputs are: i) smoothness (degrees), ii) the inliers of the 10 largest stems output from `findstems`, and iii) the downsampled tiles constituting the larger-area point cloud. The output stem point clouds can be viewed using:

```
pcl_viewer NOU11.stem.?.pcd
```

Which will look something similar to:

<img src="/doc/images/segmentstem.png" width="750">

Also output are intermediate diagnostic files. For example, the following visualises the section of point cloud generated for `NOU11.cluster.1.pcd`, after the spatial filtering, plane fitting and region-based segmentation have been applied:

```
pcl_viewer NOU11.c.ng.r.1.pcd
```

<img src="/doc/images/segmentstem_regions.png" height="600">

## Parameters

In addition to smoothness (described [here](tutorial_findstems.md#Parameters)), other important parameters have been hard-coded into `segmentstem`, which may require further experimentation when applying these methods to other lidar datasets:

1. Shape fitting: [nnearest](../src/segmentstem.cpp#L27).
2. Spatial filtering: [expansionfactor](../src/segmentstem.cpp#L35).
3. Plane fitting: [groundidx](../src/segmentstem.cpp#L50), [ground](../src/segmentstem.cpp#L54), [nnearest](../src/segmentstem.cpp#L56), [dthreshold](../src/segmentstem.cpp#L57), [nweight](../src/segmentstem.cpp#L57), [angle](../src/segmentstem.cpp#L57).
4. Region-based segmentation: [nnearest](../src/segmentstem.cpp#L68), [nneighbours](../src/segmentstem.cpp#L70), [nmin](../src/segmentstem.cpp#L70), [nmax](../src/segmentstem.cpp#L70) and [curvature](../src/segmentstem.cpp#L70).

## Errors of commission

Scenarios will arise where the outputs from `segmentstem` erroneously include neighbouring vegetation (e.g., `NOU.stem.0.pcd`). A potential approach to correcting this is to lower the value of smoothness (e.g., 3.5). However this can lead to errors of omission, which are more difficult to correct. At this point is is unnecessary to manually correct errors of commission, as this will be undertaken once the crown of each tree has been segmented in the final step.

## [Next: Crown segmentation - Part 1](tutorial_getcrownvolume.md)
