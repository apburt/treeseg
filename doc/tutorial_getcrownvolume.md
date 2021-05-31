# Crown segmentation - Part 1

At this point, the stems of the 10 largest trees inside the 1 ha stand have been successfully segmented. It is now necessary to extract their crowns from the neighbouring vegetation. The first part of this two step process is to generate a point cloud for each tree, containing: i) the stem, ii) the crown, and iii) neighbouring crown vegetation. This is undertaken using the following methods:

1. Shape fitting: RANSAC methods are used to fit a cylinder to each of the stem point clouds, to estimate the stem diameter of each tree.
2. Spatial filtering: pass-through filters (closed cylinder) are applied to the larger-area point cloud, as driven by stem-diameter:tree-height and stem-diameter:crown-diameter allometries, to generate point clouds containing the crown of each tree.
3. Duplicate point removal: the stem and crown point clouds are concatenated, and duplicate points removed.

These steps are implemented in the `getcrownvolume` executable, which can be called:

```
cd ../volumes/;
getcrownvolume 0.66 ../stems/NOU11.stem.*.pcd ../NOU11.tile.downsample.*.pcd;
```

Where the inputs are: i) the height along each stem defining the bottom of the pass-through cylinder, ii) the stem point clouds output from `segmentstem`, and iii) the downsampled tiles constituting the larger-area point cloud. The output point clouds can be viewed using:

```
pcl_viewer NOU11.volume.0.pcd
```

Which will look something similar to:

<img src="/doc/images/getcrownvolume.png" height="500">

## Allometrics

The purpose of the [stem-diameter:tree-height](../src/getcrownvolume.cpp#L7) and [stem-diameter:crown-diameter](../src/getcrownvolume.cpp#L15) allometries, is to segment an appropriately sized section of the larger-area point cloud containing the entirety of the crown of the tree of interest. These allometries were constructed using terrestrial lidar data collected across the tropics via a non-linear power-law model. The parameters used in these models are the upper 95% confidence intervals, and also include a further relaxation term. When applied to other lidar datasets, it may be necessary to relax these allometries further, to avoid accidental cropping of crowns.

## [Next: Crown segmentation - Part 2](tutorial_segmentcrown.md)
