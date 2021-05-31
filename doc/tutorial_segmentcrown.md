# Crown segmentation - Part 2

The second step in the process of extracting the crowns uses the following methods:

1. Region-based segmentation: the point clouds output from `getcrownvolume` are organised into regions, based on the properties of their underlying surfaces.
2. Connectivity testing: the tree is built from the stem up, by adding nearby regions considered belonging to the tree.

These methods are implemented in the `segmentcrown` executable, which can be called:

```
cd ../trees/;
segmentcrown 4 ../volumes/NOU11.volume.*.pcd;
```

Where the input parameters are: i) smoothness (degrees), and ii) the point clouds output from `getcrownvolume`. Depending on point cloud size and computing resources, this currently single-threaded process can take between 12-24 hours to complete. It might therefore be worthwhile running multiple instances in parallel.

The output tree-level point clouds can then be viewed using:

```
pcl_viewer NOU11_1.pcd
```

Which will look something similar to:

<img src="/doc/images/segmentcrown.png" height="500">

# The buildtree function

The connectivity testing is implemented in the [buildtree](../src/treeseg.cpp#L963) function, and called by `segmentcrown` as:

```
buildTree(regions,20,1,0.2,5,1.0,tree);
```

Where the input parameters are: i) the vector of point clouds derived from the region-based segmentation, ii) cyclecount (integer), iii) firstcount (integer), iv) firstdist (m), v) nnearest (integer), vi) seconddist (m), and vii) the resulting tree-level point cloud. That is, first, the region representing the stem is designated the seed. For `firstcount` iteration(s), regions within the distance `firstdist` to the seed(s) are added to the tree and designated the new seeds. For the remaining `cyclecount` iterations, the `nnearest` regions closest to each seed and within the distance `seconddist` are added to the tree and designated the new seeds. These parameters may require further experimentation when applied to other lidar datasets. For example, for very high-quality lidar data, or for more sparsely populated forest scenes, it might be possible to significantly reduce `cyclecount`. This will substantially reduce compute time.

In addition to this function, other important parameters have been hard-coded into `segmentcrown`:

1. Region-based segmentation: [nnearest](../src/segmentcrown.cpp#L24), [nneighbours](../src/segmentcrown.cpp#L25), [nmin](../src/segmentcrown.cpp#L25), [nmax](../src/segmentcrown.cpp#L25) and [curvature](../src/segmentcrown.cpp#L25).

# Errors of commission

Errors of commission in the final tree-level point clouds can then be manually segmented in CloudCompare, e.g.,:

```
pcdPointTreeseg2txt NOU11_1.pcd;
CloudCompare NOU11_1.txt;
```
