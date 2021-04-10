# Identifying the stems

Individual tree stems are identified in the slice of larger-area point cloud using four steps:

1. Euclidean clustering: the slice is organised into individual clusters, based on the spatial distances between points.  
2. Region-based segmentation: these clusters are further reduced to regions, based on the properties of their underlying surfaces (themselves inferred from point normals).
3. Shape fitting: RANSAC methods are used to fit cylinders to each region, and fit diagnostics are used determine the likelihood of the underlying surface being a stem.
4. Principal component analysis: the angle between the principal components of each region and the ground are calculated, with regions broadly perpendicular to the ground deemed to be stems.

These steps are implemented using the findstems executable, which can be called:

```
cd ./clusters/;
findstems 4 0.2 2 ../NOU11.coords.dat ../NOU11.slice.pcd;
```

Where the inputs are: i) smoothness (degrees, see the following subsection), ii) minimum stem diameter (m), iii) maximum stem diameter (m), iv) the coordinates of the plot boundaries, and v) the slice of point cloud segmented from the larger-area point cloud along the xy-plane. That is, regions passing steps 1-4 are considered stems if their fitted cylinders have a diameter between or equal to the minimum and maximum diameter, and their centre resides inside or on the bounding box specified by the plot boundaries.

The output point clouds, NOU11.cluster.0.pcd to NOU11.cluster.189.pcd, contain the inliers of each cylinder fit (ordering: descending by diameter). Also output are intermediate files useful for determining the source of any errors. For example, the following visualises the slice after applying Euclidean clustering and region-based segmentation: 

```
pcl_viewer NOU11.intermediate.slice.clusters.regions.pcd
```

Which looks something similar to:

<img src="/doc/images/slice_regions.png" width="500">

## Parameters

The input parameter smoothness is not particularly interpretable: it is used in the region-based segmentation to define the maximum permitted angle between point normals, for points to be considered belonging to the same region or underlying surface. However, it is important to select an appropriate value of smoothness, which unfortunately, depends on factors including the lidar instrument, the sampling pattern, and the scene itself. Therefore, some experimentation is required: the idea is that each resulting region wholly represents an individual stem (as shown in the figure above). If the value is too large, stems and surrounding vegetation will be lumped into the same region, and if too small, each stem will comprise of multiple regions.

In addition to smoothness, other important parameters have been hard-coded into findstems, which may require further experimentation when applying these methods to other lidar datasets:

1. Euclidean clustering: [nnearest](../src/findstems#L24) and [nmin](../src/findstems#L25).
2. Region-based segmentation: [nnearest](../src/findstems#L40), [nneighbours](../src/findstems#L41), [nmin](../src/findstems#L41), [nmax](../src/findstems#L41) and [curvature](../src/findstems#L41).
3. Shape fitting: [nnearest](../src/findstems#L52), [lmin](../src/findstems#L72), [stepcovmax](../src/findstems#L73) and [radratiomin](../src/findstems#L74).
4. Principal component analysis: [anglemax](../src/findstems#L102).

## Errors of omission and commission

There will be scenarios where findstems either incorrectly identifies a region as a stem, or not a stem. So it can be useful to undertake a manual check, by comparing the original slice with the extracted inliers:

```
pcdPointTreeseg2txt NOU11.intermediate.slice.clusters.regions.pcd NOU11.cluster.*.pcd
CloudCompare NOU11.intermediate.slice.clusters.regions.txt NOU11.cluster.*.txt;
```

Whereby the slice should be coloured different to the inliers for ready comparison. Errors of commission (i.e., a region incorrectly identified as a stem) can be deleted (e.g., here, by removing the file NOU11.cluster.147.pcd). Errors of omission (i.e., a region incorrectly identified as not a stem), can be corrected by manually segmenting missed stems from the slice using CloudCompare, and saving these additional point clouds (e.g., using the naming convention: NOU11.cluster.190.txt onwards). Note: it is then necessary convert these newly created ASCII point clouds to PCD format, which can be undertaken using the txtPointTreeseg2pcd executable.

## [Next: Stem segmentation](tutorial_segmentstem.md)
