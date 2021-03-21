# Digital terrain model

The first main step of treeseg is to identify the individual trees in the larger-area point cloud. This is undertaken on a slice segmented from the larger-area point cloud along the xy-plane. To generate this slice at some defined height above-ground, whilst adjusting for topography, it is necessary to construct a digital terrain model from the larger-area point cloud. Both this DTM, and the slice can be generated using:

```
getdtmslice 2 2.5 3 6 NOU11.tile.downsample.*.pcd > NOU11.dtm.txt
```

Where the input parameters are: 1) the resolution of the DTM (m), 2) the percentile of the z-coordinates inside each DTM tile considered to constitute the ground (this is used, rather than the absolute minimum, to provide more robust results for noisy point clouds), 3) the minimum height of the slice above-ground (m), and 4) the maximum height of the slice above-ground (m).

The resulting slice can be viewed using:

```
pcl_viewer NOU11.slice.pcd;
```

Which should look something similar to:

<img src="/doc/images/slice.png" width="500">

## [Next: Identifying the stems](tutorial_findstems.md)
