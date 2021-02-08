# Tutorial

The objective of this tutorial is to explain and demonstrate the usage of treeseg, by extracting the 10 largest trees inside a complex tropical forest scene.

## Primer

If you plan to extensively use or develop treeseg, familiarisation with the various point cloud processing methods used in treeseg might be useful. A brief overview of these methods can be found in our paper available [here](https://doi.org/10.1111/2041-210X.13121).

treeseg uses the Point Cloud Libray (PCL) to implement these methods, and the PCL [website](https://pointclouds.org/) provides good tutorials. The most relevant are: Basic Usage, Features, Filtering, KdTree, Octree, Sample Consensus and Segmentation.

## Data

The terrestrial lidar data used in this tutorial were collected from one hectare of old-growth intact tropical rainforest in French Guiana. These data, and a more complete description, are available from: [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.4497751.svg)](https://doi.org/10.5281/zenodo.4497751)

It is assumed the following directory structure has been created:

```
./treeseg_tutorial/
├───data/
└───processing/
    ├───clusters/
    ├───stems/
    ├───volumes/
    ├───trees/
```

The lidar data can then be downloaded and unpacked using:

```
cd ./treeseg_tutorial/data/;
wget https://zenodo.org/record/4497751/files/2015-11-10.001.riproject.part1.zip;
wget https://zenodo.org/record/4497751/files/2015-11-10.001.riproject.part2.zip;
wget https://zenodo.org/record/4497751/files/2015-11-10.001.riproject.part3.zip;
wget https://zenodo.org/record/4497751/files/2015-11-10.001.riproject.part4.zip;
wget https://zenodo.org/record/4497751/files/2015-11-10.001.riproject.part5.zip;
unzip '2015-11-10.001.riproject.part?.zip';
```

## Prerequisites

This tutorial assumes treeseg has been successfully compiled, including the rxp2pcd executable (see [here](../README.md#installation)).

Several other packages are also required, which can be installed via apt:

```
apt install pcl-tools cloudcompare; 
```

## [Next: Preprocessing](tutorial_preprocessing.md)
