# Tutorial

The objective of this tutorial is to demonstrate the usage of treeseg, by extracting 10 of largest trees inside a complex forest scene.

## Primer

If you plan to extensively use or develop treeseg, it might first be useful to familiarise yourself with the various point cloud processing methods that are used. A brief overview of these methods can be found in our [paper](https://doi.org/10.1111/2041-210X.13121).

treeseg uses the Point Cloud Libray (PCL) to implement these methods, and the PCL [website](https://pointclouds.org/) provides good generic tutorials. The most relevant are: Basic Usage, Features, Filtering, KdTree, Octree, Sample Consensus and Segmentation.

## Prerequisites

It is assumed treeseg has been successfully built. Several other packages are also required, which can be installed via apt:

```
apt install pcl-tools cloudcompare 
```

## Data

The terrestrial lidar data that will be downloaded and used in this tutorial were collected from a one hectare stand of old-growth intact tropical rainforest in French Guiana. A more complete description of these data is available via [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.4497751.svg)](https://doi.org/10.5281/zenodo.4497751)

### Directory structure

It is assumed the following folders have been created:

```
./treeseg_tutorial/
├───data/
└───processing/
    ├───clusters/
    ├───stems/
    ├───volumes/
    ├───trees/
```

### Option 1 (recommended): Raw lidar data

If the rxp2pcd executable has been compiled (see [here](../README.md#installation)), then the raw lidar data should be downloaded:

```
cd ./treeseg_tutorial/data/;
wget https://zenodo.org/record/4497751/files/2015-11-10.001.riproject.part1.zip;
wget https://zenodo.org/record/4497751/files/2015-11-10.001.riproject.part2.zip;
wget https://zenodo.org/record/4497751/files/2015-11-10.001.riproject.part3.zip;
wget https://zenodo.org/record/4497751/files/2015-11-10.001.riproject.part4.zip;
wget https://zenodo.org/record/4497751/files/2015-11-10.001.riproject.part5.zip;
unzip '2015-11-10.001.riproject.part?.zip';
```

### Option 2: Preprocessed lidar data

If the rxp2pcd executable could not been compiled (e.g., it was not possible to access and download the RIEGL RiVLIB headers and libraries), then the preprocessed data should be downloaded. It is also necessary to create a file that contains the coordinates of plot boundaries (discussed further in the following section):

```
cd ./treeseg_tutorial/processing/;
wget https://zenodo.org/record/4661301/files/2015-11-10.001.preprocessed.part1.zip;
wget https://zenodo.org/record/4661301/files/2015-11-10.001.preprocessed.part2.zip;
wget https://zenodo.org/record/4661301/files/2015-11-10.001.preprocessed.part3.zip;
wget https://zenodo.org/record/4661301/files/2015-11-10.001.preprocessed.part4.zip;
wget https://zenodo.org/record/4661301/files/2015-11-10.001.preprocessed.part5.zip;
unzip '2015-11-10.001.preprocessed.part?.zip';
echo -101.261 4.48606 -104.789 0 > NOU11.coords.dat;
```

## [Next: Preprocessing](tutorial_preprocessing.md)
