# treeseg

Extract individual trees from lidar point clouds

<img src="https://drive.google.com/uc?export=view&id=1oZNzTAbH80MxEywqH8y3RCZgzn3sbYsy" width="500">

## Overview

treeseg has been developed to near-automatically extract tree-level point clouds from high-density larger-area lidar point clouds acquired in forests.
A description of the methods can be found in our Methods in Ecology and Evolution applications paper available [here](https://besjournals.onlinelibrary.wiley.com/doi/abs/10.1111/2041-210X.13121)

Briefly, libtreeseg provides a set of generic functions that have been implemented into a series of executables to: i) identify individual stems in the larger-area point cloud (findstems), ii) extract each of these identified stems up to the first order of branching (segmentstem), and iii) segment each individual crown from neighbouring vegetation (segmentcrown).

Integrated into treeseg is libleafsep, a library providing methods to separate wood and leaf returns in unorganised point clouds. 
libleafsep is a modified translation of TLSeparation (https://github.com/TLSeparation).

## Prerequisites

treeseg has been developed using:

* Point Cloud Library (PCL) (http://pointclouds.org) (v1.9 or later)
* Armadillo (http://arma.sourceforge.net) (v9.6 or later)

On Ubuntu 19.10, these dependencies can be installed via:

```
apt install git cmake
apt install libpcl-dev
apt install libarmadillo-dev
```

## Installation

Install libtreeseg and the associated binaries as follows:

```
git clone https://github.com/apburt/treeseg.git;
mkdir treeseg/build; cd treeseg/build;
cmake ..; make;
```

Also included in treeseg is rxp2pcd, for conversion of REIGL V-Line scan data to .pcd binary format. Compiling this executable requires RiVLIB headers and libraries to be placed in the /treeseg/include/riegl/ and /treeseg/lib/riegl/ directories respectively (these can be downloaded from the Members Area of the RIEGL website, e.g., RiVLIB 2.5.7 64bit Linux gcc55)

## Usage

Below is an example usage of the treeseg binaries:

* plotcoords ../matrix/ > FGC01_coords.dat
* rxp2pcd ../ FGC01_coords.dat 25 15 FGC01
* downsample 0.04 0 FGC01.tile.*.pcd
* getdemslice 2 2.5 3 6 FGC01.tile.downsample.*.pcd
* findstems 5 0.2 2 ../FGC01_coords.dat ../FGC01.slice.pcd
* segmentstem 5 ../clusters/FGC01.cluster.*.pcd ../FGC01.tile.downsample.*.pcd
* getcrownvolume ../stems/FGC01.stem.*.pcd ../FGC01.tile.downsample.*.pcd 
* segmentcrown 5 0 ../FGC01.volume.*.pcd

## Authors

* **Andrew Burt**
* **Mathias Disney**
* **Kim Calders**
* **Matheus Boni Vicari**
* **Tony Peter**

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details

## Acknowledgements

* treeseg makes use of the Point Cloud Library (PCL) (http://pointclouds.org) and Armadillo (http://arma.sourceforge.net).
