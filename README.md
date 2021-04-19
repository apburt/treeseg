# treeseg

Extract individual trees from lidar point clouds

<img src="/doc/images/treeseg_cover.png" width="500">

## Table of contents

- [Overview](#overview)
- [Installation](#installation)
- [Usage](#usage)
- [Acknowledgements](#acknowledgements)
- [Authors](#authors)
- [Citing](#citing)
- [License](#license)

## Overview

treeseg has been developed to near-automatically extract tree-level point clouds from high-density larger-area lidar point clouds acquired in forests. A formal, albeit somewhat outdated description of the methods can be found in our [paper](https://doi.org/10.1111/2041-210X.13121).

## Installation

treeseg has been developed and tested on Ubuntu 20.04 LTS only, and is dependent on the following packages:

* Point Cloud Library (v1.10)
* Armadillo (v9.8)

These dependencies are installed via apt:

```
apt install libpcl-dev libarmadillo-dev
```

treeseg can then be installed using:

```
git clone https://github.com/apburt/treeseg.git;
mkdir ./treeseg/build; cd ./treeseg/build; cmake ..; make;
```

Optionally, for users with RIEGL V-Line scan data, treeseg includes the executable `rxp2pcd`, to convert and preprocess data in RXP data stream format, to binary PCD format. This executable will be automatically built if the directories `./treeseg/include/reigl/` and `./treeseg/lib/riegl/` are populated with the RIEGL RiVLIB headers and libraries (as appropriate for the user's particular CPU architecture and gcc version), which can be downloaded from the Members Area of the RIEGL website (e.g., rivlib-2_5_10-x86_64-linux-gcc9.zip). 

Finally, the environment variable `PATH` can then be updated to include the directory containing the built treeseg executables, either temporarily, by calling the following, or permanently, by inserting it at the top of `~/.bashrc`:

```
export PATH="$PATH:/path/to/treeseg/build"
```

## Usage

A tutorial demonstrating the usage of treeseg is available [here](/doc/tutorial_overview.md).

## Acknowledgements

treeseg makes extensive use of the Point Cloud Library ([PCL](http://pointclouds.org)).

## Authors

* Andrew Burt
* Mathias Disney
* Kim Calders
* Matheus Boni Vicari
* Tony Peter

## Citing

treeseg can be cited as:

Burt, A., Disney, M., Calders, K. (2019). Extracting individual trees from lidar point clouds using *treeseg*. *Methods Ecol Evol* 10(3), 438–445. doi: 10.1111/2041-210X.13121

A doi for the latest version is available in [releases](https://github.com/apburt/treeseg/releases).

## License

This project is licensed under the terms of the MIT license - see the [LICENSE](LICENSE) file for details.
