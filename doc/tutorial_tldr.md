# TL;DR

```
mkdir treeseg_tutorial; cd treeseg_tutorial/; mkdir data processing; cd data/;
wget https://zenodo.org/record/4497751/files/2015-11-10.001.riproject.part1.zip;
wget https://zenodo.org/record/4497751/files/2015-11-10.001.riproject.part2.zip;
wget https://zenodo.org/record/4497751/files/2015-11-10.001.riproject.part3.zip;
wget https://zenodo.org/record/4497751/files/2015-11-10.001.riproject.part4.zip;
wget https://zenodo.org/record/4497751/files/2015-11-10.001.riproject.part5.zip;
unzip '2015-11-10.001.riproject.part?.zip';
```

```
cd ../processing/;
mkdir clusters stems volumes trees;
plotcoords ../data/matrix/ > NOU11.coords.dat;
rxp2pcd ../data/ NOU11.coords.dat 25 15 NOU11;
downsample 0.04 NOU11.tile.*.pcd;
getdtmslice 2 2.5 3 6 NOU11.tile.downsample.*.pcd > NOU11.dtm.dat;
```

```
cd ./clusters/;
findstems 4 0.2 2 ../NOU11.coords.dat ../NOU11.slice.pcd;
cd ../stems/;
segmentstem 4 ../clusters/NOU11.cluster.?.pcd ../NOU11.tile.downsample.*.pcd;
cd ../volumes/;
getcrownvolume 0.66 ../stems/NOU11.stem.*.pcd ../NOU11.tile.downsample.*.pcd;
cd ../trees/;
segmentcrown 4 0 ../volumes/NOU11.volume.*.pcd;
```
