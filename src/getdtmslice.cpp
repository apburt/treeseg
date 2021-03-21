//Andrew Burt - a.burt@ucl.ac.uk

#include "treeseg.h"

#include <pcl/io/pcd_io.h>

int main(int argc, char **argv)
{
	std::vector<std::string> args(argv+1,argv+argc);
	float resolution = std::stof(args[0]);
	float percentile = std::stof(args[1]);
	float zmin = std::stof(args[2]);
	float zmax = std::stof(args[3]);
	pcl::PointCloud<PointTreeseg>::Ptr plotcloud(new pcl::PointCloud<PointTreeseg>);
	pcl::PCDWriter writer;
	std::vector<std::string> id = getFileID(args[4]);
	readTiles(args,plotcloud);
	std::stringstream ss;
	ss.str("");
	ss << id[0] << ".slice.pcd";
	std::vector<std::vector<float>> dem;
	pcl::PointCloud<PointTreeseg>::Ptr slice(new pcl::PointCloud<PointTreeseg>);
	dem = getDtmAndSlice(plotcloud,resolution,percentile,zmin,zmax,slice);
	for(int j=0;j<dem.size();j++) std::cout << dem[j][0] << " " << dem[j][1] << " " << dem[j][2] << std::endl;
	writer.write(ss.str(),*slice,true);
	return 0;
}
