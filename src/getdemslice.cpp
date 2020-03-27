//Andrew Burt - a.burt@ucl.ac.uk

#include "treeseg.h"

#include <pcl/io/pcd_io.h>

int main (int argc, char *argv[])
{
	float resolution = atof(argv[1]);
	float zmin = atof(argv[2]);
	float zmax = atof(argv[3]);
	pcl::PointCloud<PointTreeseg>::Ptr plotcloud(new pcl::PointCloud<PointTreeseg>);
	pcl::PCDWriter writer;
	readTiles(argc,argv,plotcloud);
	std::vector<std::string> id = getFileID(argv[4]);
	std::stringstream ss;
	ss.str("");
	ss << id[0] << ".slice.pcd";
	std::vector<std::vector<float>> dem;
	pcl::PointCloud<PointTreeseg>::Ptr slice(new pcl::PointCloud<PointTreeseg>);
	dem = getDemAndSlice(plotcloud,resolution,zmin,zmax,slice);
	for(int j=0;j<dem.size();j++) std::cout << dem[j][0] << " " << dem[j][1] << " " << dem[j][2] << std::endl;
	writer.write(ss.str(),*slice,true);
	return 0;
}
