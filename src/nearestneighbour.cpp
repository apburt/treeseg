#include "treeseg.h"

#include <pcl/io/pcd_io.h>

int main(int argc, char **argv)
{
	std::vector<std::string> args(argv+1,argv+argc);
	pcl::PCDReader reader;
	pcl::PointCloud<PointTreeseg>::Ptr cloud(new pcl::PointCloud<PointTreeseg>);
	float zstep = std::stof(args[0]);
	int nnearest = std::stoi(args[1]);
	reader.read(args[2],*cloud);
	std::vector<std::vector<float>> nndata;
	nndata = dNNz(cloud,nnearest,zstep);
	for(int i=0;i<nndata.size();i++) std::cout << nndata[i][0] << " " << nndata[i][1] << std::endl;
	return 0;
}
