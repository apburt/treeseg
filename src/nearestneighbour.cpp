//Andrew Burt - a.burt@ucl.ac.uk

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "treeseg.h"

int main (int argc, char** argv)
{
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	float zstep = atof(argv[1]);
	int nnearest = atoi(argv[2]);
	reader.read(argv[3],*cloud);
	std::vector<std::vector<float>> nndata;
	nndata = dNNz(cloud,nnearest,zstep);
	for(int i=0;i<nndata.size();i++) std::cout << nndata[i][0] << " " << nndata[i][1] << std::endl;
	return 0;
}
