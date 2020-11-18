//Andrew Burt - a.burt@ucl.ac.uk

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "treeseg.h"

int main (int argc, char **argv)
{
	int nnearest = atof(argv[1]);
	float zstep = atof(argv[2]);
	float diffmax = atof(argv[3]);
	float density = atof(argv[4]);
	pcl::PCDReader reader;
	for(int i=5;i<argc;i++)
	{
		std::vector<std::string> id = getFileID(argv[i]);
		pcl::PointCloud<pcl::PointXYZ>::Ptr tree(new pcl::PointCloud<pcl::PointXYZ>);
		reader.read(argv[i],*tree);
		treeparams params = getTreeParams(tree,nnearest,zstep,diffmax);
		std::cout << id[1] << "_" << id[0] << " " << params.x << " " << params.y << " " << params.d << " " << params.h << " " << density << std::endl;
	}
	return 0;
}

