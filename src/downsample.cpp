//Andrew Burt - a.burt@ucl.ac.uk

#include "treeseg.h"

#include <pcl/io/pcd_io.h>

int main (int argc, char *argv[])
{
	float edgelength = atof(argv[1]);
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	pcl::PointCloud<PointTreeseg>::Ptr original(new pcl::PointCloud<PointTreeseg>);
	pcl::PointCloud<PointTreeseg>::Ptr filtered(new pcl::PointCloud<PointTreeseg>);
	std::stringstream ss;
	for(int i=2;i<argc;i++)
	{
		reader.read(argv[i],*original);
		std::vector<std::string> id = getFileID(argv[i]);
		downsample(original,edgelength,filtered);
		ss.str("");
		ss << id[0] << ".tile.downsample."  << id[1] << ".pcd"; 
		writer.write(ss.str(),*filtered,true);
		original->clear();	
		filtered->clear();
	}
	return 0;
}
