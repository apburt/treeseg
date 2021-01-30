//Andrew Burt - a.burt@ucl.ac.uk

#include "treeseg.h"

#include <pcl/io/pcd_io.h>

int main(int argc, char **argv)
{
	std::vector<std::string> args(argv+1,argv+argc);
	float edgelength = std::stof(args[0]);
	bool otree = true;
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	pcl::PointCloud<PointTreeseg>::Ptr original(new pcl::PointCloud<PointTreeseg>);
	pcl::PointCloud<PointTreeseg>::Ptr downsampled(new pcl::PointCloud<PointTreeseg>);
	for(int i=1;i<args.size();i++)
	{
		std::vector<std::string> id = getFileID(args[i]);
		std::stringstream ss;
		ss << id[0] << ".tile.thin." << id[1] << ".pcd";
		reader.read(args[i],*original);
		thin(original,edgelength,downsampled);
		writer.write(ss.str(),*downsampled,true);
		original->clear();
		downsampled->clear();
	}
	return 0;
}
