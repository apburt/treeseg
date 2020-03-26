//Andrew Burt - a.burt@ucl.ac.uk

#include "treeseg.h"

#include <pcl/io/pcd_io.h>

int main (int argc, char *argv[])
{
	float edgelength = atof(argv[1]);
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	pcl::PointCloud<PointTreeseg>::Ptr original(new pcl::PointCloud<PointTreeseg>);
	pcl::PointCloud<PointTreeseg>::Ptr downsampled(new pcl::PointCloud<PointTreeseg>);
	pcl::PointCloud<PointTreeseg>::Ptr thinned(new pcl::PointCloud<PointTreeseg>);
	std::stringstream ss;
	for(int i=2;i<argc;i++)
	{
		reader.read(argv[i],*original);
		downsample(original,edgelength,downsampled);
		std::vector<int> idxs = nearestIdx(downsampled,original);
		for(int j=0;j<idxs.size();j++)
		{
			thinned->push_back(original->points[idxs[j]]);
		}
		std::vector<std::string> id = getFileID(argv[i]);
		ss.str("");
		ss << id[0] << ".tile.thin." << id[1] << ".pcd";
		writer.write(ss.str(),*thinned,true);
		original->clear();
		downsampled->clear();
		thinned->clear();
	}
	return 0;
}
