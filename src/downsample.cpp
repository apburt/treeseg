//Andrew Burt - a.burt@ucl.ac.uk

#include "treeseg.h"

#include <pcl/io/pcd_io.h>

int main(int argc, char **argv)
{
	std::vector<std::string> args(argv+1,argv+argc);
	float edgelength = std::stof(args[0]);
	bool thin = std::stoi(args[1]);
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	pcl::PointCloud<PointTreeseg>::Ptr original(new pcl::PointCloud<PointTreeseg>);
	pcl::PointCloud<PointTreeseg>::Ptr downsampled(new pcl::PointCloud<PointTreeseg>);
	pcl::PointCloud<PointTreeseg>::Ptr thinned(new pcl::PointCloud<PointTreeseg>);
	std::stringstream ss;
	for(int i=2;i<args.size();i++)
	{
		ss.str("");
		std::vector<std::string> id = getFileID(args[i]);
		reader.read(args[i],*original);
		downsample(original,edgelength,downsampled);
		if(thin == true)
		{
			std::vector<int> idxs = nearestIdx(downsampled,original);
			for(int j=0;j<idxs.size();j++)
			{
				thinned->push_back(original->points[idxs[j]]);
			}
			ss << id[0] << ".tile.thin."  << id[1] << ".pcd"; 
			writer.write(ss.str(),*thinned,true);
		}
		else
		{
			ss << id[0] << ".tile.downsample." << id[1] << ".pcd";
			writer.write(ss.str(),*downsampled,true);
		}
		original->clear();	
		downsampled->clear();
		thinned->clear();
	}
	return 0;
}
