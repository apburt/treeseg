//Andrew Burt - a.burt@ucl.ac.uk

#include "treeseg.h"

#include <pcl/io/pcd_io.h>

int main (int argc, char *argv[])
{
	float edgelength = atof(argv[1]);
	bool catclouds = atoi(argv[2]);
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	pcl::PointCloud<PointTreeseg>::Ptr original(new pcl::PointCloud<PointTreeseg>);
	pcl::PointCloud<PointTreeseg>::Ptr filtered(new pcl::PointCloud<PointTreeseg>);
	std::stringstream ss;
	for(int i=3;i<argc;i++)
	{
		reader.read(argv[i],*original);
		if(catclouds == true)
		{
			pcl::PointCloud<PointTreeseg>::Ptr tmp(new pcl::PointCloud<PointTreeseg>);
			downsample(original,edgelength,tmp);
			*filtered += *tmp;
			original->clear();
		}
		if(catclouds == false)
		{
			std::vector<std::string> id = getFileID(argv[i]);
			downsample(original,edgelength,filtered);
			ss.str("");
			ss << id[1] << "_" << id[0] << ".downsample.pcd"; 
			writer.write(ss.str(),*filtered,true);
			original->clear();	
			filtered->clear();
		}

	}
	if(catclouds == true)
	{
		std::vector<std::string> id = getFileID(argv[3]);
		ss.str("");
		ss << id[1] << ".downsample.pcd";
		writer.write(ss.str(),*filtered,true);
	}
	return 0;
}
