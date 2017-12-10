
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "treeseg.h"

int main (int argc, char *argv[])
{
	float edgelength = atof(argv[1]);
	bool catclouds = atoi(argv[2]);
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	pcl::PointCloud<pcl::PointXYZ>::Ptr original(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
	std::stringstream ss;
	for(int i=3;i<argc;i++)
	{
		reader.read(argv[i],*original);
		if(catclouds == true)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
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
