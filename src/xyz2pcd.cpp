//Andrew Burt - a.burt@ucl.ac.uk

#include "treeseg.h"

#include <pcl/io/pcd_io.h>

int main (int argc, char* argv[])
{
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	for(int i=1;i<argc;i++)
	{
		std::string tmp(argv[i]);
		std::vector<std::string> split;
		boost::split(split,tmp,boost::is_any_of("."));
		std::stringstream ss;
		for(int j=0;j<split.size()-1;j++) ss << split[j] << ".";
		ss << "pcd";
		std::ifstream infile(argv[i]);
		float x,y,z;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		while (infile >> x >> y >> z)
		{
			pcl::PointXYZ point;
			point.x = x;
			point.y = y;
			point.z = z;
			cloud->insert(cloud->end(),point);
		}
		infile.close();
		writer.write(ss.str(),*cloud,true);
	}
	return 0;
}
