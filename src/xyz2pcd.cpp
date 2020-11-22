//Andrew Burt - a.burt@ucl.ac.uk

#include "treeseg.h"

#include <pcl/io/pcd_io.h>

int main (int argc, char **argv)
{
	std::vector<std::string> args(argv+1,argv+argc);
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	for(int i=0;i<args.size();i++)
	{
		//std::cout << args[i] << std::endl;
		std::string tmp = args[i];
		std::vector<std::string> split;
		boost::split(split,tmp,boost::is_any_of("."));
		std::stringstream ss;
		for(int j=0;j<split.size()-1;j++) ss << split[j] << ".";
		ss << "pcd";
		std::ifstream infile(args[i]);
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
