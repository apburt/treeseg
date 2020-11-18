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
		std::string tmp = args[i];
		std::vector<std::string> split;
		boost::split(split,tmp,boost::is_any_of("."));
		std::stringstream ss;
		for(int j=0;j<split.size()-1;j++) ss << split[j] << ".";
		ss << "txt";
		std::ofstream outfile(ss.str());
		pcl::PointCloud<PointTreeseg>::Ptr cloud(new pcl::PointCloud<PointTreeseg>);
		reader.read(argv[i],*cloud);
		for(int i=0;i<cloud->points.size();i++)
		{
			outfile << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << "\n";
		}
		outfile.close();
	}
	return 0;
}
