//Andrew Burt - a.burt@ucl.ac.uk

#include <treeseg.hpp>

int main (int argc,char** argv)
{
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	for (int i=1;i<argc;i++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		reader.read(argv[i],*cloud);
		writer.write(argv[i],*cloud,false);
	}
	return 0;
}
