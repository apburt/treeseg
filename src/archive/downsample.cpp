//Andrew Burt - a.burt@ucl.ac.uk

#include <treeseg.hpp>

int main (int argc,char** argv)
{
	float tolerance = atof(argv[1]);
	for(int i=2;i<argc;i++)
	{
		std::string fname;
		std::vector<std::string> name1;
		std::vector<std::string> name2;
		std::vector<std::string> name3;
		boost::split(name1,argv[i],boost::is_any_of("."));
		boost::split(name2,name1[name1.size()-2],boost::is_any_of("/"));
		//boost::split(name3,name2[name2.size()-1],boost::is_any_of("_"));
		fname = name2[name2.size()-1];
		std::stringstream ss;
		ss << fname << ".downsample.pcd";
		pcl::PCDReader reader;
		pcl::PCDWriter writer;
		pcl::VoxelGrid<pcl::PointXYZ> downsample;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
		reader.read(argv[i],*cloud_in);
		downsample.setInputCloud(cloud_in);
		downsample.setLeafSize(tolerance,tolerance,tolerance);
		downsample.filter(*cloud_down);
		writer.write(ss.str(),*cloud_down,true);
	}
	return 0;
}
