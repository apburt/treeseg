//Andrew Burt - a.burt@ucl.ac.uk

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <treeseg.h>

int main (int argc, char *argv[])
{
	float resolution = atof(argv[1]);
	float zmin = atof(argv[2]);
	float zmax = atof(argv[3]);
	pcl::PointCloud<pcl::PointXYZ>::Ptr plotcloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	std::cout << "Reading plotcloud..." << std::endl;
	reader.read(argv[4],*plotcloud);
	std::vector<std::string> id = getFileID(argv[4]);
	std::stringstream ss;
	ss.str("");
	ss << id[1] << ".slice.downsample.pcd";
	std::vector<std::vector<float>> dem;
	pcl::PointCloud<pcl::PointXYZ>::Ptr slice(new pcl::PointCloud<pcl::PointXYZ>);
	std::cout << "Running getDemAndSlice..." << std::endl;
	dem = getDemAndSlice(plotcloud,resolution,zmin,zmax,slice);
	for(int j=0;j<dem.size();j++) std::cout << dem[j][0] << " " << dem[j][1] << " " << dem[j][2] << std::endl;
	std::cout << "Writing slice..." << std::endl;
	writer.write(ss.str(),*slice,true);
	return 0;
}
