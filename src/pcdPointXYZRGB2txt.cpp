#include <pcl/io/pcd_io.h>

int main (int argc, char **argv)
{
	std::vector<std::string> args(argv+1,argv+argc);
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	for(int i=0;i<args.size();i++)
	{
		std::stringstream ss;
		std::vector<std::string> tmp1,tmp2;
		boost::split(tmp1,args[i],boost::is_any_of("/"));
		boost::split(tmp2,tmp1[tmp1.size()-1],boost::is_any_of("."));
		for(int j=0;j<tmp2.size()-1;j++) ss << tmp2[j] << ".";
		ss << "txt";
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		reader.read(args[i],*cloud);
		std::ofstream outfile(ss.str());
		for(int i=0;i<cloud->points.size();i++)
		{
			outfile << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << " " << (int)cloud->points[i].r << " " << (int)cloud->points[i].g << " " << (int)cloud->points[i].b << "\n";
		}
		outfile.close();
	}
	return 0;
}
