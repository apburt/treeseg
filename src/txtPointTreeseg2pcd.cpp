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
		std::stringstream ss;
		std::vector<std::string> tmp1,tmp2;
		boost::split(tmp1,args[i],boost::is_any_of("/"));
		boost::split(tmp2,tmp1[tmp1.size()-1],boost::is_any_of("."));
		for(int j=0;j<tmp2.size()-1;j++) ss << tmp2[j] << ".";
		ss << "pcd";
		std::ifstream infile(args[i]);
		pcl::PointCloud<PointTreeseg>::Ptr cloud(new pcl::PointCloud<PointTreeseg>);
		#if XYZRRDRS == false
			float x,y,z;
			while(infile >> x >> y >> z)
			{
				PointTreeseg point;
				point.x = x;
				point.y = y;
				point.z = z;
				cloud->insert(cloud->end(),point);	
			}
		#else	
			float x,y,z,range,reflectance,deviation,return_number,scan_number;
			while(infile >> x >> y >> z >> range >> reflectance >> deviation >> return_number >> scan_number)
			{
				PointTreeseg point;
				point.x = x;
				point.y = y;
				point.z = z;
				point.range = range;
				point.reflectance = reflectance;
				point.deviation = std::uint16_t(deviation);
				point.return_number = std::uint16_t(return_number);
				point.scan_number = std::uint16_t(scan_number);
				cloud->insert(cloud->end(),point);	
			}
		#endif
		infile.close();
		writer.write(ss.str(),*cloud,true);
	}
	return 0;
}
