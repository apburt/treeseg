//Andrew Burt - a.burt@ucl.ac.uk

#include "treeseg.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

int main(int argc, char **argv)
{
	std::vector<std::string> args(argv+1,argv+argc);
	//
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	std::stringstream ss;
	//
	pcl::PointCloud<PointTreeseg>::Ptr cloud(new pcl::PointCloud<PointTreeseg>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloudnormals(new pcl::PointCloud<pcl::PointNormal>);
	std::vector<pcl::PointCloud<PointTreeseg>::Ptr> regions;
	reader.read(args[0],*cloud);

	//Normals 50
	normals->clear();
	cloudnormals->clear();
	regions.clear();
	estimateNormals(cloud,50,normals);
	pcl::concatenateFields(*cloud,*normals,*cloudnormals);
	writer.write("normal_50.pcd",*cloudnormals,true);
	//
	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),5,1,regions);
	writeClouds(regions,"50_10_3_5_1.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),10,1,regions);
	writeClouds(regions,"50_10_3_10_1.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),15,1,regions);
	writeClouds(regions,"50_10_3_15_1.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),5,1,regions);
	writeClouds(regions,"50_50_3_5_1.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),10,1,regions);
	writeClouds(regions,"50_50_3_10_1.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),15,1,regions);
	writeClouds(regions,"50_50_3_15_1.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),5,1,regions);
	writeClouds(regions,"50_100_3_5_1.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),10,1,regions);
	writeClouds(regions,"50_100_3_10_1.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),15,1,regions);
	writeClouds(regions,"50_100_3_15_1.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),5,1,regions);
	writeClouds(regions,"50_250_3_5_1.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),10,1,regions);
	writeClouds(regions,"50_250_3_10_1.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),15,1,regions);
	writeClouds(regions,"50_250_3_15_1.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),5,2,regions);
	writeClouds(regions,"50_10_3_5_2.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),10,2,regions);
	writeClouds(regions,"50_10_3_10_2.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),15,2,regions);
	writeClouds(regions,"50_10_3_15_2.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),5,2,regions);
	writeClouds(regions,"50_50_3_5_2.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),10,2,regions);
	writeClouds(regions,"50_50_3_10_2.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),15,2,regions);
	writeClouds(regions,"50_50_3_15_2.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),5,2,regions);
	writeClouds(regions,"50_100_3_5_2.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),10,2,regions);
	writeClouds(regions,"50_100_3_10_2.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),15,2,regions);
	writeClouds(regions,"50_100_3_15_2.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),5,2,regions);
	writeClouds(regions,"50_250_3_5_2.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),10,2,regions);
	writeClouds(regions,"50_250_3_10_2.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),15,2,regions);
	writeClouds(regions,"50_250_3_15_2.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),5,3,regions);
	writeClouds(regions,"50_10_3_5_3.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),10,3,regions);
	writeClouds(regions,"50_10_3_10_3.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),15,3,regions);
	writeClouds(regions,"50_10_3_15_3.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),5,3,regions);
	writeClouds(regions,"50_50_3_5_3.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),10,3,regions);
	writeClouds(regions,"50_50_3_10_3.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),15,3,regions);
	writeClouds(regions,"50_50_3_15_3.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),5,3,regions);
	writeClouds(regions,"50_100_3_5_3.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),10,3,regions);
	writeClouds(regions,"50_100_3_10_3.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),15,3,regions);
	writeClouds(regions,"50_100_3_15_3.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),5,3,regions);
	writeClouds(regions,"50_250_3_5_3.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),10,3,regions);
	writeClouds(regions,"50_250_3_10_3.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),15,3,regions);
	writeClouds(regions,"50_250_3_15_3.pcd",false);
	regions.clear();


	//Normals 100
	normals->clear();
	cloudnormals->clear();
	regions.clear();
	estimateNormals(cloud,100,normals);
	pcl::concatenateFields(*cloud,*normals,*cloudnormals);
	writer.write("normal_100.pcd",*cloudnormals,true);
	//
	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),5,1,regions);
	writeClouds(regions,"100_10_3_5_1.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),10,1,regions);
	writeClouds(regions,"100_10_3_10_1.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),15,1,regions);
	writeClouds(regions,"100_10_3_15_1.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),5,1,regions);
	writeClouds(regions,"100_50_3_5_1.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),10,1,regions);
	writeClouds(regions,"100_50_3_10_1.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),15,1,regions);
	writeClouds(regions,"100_50_3_15_1.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),5,1,regions);
	writeClouds(regions,"100_100_3_5_1.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),10,1,regions);
	writeClouds(regions,"100_100_3_10_1.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),15,1,regions);
	writeClouds(regions,"100_100_3_15_1.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),5,1,regions);
	writeClouds(regions,"100_250_3_5_1.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),10,1,regions);
	writeClouds(regions,"100_250_3_10_1.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),15,1,regions);
	writeClouds(regions,"100_250_3_15_1.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),5,2,regions);
	writeClouds(regions,"100_10_3_5_2.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),10,2,regions);
	writeClouds(regions,"100_10_3_10_2.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),15,2,regions);
	writeClouds(regions,"100_10_3_15_2.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),5,2,regions);
	writeClouds(regions,"100_50_3_5_2.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),10,2,regions);
	writeClouds(regions,"100_50_3_10_2.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),15,2,regions);
	writeClouds(regions,"100_50_3_15_2.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),5,2,regions);
	writeClouds(regions,"100_100_3_5_2.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),10,2,regions);
	writeClouds(regions,"100_100_3_10_2.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),15,2,regions);
	writeClouds(regions,"100_100_3_15_2.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),5,2,regions);
	writeClouds(regions,"100_250_3_5_2.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),10,2,regions);
	writeClouds(regions,"100_250_3_10_2.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),15,2,regions);
	writeClouds(regions,"100_250_3_15_2.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),5,3,regions);
	writeClouds(regions,"100_10_3_5_3.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),10,3,regions);
	writeClouds(regions,"100_10_3_10_3.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),15,3,regions);
	writeClouds(regions,"100_10_3_15_3.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),5,3,regions);
	writeClouds(regions,"100_50_3_5_3.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),10,3,regions);
	writeClouds(regions,"100_50_3_10_3.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),15,3,regions);
	writeClouds(regions,"100_50_3_15_3.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),5,3,regions);
	writeClouds(regions,"100_100_3_5_3.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),10,3,regions);
	writeClouds(regions,"100_100_3_10_3.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),15,3,regions);
	writeClouds(regions,"100_100_3_15_3.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),5,3,regions);
	writeClouds(regions,"100_250_3_5_3.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),10,3,regions);
	writeClouds(regions,"100_250_3_10_3.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),15,3,regions);
	writeClouds(regions,"100_250_3_15_3.pcd",false);
	regions.clear();





	//Normals 250
	normals->clear();
	cloudnormals->clear();
	regions.clear();
	estimateNormals(cloud,250,normals);
	pcl::concatenateFields(*cloud,*normals,*cloudnormals);
	writer.write("normal_250.pcd",*cloudnormals,true);
	//
	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),5,1,regions);
	writeClouds(regions,"250_10_3_5_1.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),10,1,regions);
	writeClouds(regions,"250_10_3_10_1.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),15,1,regions);
	writeClouds(regions,"250_10_3_15_1.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),5,1,regions);
	writeClouds(regions,"250_50_3_5_1.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),10,1,regions);
	writeClouds(regions,"250_50_3_10_1.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),15,1,regions);
	writeClouds(regions,"250_50_3_15_1.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),5,1,regions);
	writeClouds(regions,"250_100_3_5_1.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),10,1,regions);
	writeClouds(regions,"250_100_3_10_1.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),15,1,regions);
	writeClouds(regions,"250_100_3_15_1.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),5,1,regions);
	writeClouds(regions,"250_250_3_5_1.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),10,1,regions);
	writeClouds(regions,"250_250_3_10_1.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),15,1,regions);
	writeClouds(regions,"250_250_3_15_1.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),5,2,regions);
	writeClouds(regions,"250_10_3_5_2.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),10,2,regions);
	writeClouds(regions,"250_10_3_10_2.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),15,2,regions);
	writeClouds(regions,"250_10_3_15_2.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),5,2,regions);
	writeClouds(regions,"250_50_3_5_2.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),10,2,regions);
	writeClouds(regions,"250_50_3_10_2.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),15,2,regions);
	writeClouds(regions,"250_50_3_15_2.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),5,2,regions);
	writeClouds(regions,"250_100_3_5_2.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),10,2,regions);
	writeClouds(regions,"250_100_3_10_2.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),15,2,regions);
	writeClouds(regions,"250_100_3_15_2.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),5,2,regions);
	writeClouds(regions,"250_250_3_5_2.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),10,2,regions);
	writeClouds(regions,"250_250_3_10_2.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),15,2,regions);
	writeClouds(regions,"250_250_3_15_2.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),5,3,regions);
	writeClouds(regions,"250_10_3_5_3.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),10,3,regions);
	writeClouds(regions,"250_10_3_10_3.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,10,3,std::numeric_limits<int>::max(),15,3,regions);
	writeClouds(regions,"250_10_3_15_3.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),5,3,regions);
	writeClouds(regions,"250_50_3_5_3.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),10,3,regions);
	writeClouds(regions,"250_50_3_10_3.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,50,3,std::numeric_limits<int>::max(),15,3,regions);
	writeClouds(regions,"250_50_3_15_3.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),5,3,regions);
	writeClouds(regions,"250_100_3_5_3.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),10,3,regions);
	writeClouds(regions,"250_100_3_10_3.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,100,3,std::numeric_limits<int>::max(),15,3,regions);
	writeClouds(regions,"250_100_3_15_3.pcd",false);
	regions.clear();

	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),5,3,regions);
	writeClouds(regions,"250_250_3_5_3.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),10,3,regions);
	writeClouds(regions,"250_250_3_10_3.pcd",false);
	regions.clear();
	regionSegmentation(cloud,normals,250,3,std::numeric_limits<int>::max(),15,3,regions);
	writeClouds(regions,"250_250_3_15_3.pcd",false);
	regions.clear();

	return 0;
}
