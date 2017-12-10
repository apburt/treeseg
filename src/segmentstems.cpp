//Andrew Burt - a.burt@ucl.ac.uk

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

#include "treeseg.h"

int main(int argc, char* argv[])
{
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	std::stringstream ss;
	//
	std::cout << "Reading plot-level cloud: " << std::flush;
	pcl::PointCloud<pcl::PointXYZ>::Ptr plot(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read(argv[2],*plot);
	std::cout << "complete" << std::endl;
	//
	for(int i=3;i<argc;i++)
	{
		std::cout << "----------: " << argv[i] << std::endl;		
		std::vector<std::string> id = getFileID(argv[i]);
		pcl::PointCloud<pcl::PointXYZ>::Ptr foundstem(new pcl::PointCloud<pcl::PointXYZ>);
		reader.read(argv[i],*foundstem);
		//
		std::cout << "RANSAC cylinder fit: " << std::flush;
		int nnearest = 60;
		cylinder cyl;
		fitCylinder(foundstem,nnearest,false,false,cyl);
		std::cout << cyl.rad << std::endl;	
		//
		std::cout << "Segmenting extended cylinder: " << std::flush;
		pcl::PointCloud<pcl::PointXYZ>::Ptr volume(new pcl::PointCloud<pcl::PointXYZ>);
		float expansionfactor = 6;
		cyl.rad = cyl.rad*expansionfactor;
		spatial3DCylinderFilter(plot,cyl,volume);
		ss.str("");
		ss << "cylinder_" << id[0] << ".pcd";
		writer.write(ss.str(),*volume,true);
		std::cout << ss.str() << std::endl;
		//
		std::cout << "Segmenting ground returns: " << std::flush;
		pcl::PointCloud<pcl::PointXYZ>::Ptr bottom(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr top(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr vnoground(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		float zdelta = 0.75;
		Eigen::Vector4f min, max;
		pcl::getMinMax3D(*volume,min,max);
		spatial1DFilter(volume,"z",min[2],min[2]+zdelta,bottom);
		spatial1DFilter(volume,"z",min[2]+zdelta,max[2],top);
		fitPlane(bottom,nnearest,inliers);
		extractIndices(bottom,inliers,true,vnoground);
		*vnoground += *top;
		ss.str("");
		ss << "cylinder_noground_" << id[0] << ".pcd";
		writer.write(ss.str(),*vnoground,true);
		std::cout << ss.str() << std::endl;
		//
		std::cout << "Euclidean clustering: " << std::flush;
		std::vector<std::vector<float>> nndata;
		nndata = dNNz(vnoground,5,2);
		float nnmin = std::numeric_limits<int>().max();
		float nnmax = 0;
		for(int i=0;i<nndata.size();i++)
		{
			if(nndata[i][1] < nnmin) nnmin = nndata[i][1];
			if(nndata[i][1] > nnmax) nnmax = nndata[i][1];
		}
		float dmax = (nnmax+nnmin)/2;
		std::cout << dmax << ", " << std::flush;
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
		euclideanClustering(vnoground,dmax,3,clusters);
		ss.str("");
		ss << "cylinder_noground_clusters_" << id[0] << ".pcd";
		writeClouds(clusters,ss.str(),false);
		std::cout << ss.str() << std::endl;
		//
		std::cout << "Region-based segmentation: " << std::flush; 
		int idx = findClosestIdx(foundstem,clusters);
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> regions;
		nnearest = 50;
		int nmin = 3;
		float smoothness = atof(argv[1]);
		regionSegmentation(clusters[idx],nnearest,nmin,smoothness,regions);
		ss.str("");
		ss << "cylinder_noground_clusters_regions_" << id[0] << ".pcd";
		writeClouds(regions,ss.str(),false);
		std::cout << ss.str() << std::endl;
		//
		std::cout << "Correcting stem: " << std::flush;
		pcl::PointCloud<pcl::PointXYZ>::Ptr stem(new pcl::PointCloud<pcl::PointXYZ>);
		idx = findClosestIdx(foundstem,regions);
		nnearest = 60;
		zdelta = 0.75;
		float zstart = 5;
		float stepcovmax = 0.05;
		float radchangemin = 0.9;
		correctStem(regions[idx],nnearest,zstart,zdelta,stepcovmax,radchangemin,stem);
		ss.str("");
		ss << "stem_" << id[0] << ".pcd";
		writer.write(ss.str(),*stem,true);
		std::cout << ss.str() << std::endl;
	}
	return 0;
}
