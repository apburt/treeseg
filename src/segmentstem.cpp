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
	std::cout << "Reading plot-level cloud: " << std::flush;
	pcl::PointCloud<PointTreeseg>::Ptr plot(new pcl::PointCloud<PointTreeseg>);
	readTiles(args,plot);
	std::cout << "complete" << std::endl;
	//
	for(int i=1;i<getTilesStartIdx(args);i++)
	{
		std::cout << "----------: " << args[i] << std::endl;		
		std::vector<std::string> id = getFileID(args[i]);
		pcl::PointCloud<PointTreeseg>::Ptr foundstem(new pcl::PointCloud<PointTreeseg>);
		reader.read(args[i],*foundstem);
		//
		std::cout << "RANSAC cylinder fit: " << std::flush;
		int nnearest = 60;
		cylinder cyl;
		fitCylinder(foundstem,nnearest,false,false,cyl);
		if(cyl.rad < 0.2) cyl.rad = 0.2;
		std::cout << cyl.rad << std::endl;	
		//
		std::cout << "Segmenting extended cylinder: " << std::flush;
		pcl::PointCloud<PointTreeseg>::Ptr volume(new pcl::PointCloud<PointTreeseg>);
		float expansionfactor = 12;
		cyl.rad = cyl.rad*expansionfactor;
		spatial3DCylinderFilter(plot,cyl,volume);
		ss.str("");
		ss << id[0] << ".intermediate.cylinder." << id[1] << ".pcd";
		writer.write(ss.str(),*volume,true);
		std::cout << ss.str() << std::endl;
		//
		std::cout << "Segmenting ground returns: " << std::flush;
		pcl::PointCloud<PointTreeseg>::Ptr bottom(new pcl::PointCloud<PointTreeseg>);
		pcl::PointCloud<PointTreeseg>::Ptr top(new pcl::PointCloud<PointTreeseg>);
		pcl::PointCloud<PointTreeseg>::Ptr vnoground(new pcl::PointCloud<PointTreeseg>);
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		std::sort(volume->points.begin(),volume->points.end(),sortCloudByZ);
		int idx = (2.5/100) * volume->points.size();
		float ground = volume->points[idx].z;
		Eigen::Vector4f min, max;
		pcl::getMinMax3D(*volume,min,max);
		spatial1DFilter(volume,"z",ground-1,ground+3,bottom);
		spatial1DFilter(volume,"z",ground+3,max[2],top);
		estimateNormals(bottom,250,normals); //tested both 100 and 250
		fitPlane(bottom,normals,0.5,inliers,0.75,30);
		extractIndices(bottom,inliers,true,vnoground);
		*vnoground += *top;
		ss.str("");
		ss << id[0] << ".intermediate.cylinder.noground." << id[1] << ".pcd";
		writer.write(ss.str(),*vnoground,true);
		std::cout << ss.str() << std::endl;
		//
		std::cout << "Euclidean clustering: " << std::flush;
		std::vector<std::vector<float>> nndata;
		nndata = dNNz(vnoground,9,2);
		float nnmin = std::numeric_limits<int>().max();
		float nnmax = 0;
		for(int i=0;i<nndata.size();i++)
		{
			if(nndata[i][1] < nnmin) nnmin = nndata[i][1];
			if(nndata[i][1] > nnmax) nnmax = nndata[i][1];
		}
		float dmax = (nnmax+nnmin)/2;
		std::cout << dmax << ", " << std::flush;
		std::vector<pcl::PointCloud<PointTreeseg>::Ptr> clusters;
		euclideanClustering(vnoground,dmax,3,clusters);
		ss.str("");
		ss << id[0] << ".intermediate.cylinder.noground.clusters." << id[1] << ".pcd";
		writeClouds(clusters,ss.str(),false);
		std::cout << ss.str() << std::endl;
		//
		std::cout << "Region-based segmentation: " << std::flush; 
		idx = findClosestIdx(foundstem,clusters,true);
		std::vector<pcl::PointCloud<PointTreeseg>::Ptr> regions;
		nnearest = 50;
		int nmin = 3;
		float smoothness = std::stof(args[0]);
		regionSegmentation(clusters[idx],nnearest,nmin,smoothness,regions);
		ss.str("");
		ss << id[0] << ".intermediate.cylinder.noground.clusters.regions." << id[1] << ".pcd";
		writeClouds(regions,ss.str(),false);
		std::cout << ss.str() << std::endl;
		//
		std::cout << "Correcting stem: " << std::flush;
		pcl::PointCloud<PointTreeseg>::Ptr stem(new pcl::PointCloud<PointTreeseg>);
		idx = findClosestIdx(foundstem,regions,true);
		nnearest = 60;
		float zdelta = 0.75;
		float zstart = 5;
		float stepcovmax = 0.05;
		float radchangemin = 0.9;
		correctStem(regions[idx],nnearest,zstart,zdelta,stepcovmax,radchangemin,stem);
		ss.str("");
		ss << id[0] << ".stem." << id[1] << ".pcd";
		writer.write(ss.str(),*stem,true);
		std::cout << ss.str() << std::endl;
	}
	return 0;
}
