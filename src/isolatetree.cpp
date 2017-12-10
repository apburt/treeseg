//Andrew Burt - a.burt@ucl.ac.uk

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

#include "treeseg.h"

int main (int argc, char* argv[])
{
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	std::stringstream ss;
	for(int i=2;i<argc;i++)
	{
		std::cout << "----------: " << argv[i] << std::endl;
		std::cout << "Reading volume cloud: " << std::flush;
		std::vector<std::string> id = getFileID(argv[i]);
		pcl::PointCloud<pcl::PointXYZ>::Ptr volume(new pcl::PointCloud<pcl::PointXYZ>);
		reader.read(argv[i],*volume);
		std::cout << "complete" << std::endl;
		//
		std::cout << "Euclidean clustering: " << std::flush;
		std::vector<std::vector<float>> nndata = dNNz(volume,50,2);
		float nnmax = 0;
		for(int i=0;i<nndata.size();i++) if(nndata[i][1] > nnmax) nnmax = nndata[i][1];
		std::cout << nnmax << ", " << std::flush;
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
		euclideanClustering(volume,nnmax,3,clusters);
		ss.str("");
		ss << "clusters_" << id[0] << ".pcd";
		writeClouds(clusters,ss.str(),false);
		std::cout << ss.str() << std::endl;
		//	
		std::cout << "Region-based segmentation: " << std::flush;
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> regions;
		int idx = findPrincipalCloudIdx(clusters);
		int nnearest = 50;
		int nmin = 3;
		float smoothness = atof(argv[1]);
		regionSegmentation(clusters[idx],nnearest,nmin,smoothness,regions);
		ss.str("");
		ss << "clusters_regions_" << id[0] << ".pcd";
		writeClouds(regions,ss.str(),false);
		std::cout << ss.str() << std::endl;
		//
		std::cout << "Building tree: " << std::flush;
		pcl::PointCloud<pcl::PointXYZ>::Ptr tree(new pcl::PointCloud<pcl::PointXYZ>);
		buildTree(regions,tree);
		ss.str("");
		ss << "tree_" << id[0] << ".pcd";
		writer.write(ss.str(),*tree,true);
		std::cout << ss.str() << std::endl;
	}
	return 0;
}
