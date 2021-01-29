//Andrew Burt - a.burt@ucl.ac.uk

#include "treeseg.h"

#include <pcl/io/pcd_io.h>

int main(int argc, char **argv)
{
	std::vector<std::string> args(argv+1,argv+argc);
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	std::stringstream ss;
	float smoothness = std::stof(args[0]);
	bool quick = std::stoi(args[1]); 
	for(int i=2;i<args.size();i++)
	{
		std::cout << "----------: " << args[i] << std::endl;
		std::cout << "Reading volume cloud: " << std::flush;
		std::vector<std::string> id = getFileID(args[i]);
		pcl::PointCloud<PointTreeseg>::Ptr volume(new pcl::PointCloud<PointTreeseg>);
		reader.read(args[i],*volume);
		std::cout << "complete" << std::endl;
		//
		std::cout << "Region-based segmentation: " << std::flush; 
		std::vector<pcl::PointCloud<PointTreeseg>::Ptr> regions;
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		estimateNormals(volume,50,normals);
		regionSegmentation(volume,normals,250,3,std::numeric_limits<int>::max(),smoothness,1,regions);
		volume->clear();
		normals->clear();
		ss.str("");
		ss << id[0] << ".rg." << id[1] << ".pcd";
		writeClouds(regions,ss.str(),false);
		std::cout << ss.str() << std::endl;
		//
		std::cout << "Optimising regions: " << std::flush;
		removeFarRegions(0.5,regions);
		ss.str("");
		ss << id[0] << ".rg.o." << id[1] << ".pcd";
		writeClouds(regions,ss.str(),false);
		std::cout << ss.str() << std::endl;
		//
		std::cout << "Euclidean clustering: " << std::flush;		
		for(int i=0;i<regions.size();i++) *volume += *regions[i];
		regions.clear();
		std::vector<pcl::PointCloud<PointTreeseg>::Ptr> clusters;
		euclideanClustering(volume,1.0,1,clusters);
		volume->clear();
		int idx = findPrincipalCloudIdx(clusters);
		*volume = *clusters[idx];
		ss.str("");
		ss << id[0] << ".rg.o.ec." << id[1] << ".pcd";
		writeClouds(clusters,ss.str(),false);
		std::cout << ss.str() << std::endl; 
		//	
		if(quick == true)
		{
			ss.str("");
			ss << id[0] << "_" << id[1] << ".pcd";
			writer.write(ss.str(),*volume,true);
			std::cout << "Tree: " << ss.str() << std::endl;
		}
		else
		{
			std::cout << "Rerunning region-based segmentation: " << std::flush; 
			estimateNormals(volume,50,normals);
			regionSegmentation(volume,normals,250,3,std::numeric_limits<int>::max(),smoothness,1,regions);
			ss.str("");
			ss << id[0] << ".rg.o.ec.rg." << id[1] << ".pcd";
			writeClouds(regions,ss.str(),false);
			std::cout << ss.str() << std::endl;
			//
			std::cout << "Building tree: " << std::flush;
			pcl::PointCloud<PointTreeseg>::Ptr tree(new pcl::PointCloud<PointTreeseg>);
			buildTree(regions,15,1,0.2,5,1.0,tree);
			ss.str("");
			ss << id[0] << "_" << id[1] << ".pcd";
			writer.write(ss.str(),*tree,true);
			std::cout << " " << ss.str() << std::endl;
		}
	}
	return 0;
}
