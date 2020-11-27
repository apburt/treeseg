//Andrew Burt - a.burt@ucl.ac.uk

#include "leafsep.h"

#include <pcl/io/pcd_io.h>

#include <armadillo>

int main(int argc, char **argv)
{
	std::vector<std::string> args(argv+1,argv+argc);
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	std::stringstream ss;
	bool sepwoodleaf = std::stoi(args[1]);
	for(int i=2;i<args.size();i++)
	{
		std::cout << "----------: " << args[i] << std::endl;
		std::cout << "Reading volume cloud: " << std::flush;
		std::vector<std::string> id = getFileID(args[i]);
		pcl::PointCloud<PointTreeseg>::Ptr volume(new pcl::PointCloud<PointTreeseg>);
		reader.read(args[i],*volume);
		std::cout << "complete" << std::endl;
		//
		//std::cout << "Euclidean clustering: " << std::flush;
		//std::vector<pcl::PointCloud<PointTreeseg>::Ptr> clusters;
		//euclideanClustering(volume,2,3,clusters);
		//ss.str("");
		//ss << id[0] << ".ec." << id[1] << ".pcd";
		//writeClouds(clusters,ss.str(),false);
		//std::cout << ss.str() << std::endl;
		//int idx = findPrincipalCloudIdx(clusters);
		//
		std::cout << "Region-based segmentation: " << std::flush; 
		std::vector<pcl::PointCloud<PointTreeseg>::Ptr> regions;
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		float smoothness = std::stof(args[0]);
		estimateNormals(volume,50,normals);
		regionSegmentation(volume,normals,250,3,std::numeric_limits<int>::max(),smoothness,2,regions);
		ss.str("");
		ss << id[0] << ".ec.rg." << id[1] << ".pcd";
		writeClouds(regions,ss.str(),false);
		std::cout << ss.str() << std::endl;
		//
		if(sepwoodleaf == true)
		{
			std::cout << "Leaf stripping: " << std::endl;
			//
			std::cout << " Region-wise, " << std::flush;
		        arma::mat rfmat;
		        arma::gmm_full rmodel;
		        gmmByCluster(regions,5,1,5,50,100,rfmat,rmodel);
		        std::vector<int> rclassifications;
		        rclassifications = classifyGmmClusterModel(regions,5,rfmat,rmodel);
		        std::vector<pcl::PointCloud<PointTreeseg>::Ptr> csepclouds;
		        separateCloudsClassifiedByCluster(regions,rclassifications,csepclouds);
			ss.str("");
			ss << id[0] << ".ec.rg.rlw." << id[1] << ".pcd";
			writeCloudClassifiedByCluster(regions,rclassifications,ss.str());
			std::cout << ss.str() << std::endl;
			//
			std::cout << " Point-wise, " << std::flush;
			arma::mat pfmat;
			arma::gmm_diag pmodel;
			gmmByPoint(csepclouds[1],50,5,1,5,50,100,pfmat,pmodel);
			std::vector<int> pclassifications;
			pclassifications = classifyGmmPointModel(csepclouds[1],5,pfmat,pmodel);
			std::vector<pcl::PointCloud<PointTreeseg>::Ptr> psepclouds;
			separateCloudsClassifiedByPoint(csepclouds[1],pclassifications,psepclouds);
			ss.str("");
			ss << id[0] << ".ec.rg.rlw.plw." << id[1] << ".pcd";
			writeCloudClassifiedByPoint(csepclouds[1],pclassifications,ss.str());
			std::cout << ss.str() << std::endl;
			//
			ss.str("");
			ss << id[0] << ".ec.rg.rlw.plw.w." << id[1] << ".pcd";
			pcl::PointCloud<PointTreeseg>::Ptr wood(new pcl::PointCloud<PointTreeseg>);
			*wood += *csepclouds[0] + *psepclouds[0];
			writer.write(ss.str(),*wood,true);
			std::cout << ss.str() << std::endl;
			//
			std::cout << "Re-segmenting regions: " << std::flush;
			normals->clear();
			regions.clear();
			estimateNormals(wood,50,normals);
			regionSegmentation(wood,normals,30,3,std::numeric_limits<int>::max(),smoothness,1,regions);
			ss.str("");
			ss << id[0] << ".ec.rg.rlw.plw.w.rg." << id[1] << ".pcd";
			writeClouds(regions,ss.str(),false);
			std::cout << ss.str() << std::endl;
		}
		//
		//std::cout << "Optimising regions: " << std::flush;
		//removeFarRegions(regions);
		//ss.str("");
		//if(sepwoodleaf == true)	ss << id[0] << ".ec.rg.rlw.plw.w.rg.o." << id[1] << ".pcd";
		//else ss << id[0] << ".ec.rg.o." << id[1] << ".pcd";
		//writeClouds(regions,ss.str(),false);
		//std::cout << ss.str() << std::endl;
		//
		std::cout << "Building tree: " << std::flush;
		pcl::PointCloud<PointTreeseg>::Ptr tree(new pcl::PointCloud<PointTreeseg>);
		buildTree(regions,tree);
		ss.str("");
		ss << id[0] << "_" << id[1] << ".pcd";
		writer.write(ss.str(),*tree,true);
		std::cout << ss.str() << std::endl;
	}
	return 0;
}
