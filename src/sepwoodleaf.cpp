//Andrew Burt - a.burt@ucl.ac.uk

#include "leafsep.h"

#include <pcl/io/pcd_io.h>

#include <armadillo>

int main (int argc, char* argv[])
{
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	int kn = atoi(argv[1]);
	int n_gaus = atoi(argv[2]);
	int dist_mode = atoi(argv[3]);
	int seed_mode = atoi(argv[4]);
	int km_iter = atoi(argv[5]);
	int em_iter = atoi(argv[6]);
	float smoothness = atof(argv[7]);
	for(int i=8;i<argc;i++)
	{
		std::cout << "----------: " << argv[i] << std::endl;
		std::cout << "Reading cloud: " << std::flush;
		std::vector<std::string> id = getFileID(argv[i]);
		pcl::PointCloud<PointTreeseg>::Ptr cloud(new pcl::PointCloud<PointTreeseg>);
		reader.read(argv[i],*cloud);
		std::cout << "complete" << std::endl;
		//
		std::cout << "Region-based segmentation: " << std::flush;
		std::vector<pcl::PointCloud<PointTreeseg>::Ptr> regions;
		regionSegmentation(cloud,kn,3,smoothness,regions);
		std::stringstream ss;
		ss << "regions_" << id[0] << ".pcd";
		writeClouds(regions,ss.str(),false);
		std::cout << ss.str() << std::endl;
		//
		std::cout << "Classifying cloud by region: " << std::flush;
		arma::mat featmatrix;
		arma::gmm_full model;
		gmmByCluster(regions,n_gaus,dist_mode,seed_mode,km_iter,em_iter,featmatrix,model);
		std::vector<int> classifications;
		classifications = classifyGmmClusterModel(regions,n_gaus,featmatrix,model);
		std::vector<pcl::PointCloud<PointTreeseg>::Ptr> clouds;
		separateCloudsClassifiedByCluster(regions,classifications,clouds);
		ss.str("");
		ss << "regions_rclass_" << id[0] << ".pcd";
		writeCloudClassifiedByCluster(regions,classifications,ss.str());
		std::cout << ss.str() << std::endl;
		//
		std::cout << "Classifying remaining cloud by point: " << std::flush;
		arma::mat pfeatmat;
		arma::gmm_diag pmodel;
		gmmByPoint(clouds[1],kn,n_gaus,dist_mode,seed_mode,km_iter,em_iter,pfeatmat,pmodel);
		std::vector<int> pclassifications;
		pclassifications = classifyGmmPointModel(clouds[1],n_gaus,pfeatmat,pmodel);
		std::vector<pcl::PointCloud<PointTreeseg>::Ptr> pclouds;
		separateCloudsClassifiedByPoint(clouds[1],pclassifications,pclouds);
		ss.str("");
		ss << "regions_rclass_pclass_" << id[0] << ".pcd";
		writeCloudClassifiedByPoint(clouds[1],pclassifications,ss.str());
		std::cout << ss.str() << std::endl;
		//
		std::cout << "Writing wood/leaf clouds: " << std::flush;
		pcl::PointCloud<PointTreeseg>::Ptr wood(new pcl::PointCloud<PointTreeseg>);
		*wood += *clouds[0] + *pclouds[0];
		ss.str("");
		ss << id[1] << "_" << id[0] << "w" << ".pcd";
		writer.write(ss.str(),*wood,true);
		std::cout << ss.str() << "," << std::flush;
		pcl::PointCloud<PointTreeseg>::Ptr leaf(new pcl::PointCloud<PointTreeseg>);
		*leaf = *pclouds[1];
		ss.str("");
		ss << id[1] << "_" << id[0] << "l" << ".pcd";
		writer.write(ss.str(),*leaf,true);
		std::cout << ss.str() << std::endl;
	}
	return 0;
}
