#include "leafsep.h"

#include <pcl/io/pcd_io.h>

#include <armadillo>

int main (int argc, char **argv)
{
	std::vector<std::string> args(argv+1,argv+argc);
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	int kn = std::stoi(args[0]);
	int n_gaus = std::stoi(args[1]);
	int dist_mode = std::stoi(args[2]);
	int seed_mode = std::stoi(args[3]);
	int km_iter = std::stoi(args[4]);
	int em_iter = std::stoi(args[5]);
	float smoothness = std::stof(args[6]);
	for(int i=7;i<args.size();i++)
	{
		std::cout << "----------: " << args[i] << std::endl;
		std::cout << "Reading cloud: " << std::flush;
		std::vector<std::string> id = getFileID(args[i]);
		pcl::PointCloud<PointTreeseg>::Ptr cloud(new pcl::PointCloud<PointTreeseg>);
		reader.read(args[i],*cloud);
		std::cout << "complete" << std::endl;
		//
		std::cout << "Region-based segmentation: " << std::flush;
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		std::vector<pcl::PointCloud<PointTreeseg>::Ptr> regions;
		estimateNormals(cloud,250,normals);
		regionSegmentation(cloud,normals,30,3,std::numeric_limits<int>::max(),smoothness,1,regions);
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
