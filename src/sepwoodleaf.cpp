//Andrew Burt - a.burt@ucl.ac.uk

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

#include <armadillo>

#include "treeseg.h"
#include "leafsep.h"

int main (int argc, char* argv[])
{
	//inputs
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::cout << "Reading plotcloud..." << std::endl;
	reader.read(argv[1],*cloud);
	std::string fname;
	int kn = atoi(argv[2]);
	int n_gaus = atoi(argv[3]);
	int dist_mode = atoi(argv[4]);
	int seed_mode = atoi(argv[5]);
	int km_iter = atoi(argv[6]);
	int em_iter = atoi(argv[7]);
	float smoothness = atof(argv[8]);
	//by cluster
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> regions;
	std::cout << "regionSegmentation..." << std::endl;
	regionSegmentation(cloud,kn,3,smoothness,regions);
	arma::mat featmatrix;
	arma::gmm_full model;
	std::cout << "gmmByCluster..." << std::endl;
	gmmByCluster(regions,n_gaus,dist_mode,seed_mode,km_iter,em_iter,featmatrix,model);
	std::vector<int> classifications;
	std::cout << "classifyGmmClusterModel..." << std::endl;
	classifications = classifyGmmClusterModel(regions,n_gaus,featmatrix,model);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
	std::cout << "separateCloudsClassifiedByCluster..." << std::endl;
	separateCloudsClassifiedByCluster(regions,classifications,clouds);
	std::cout << "writing cc1.pcd and cc2.pcd..." << std::endl;
	writer.write("cc1.pcd",*clouds[0],true);
	writer.write("cc2.pcd",*clouds[1],true);
	//by point
	arma::mat pfeatmat;
	arma::gmm_diag pmodel;
	std::cout << "gmmByPoint..." << std::endl;
	gmmByPoint(clouds[1],50,n_gaus,dist_mode,seed_mode,km_iter,em_iter,pfeatmat,pmodel);
	std::vector<int> pclassifications;
	std::cout << "classifyGmmPointModel..." << std::endl;
	pclassifications = classifyGmmPointModel(clouds[1],n_gaus,pfeatmat,pmodel);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pclouds;
	std::cout << "separateCloudsClassifiedByPoint..." << std::endl;
	separateCloudsClassifiedByPoint(clouds[1],pclassifications,pclouds);
	std::cout << "writing pc1.pcd and pc2.pcd..." << std::endl;
	writer.write("pc1.pcd",*pclouds[0],true);
	writer.write("pc2.pcd",*pclouds[1],true);
	//write clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr wood(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr leaf(new pcl::PointCloud<pcl::PointXYZ>);
	*wood += *clouds[0] + *pclouds[0];
	*leaf += *clouds[1] + *pclouds[1];
	std::cout << "writing wood.pcd and leaf.pcd..." << std::endl;
	writer.write("wood.pcd",*wood,true);
	writer.write("leaf.pcd",*leaf,true);
	return 0;
}
