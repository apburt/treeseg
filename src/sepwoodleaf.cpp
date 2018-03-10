//Andrew Burt - a.burt@ucl.ac.uk

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

#include <Armadillo>

#include "treeseg.h"
#include "leafsep.h"

int main (int argc, char* argv[])
{
	//inputs
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
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
	regionSegmentation(cloud,kn,3,smoothness,regions);
	arma::mat featmatrix;
	arma::gmm_full model;
	gmmByCluster(regions,n_gaus,dist_mode,seed_mode,km_iter,em_iter,featmatrix,model);
	std::vector<int> classifications;
	classifications = classifyGmmClusterModel(regions,n_gaus,featmatrix,model);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
	separateCloudsClassifiedByCluster(regions,classifications,clouds);
	writer.write("cc1.pcd",*clouds[0],true);
	writer.write("cc2.pcd",*clouds[1],true);
	//by point
	arma::mat pfeatmat;
	arma::gmm_diag pmodel;
	gmmByPoint(clouds[1],50,n_gaus,dist_mode,seed_mode,km_iter,em_iter,pfeatmat,pmodel);
	std::vector<int> pclassifications;
	pclassifications = classifyGmmPointModel(clouds[1],n_gaus,pfeatmat,pmodel);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pclouds;
	separateCloudsClassifiedByPoint(clouds[1],pclassifications,pclouds);
	writer.write("pc1.pcd",*pclouds[0],true);
	writer.write("pc2.pcd",*pclouds[1],true);
	//write clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr wood(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr leaf(new pcl::PointCloud<pcl::PointXYZ>);
	*wood += *clouds[0] + *pclouds[0];
	*leaf += *clouds[1] + *pclouds[1];
	writer.write("wood.pcd",*wood,true);
	writer.write("leaf.pcd",*leaf,true);
	return 0;
}
