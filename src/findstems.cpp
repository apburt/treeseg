//Andrew Burt - a.burt@ucl.ac.uk

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

#include "treeseg.h"

int main (int argc, char *argv[])
{
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	std::stringstream ss;
	//
	std::cout << "Reading slice: " << std::flush;
	pcl::PointCloud<pcl::PointXYZ>::Ptr slice(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read(argv[4],*slice);
	std::cout << "complete" << std::endl;
	//
	std::cout << "Region-based segmentation: " << std::flush;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> regions;
	float nnearest = 9;
	float nmin = 100;
	float smoothness = atof(argv[1]);
	regionSegmentation(slice,nnearest,nmin,smoothness,regions);
	ss << "slice_regions.pcd";
	writeClouds(regions,ss.str(),false);
	std::cout << ss.str() << " | " << regions.size() << std::endl;
	//
	std::cout << "RANSAC cylinder fits: " << std::flush;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cyls;
	nnearest = 60;
	float dmin = atof(argv[2]);
	float dmax = atof(argv[3]);
	float lmin = 2.25; //assuming 3m slice
	float stepcovmax = 0.2;
	float radratiomin = 0.8;
	for(int i=0;i<regions.size();i++)
	{
		cylinder cyl;
		fitCylinder(regions[i],nnearest,true,true,cyl);
		if(cyl.ismodel == true)
		{		
			if(cyl.rad*2 >= dmin && cyl.rad*2 <= dmax && cyl.len >= lmin)
			{
				if(cyl.stepcov <= stepcovmax && cyl.radratio > radratiomin)
				{
					cyls.push_back(cyl.inliers);
				}
			}
		}
	}
	ss.str("");
	ss << "slice_regions_cylinders.pcd";
	writeClouds(cyls,ss.str(),false);
	std::cout << ss.str() << " | " << cyls.size() << std::endl;
	//
	std::cout << "Principal component trimming: " << std::flush;
	float anglemax = 35;
	std::vector<int> idx;
	for(int j=0;j<cyls.size();j++)
	{
		Eigen::Vector4f centroid;
		Eigen::Matrix3f covariancematrix;
		Eigen::Matrix3f eigenvectors;
		Eigen::Vector3f eigenvalues;
		computePCA(cyls[j],centroid,covariancematrix,eigenvectors,eigenvalues);
		Eigen::Vector4f gvector(eigenvectors(0,2),eigenvectors(1,2),0,0);
		Eigen::Vector4f cvector(eigenvectors(0,2),eigenvectors(1,2),eigenvectors(2,2),0);
		float angle = pcl::getAngle3D(gvector,cvector) * (180/M_PI);
		if(angle >= (90 - anglemax) || angle <= (90 + anglemax)) idx.push_back(j);
	}
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pca;
        for(int k=0;k<idx.size();k++) pca.push_back(cyls[idx[k]]);	
	ss.str("");
	ss << "slice_regions_cylinders_principal.pcd";
	writeClouds(pca,ss.str(),false);
	std::cout << ss.str() << " | " << pca.size() << std::endl;
	//
	std::cout << "Concatenating stems: " << std::flush;
	float expansionfactor = 0;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> stems;
	stems = pca;
	catIntersectingClouds(stems);
	ss.str("");
	ss << "slice_regions_cylinder_principal_cat.pcd";
	writeClouds(stems,ss.str(),false);
	for(int m=0;m<stems.size();m++)
	{
		ss.str("");
		ss << "cluster_" << m << ".pcd";
		writer.write(ss.str(),*stems[m],true);
	}
	std::cout << ss.str() << " | " << stems.size() << std::endl;
	//
	return 0;
}
