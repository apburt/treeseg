//Andrew Burt - a.burt@ucl.ac.uk

#include "treeseg.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

int main (int argc, char *argv[])
{
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	std::stringstream ss;
	//
	std::cout << "Reading slice: " << std::flush;
	pcl::PointCloud<PointTreeseg>::Ptr slice(new pcl::PointCloud<PointTreeseg>);
	reader.read(argv[5],*slice);
	std::cout << "complete" << std::endl;
	//
	std::cout << "Cluster extraction: " << std::flush;
	std::vector<pcl::PointCloud<PointTreeseg>::Ptr> clusters;
	int nnearest = 18;
	int nmin = 100;
	std::vector<float> nndata = dNN(slice,nnearest);
	euclideanClustering(slice,nndata[0],nmin,clusters);
	ss.str("");
	ss << "slice_clusters.pcd";
	writeClouds(clusters,ss.str(),false);
	std::cout << ss.str() << " | " << clusters.size() << std::endl;
	//
	std::cout << "Region-based segmentation: " << std::flush;
	std::vector<pcl::PointCloud<PointTreeseg>::Ptr> regions;
	nnearest = 9;
	nmin = 100;
	float smoothness = atof(argv[1]);
	for(int i=0;i<clusters.size();i++)
	{
		std::vector<pcl::PointCloud<PointTreeseg>::Ptr> tmpregions;
		regionSegmentation(clusters[i],nnearest,nmin,smoothness,tmpregions);
		for(int j=0;j<tmpregions.size();j++) regions.push_back(tmpregions[j]);
	}
	ss.str("");
	ss << "slice_clusters_regions.pcd";
	writeClouds(regions,ss.str(),false);
	std::cout << ss.str() << " | " << regions.size() << std::endl;
	//
	std::cout << "RANSAC cylinder fits: " << std::flush;
	std::vector<pcl::PointCloud<PointTreeseg>::Ptr> cyls;
	nnearest = 60;
	float dmin = atof(argv[2]);
	float dmax = atof(argv[3]);
	std::ifstream coordfile;
	coordfile.open(argv[4]);
	float coords[4];
	int n = 0;
	if(coordfile.is_open())
	{
		while(!coordfile.eof())
		{
			coordfile >> coords[n];
			n++;
		}
	}
	coordfile.close();
	float xmin = coords[0];
	float xmax = coords[1];
	float ymin = coords[2];
	float ymax = coords[3];
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
					if(cyl.x >= xmin && cyl.x <= xmax)
					{
						if(cyl.y >= ymin && cyl.y <= ymax)
						{
							cyls.push_back(cyl.inliers);
						}
					}
				}
			}
		}
	}
	ss.str("");
	ss << "slice_clusters_regions_cylinders.pcd";
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
	std::vector<pcl::PointCloud<PointTreeseg>::Ptr> pca;
        for(int k=0;k<idx.size();k++) pca.push_back(cyls[idx[k]]);	
	ss.str("");
	ss << "slice_clusters_regions_cylinders_principal.pcd";
	writeClouds(pca,ss.str(),false);
	std::cout << ss.str() << " | " << pca.size() << std::endl;
	//
	std::cout << "Concatenating stems: " << std::flush;
	float expansionfactor = 0;
	std::vector<pcl::PointCloud<PointTreeseg>::Ptr> stems;
	stems = pca;
	catIntersectingClouds(stems);
	ss.str("");
	ss << "slice_clusters_regions_cylinders_principal_cat.pcd";
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
