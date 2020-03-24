/*
* leafsep.cpp
*
* MIT License
*
* Copyright 2020 Andrew Burt and Matheus Boni Vicari
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

// leafsep has been developed using:
// TLSeparation (https://github.com/TLSeparation)
// Point Cloud Library (http://www.pointclouds.org)
// Armadillo (http://arma.sourceforge.net)

#include "leafsep.h"

#include <cmath>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

void gmmByPoint(pcl::PointCloud<PointTreeseg>::Ptr &cloud, int knn, arma::uword n_gaus, int dist_mode, int seed_mode, int km_iter, int em_iter, arma::mat &featmatrix, arma::gmm_diag &model)
{
	pcl::PointCloud<PointTreeseg>::Ptr kcloud(new pcl::PointCloud<PointTreeseg>);
	Eigen::Vector4f centroid;
	Eigen::Matrix3f covariancematrix,eigenvectors;
	Eigen::Vector3f eigenvalues;
	float esum,e1,e2,e3;
	float features[6];
	featmatrix.resize(6,cloud->points.size());
	featmatrix.zeros();
	pcl::KdTreeFLANN<PointTreeseg> tree;
	tree.setInputCloud(cloud);
	int k = knn+1;
	std::vector<int> pointIdxNKNSearch(k);
	std::vector<float> pointNKNSquaredDistance(k);
	for(int i=0;i<cloud->points.size();i++)
	{
		PointTreeseg searchPoint = cloud->points[i];
		tree.nearestKSearch(searchPoint,k,pointIdxNKNSearch,pointNKNSquaredDistance);
		for(int j=0;j<pointIdxNKNSearch.size();j++) kcloud->insert(kcloud->end(),cloud->points[pointIdxNKNSearch[j]]);
		computePCA(kcloud,centroid,covariancematrix,eigenvectors,eigenvalues);
		esum = eigenvalues(0) + eigenvalues(1) + eigenvalues(2); 
		e1 = eigenvalues(2) / esum;
		e2 = eigenvalues(1) / esum;
		e3 = eigenvalues(0) / esum;
		features[0] = e3;
		features[1] = e1 - e2;
		features[2] = e2 - e3;
		features[3] = (e2 - e3) / e1;
		features[4] = (e1 * log(e1)) + (e2 * log(e2)) + (e3 * log(e3)); 
		features[5] = (e1 - e2) / e1;
		for(int k=0;k<sizeof(features)/sizeof(features[0]);k++) featmatrix(k,i) = features[k];
		kcloud->clear();
	}
	bool status = model.learn(featmatrix,n_gaus,arma::gmm_dist_mode(dist_mode),arma::gmm_seed_mode(seed_mode),km_iter,em_iter,1e-10,false);
}

void gmmByCluster(std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clouds, arma::uword n_gaus, int dist_mode, int seed_mode, int km_iter, int em_iter, arma::mat &featmatrix, arma::gmm_full &model)
{
	Eigen::Vector4f centroid;
	Eigen::Matrix3f covariancematrix,eigenvectors;
	Eigen::Vector3f eigenvalues;
	float esum,e1,e2,e3;
	float features[6];
	featmatrix.resize(6,clouds.size());
	featmatrix.zeros();
	for(int i=0;i<clouds.size();i++)
	{
		computePCA(clouds[i],centroid,covariancematrix,eigenvectors,eigenvalues);
		esum = eigenvalues(0) + eigenvalues(1) + eigenvalues(2); 
		e1 = eigenvalues(2) / esum;
		e2 = eigenvalues(1) / esum;
		e3 = eigenvalues(0) / esum;
		//UGLY:
		if(e3 == 0) e3 = e2 * 0.01;
		//
		features[0] = e3;
		features[1] = e1 - e2;
		features[2] = e2 - e3;
		features[3] = (e2 - e3) / e1;
		features[4] = (e1 * log(e1)) + (e2 * log(e2)) + (e3 * log(e3)); 
		features[5] = (e1 - e2) / e1;
		for(int j=0;j<sizeof(features)/sizeof(features[0]);j++) featmatrix(j,i) = features[j];
	}
	bool status = model.learn(featmatrix,n_gaus,arma::gmm_dist_mode(dist_mode),arma::gmm_seed_mode(seed_mode),km_iter,em_iter,1e-10,false);
}

std::vector<int> classifyGmmPointModel(pcl::PointCloud<PointTreeseg>::Ptr &cloud, int n_gaus, arma::mat &featmatrix, arma::gmm_diag &model)
{
	int features =6;
	Eigen::MatrixXf means(features,n_gaus);
	for(int i=0;i<features;i++)
	{
		for(int j=0;j<n_gaus;j++) means(i,j) = model.means(i,j);
	}
	Eigen::MatrixXf normmeans(features,n_gaus);
        for(int i=0;i<features;i++)
        {       
                float min = means.row(i).minCoeff();
                float max = means.row(i).maxCoeff();
                for(int j=0;j<n_gaus;j++) normmeans(i,j) = (means(i,j) - min) / (max - min);
        }
	Eigen::VectorXf wood(features);
	wood << 0, 1, 0, 0, 1, 1;
	Eigen::VectorXf normd = (normmeans.colwise() - wood).colwise().squaredNorm();
        std::vector<int> woodlist(n_gaus,0);
        for(int i=0;i<normd.size();i++)
        {
                if(normd[i] < 0.5) woodlist[i] = 1;
        }
	arma::urowvec ids = model.assign(featmatrix.cols(0,cloud->points.size()-1),arma::eucl_dist);
	std::vector<int> classifications;
	for(int i=0;i<cloud->points.size();i++)
	{
		if(woodlist[ids[i]] == 1) classifications.push_back(0);
                else if(woodlist[ids[i]] == 0) classifications.push_back(1);
	}
	return classifications;
}

std::vector<int> classifyGmmClusterModel(std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clouds, int n_gaus, arma::mat &featmatrix, arma::gmm_full &model)
{
	int features = 6;
	Eigen::MatrixXf means(features,n_gaus);
	for(int i=0;i<features;i++)
	{
		for(int j=0;j<n_gaus;j++) means(i,j) = model.means(i,j);
	}
	Eigen::MatrixXf normmeans(features,n_gaus);
	for(int i=0;i<features;i++)
	{
		float min = means.row(i).minCoeff();
		float max = means.row(i).maxCoeff();
		for(int j=0;j<n_gaus;j++) normmeans(i,j) = (means(i,j) - min) / (max - min);
	}
	Eigen::VectorXf wood(features);
	wood << 0, 1, 0, 0, 1, 1;
	Eigen::VectorXf normd = (normmeans.colwise() - wood).colwise().squaredNorm();
	std::vector<int> woodlist(n_gaus,0);
	for(int i=0;i<normd.size();i++)
	{
		if(normd[i] < 0.5) woodlist[i] = 1;
	}
	std::vector<int> classifications;
	arma::urowvec ids = model.assign(featmatrix.cols(0,clouds.size()-1),arma::eucl_dist);
	for(int i=0;i<clouds.size();i++)
	{
		if(woodlist[ids[i]] == 1) classifications.push_back(0);
		else if(woodlist[ids[i]] == 0) classifications.push_back(1);
	}
	return classifications;
}

void separateCloudsClassifiedByPoint(pcl::PointCloud<PointTreeseg>::Ptr &cloud, std::vector<int> classifications, std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &classifiedclouds)
{
	auto minmax = minmax_element(classifications.begin(),classifications.end());
	int min = *(minmax.first);
	int max = *(minmax.second);
	for(int i=0;i<=max;i++)
	{
		pcl::PointCloud<PointTreeseg>::Ptr ccloud(new pcl::PointCloud<PointTreeseg>);
		classifiedclouds.push_back(ccloud);
	}
	for(int j=0;j<cloud->points.size();j++) classifiedclouds[classifications[j]]->insert(classifiedclouds[classifications[j]]->end(),cloud->points[j]);
}

void separateCloudsClassifiedByCluster(std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clouds, std::vector<int> classifications, std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &classifiedclouds)
{
	auto minmax = minmax_element(classifications.begin(),classifications.end());
	int min = *(minmax.first);
	int max = *(minmax.second);
	for(int i=0;i<=max;i++)
	{
		pcl::PointCloud<PointTreeseg>::Ptr ccloud(new pcl::PointCloud<PointTreeseg>);
		classifiedclouds.push_back(ccloud);
	}
	for(int j=0;j<clouds.size();j++)
	{
		for(int k=0;k<clouds[j]->points.size();k++) classifiedclouds[classifications[j]]->insert(classifiedclouds[classifications[j]]->end(),clouds[j]->points[k]);
	}
}

void writeCloudClassifiedByPoint(pcl::PointCloud<PointTreeseg>::Ptr &cloud, std::vector<int> &classifications, std::string fname)
{
	pcl::PCDWriter writer;
	auto minmax = minmax_element(classifications.begin(),classifications.end());
	int min = *(minmax.first);
	int max = *(minmax.second);
	int colours[max+1][3];
	for(int i=0;i<max;i++)
	{
		colours[i][0] = rand()%256;
		colours[i][1] = rand()%256;
		colours[i][2] = rand()%256;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>);
	//srand(time(NULL));
	for(int j=0;j<cloud->points.size();j++)
	{
		pcl::PointXYZRGB point;
		point.x = cloud->points[j].x;
		point.y = cloud->points[j].y;
		point.z = cloud->points[j].z;
		point.r = colours[classifications[j]][0];
		point.g = colours[classifications[j]][1];
		point.b = colours[classifications[j]][2];
		out->insert(out->end(),point);
	}
	writer.write(fname,*out,true);
}

void writeCloudClassifiedByCluster(std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clouds, std::vector<int> &classifications, std::string fname)
{
	pcl::PCDWriter writer;
	auto minmax = minmax_element(classifications.begin(),classifications.end());
	int min = *(minmax.first);
	int max = *(minmax.second);
	int colours[max+1][3];
	for(int i=0;i<max;i++)
	{
		colours[i][0] = rand()%256;
		colours[i][1] = rand()%256;
		colours[i][2] = rand()%256;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>);
	for(int j=0;j<clouds.size();j++)
	{
		for(int k=0;k<clouds[j]->points.size();k++)
		{
			pcl::PointXYZRGB point;
			point.x = clouds[j]->points[k].x;
			point.y = clouds[j]->points[k].y;
			point.z = clouds[j]->points[k].z;
			point.r = colours[classifications[j]][0];
			point.g = colours[classifications[j]][1];
			point.b = colours[classifications[j]][2];
			out->insert(out->end(),point);
		}
	}	
	writer.write(fname,*out,true);
}
