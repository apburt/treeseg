/*
* treeseg.cpp
*
* MIT License
*
* Copyright 2017-2018 Andrew Burt - a.burt@ucl.ac.uk
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

// treeseg has been developed using:
// Point Cloud Library (http://www.pointclouds.org)

#include "treeseg.h"

#include <numeric>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <Eigen/Core>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>

std::vector<std::string> getFileID(char *fname)
{
	//Assuming ../DIR/PLOT_ID.XXX.PCD
	std::string tmp(fname);
	std::string plotid,treeid;
	std::vector<std::string> info;
	std::vector<std::string> tmp1,tmp2,tmp3; 
	boost::split(tmp1,tmp,boost::is_any_of("/"));
	boost::split(tmp2,tmp1[tmp1.size()-1],boost::is_any_of("."));
	boost::split(tmp3,tmp2[0],boost::is_any_of("_"));	
	plotid = tmp3[0];
	treeid = tmp3[1];
	info.push_back(treeid);
	info.push_back(plotid);
	return info;
}

std::vector<float> dNN(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int nnearest)
{
	std::vector<float> dist;
	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);
	int k = nnearest + 1;
	for(pcl::PointCloud<pcl::PointXYZ>::iterator it=cloud->begin();it!=cloud->end();it++)
	{
		std::vector<float> p_dist;
		pcl::PointXYZ searchPoint;
		searchPoint.x = it->x;
		searchPoint.y = it->y;
		searchPoint.z = it->z;
		std::vector<int> pointIdxNKNSearch(k);
		std::vector<float> pointNKNSquaredDistance(k);
		tree.nearestKSearch(searchPoint,k,pointIdxNKNSearch,pointNKNSquaredDistance);
		for(int i=1;i<pointIdxNKNSearch.size();i++)
		{
			p_dist.push_back(sqrt(pointNKNSquaredDistance[i]));
		}
		float p_dist_sum = std::accumulate(p_dist.begin(),p_dist.end(),0.0);
		float p_dist_mean = p_dist_sum/p_dist.size();
		dist.push_back(p_dist_mean);
	}
	float dist_sum = std::accumulate(dist.begin(),dist.end(),0.0);
	float dist_mean = dist_sum/dist.size();
	float sq_sum = std::inner_product(dist.begin(), dist.end(), dist.begin(), 0.0);
	float stddev = std::sqrt(sq_sum / dist.size() - dist_mean * dist_mean);
	std::vector<float> results;
	results.push_back(dist_mean);
	results.push_back(stddev);
	return results;
}

std::vector<std::vector<int>> nNearestIdx(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int nnearest)
{
	std::vector<std::vector<int>> nidx;
	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
        tree.setInputCloud(cloud);
	int k = nnearest + 1;
	for(pcl::PointCloud<pcl::PointXYZ>::iterator it=cloud->begin();it!=cloud->end();it++)
	{
		pcl::PointXYZ searchPoint;
                searchPoint.x = it->x;
                searchPoint.y = it->y;
                searchPoint.z = it->z;
		std::vector<int> pointIdxNKNSearch(k);
		std::vector<float> pointNKNSquaredDistance(k);
                tree.nearestKSearch(searchPoint,k,pointIdxNKNSearch,pointNKNSquaredDistance);
		nidx.push_back(pointIdxNKNSearch);	
	}
	return nidx;
}

float minDistBetweenClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr &a, pcl::PointCloud<pcl::PointXYZ>::Ptr &b)
{
	//assuming a is the larger of the two clouds
	float distance = std::numeric_limits<float>::infinity();
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(a);
	int K = 1;
	for(pcl::PointCloud<pcl::PointXYZ>::iterator it=b->begin();it!=b->end();it++)
	{
		std::vector<float> p_dist;
		pcl::PointXYZ searchPoint;
		searchPoint.x = it->x;
		searchPoint.y = it->y;
		searchPoint.z = it->z;
		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);
		kdtree.nearestKSearch(searchPoint,K,pointIdxNKNSearch,pointNKNSquaredDistance);
		float d = sqrt(pointNKNSquaredDistance[0]);
		if(d < distance) distance = d;
	}
	return distance;
}

void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &original, float edgelength, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered)
{
	pcl::VoxelGrid<pcl::PointXYZ> downsample;
	downsample.setInputCloud(original);
	downsample.setLeafSize(edgelength,edgelength,edgelength);
	downsample.filter(*filtered);
}

void extractIndices(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointIndices::Ptr &inliers, bool invert, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered)
{
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(invert);
	extract.filter(*filtered);
}

int findClosestIdx(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds, bool biggest)
{
	int idx = 0;
	std::vector<std::vector<float>> data;
	for(int i=0;i<clouds.size();i++)
	{
		float d;
		if(cloud->points.size() > clouds[i]->points.size()) d = minDistBetweenClouds(cloud,clouds[i]);
		else d = minDistBetweenClouds(clouds[i],cloud);
		std::vector<float> tmp;
		tmp.push_back(d);
		tmp.push_back(clouds[i]->points.size());
		data.push_back(tmp);
	}
	float mindist = std::numeric_limits<float>::infinity();
	for(int j=0;j<data.size();j++)
	{
		if(data[j][0] < mindist)
		{
			mindist = data[j][0];
			idx = j;
		}
	}
	if(biggest == true)
	{		
		float tolerance = 1;
		mindist = mindist + (1.0 * tolerance);
		int size = 0;
		for(int k=0;k<data.size();k++)
		{
			if(data[k][0] < mindist && int(data[k][1]) > size)
			{
				size = int(data[k][1]);
				idx = k;
			}
		}
	}
	return idx;
}

void spatial1DFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &original, std::string dimension, float min, float max, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered)
{
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(original);
	pass.setFilterFieldName(dimension);
	pass.setFilterLimits(min,max);
	pass.filter(*filtered);
}

void spatial3DCylinderFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &original, cylinder cyl, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered)
{
	float len = 1000;
	Eigen::Vector3f cp1(cyl.x+len*cyl.dx,cyl.y+len*cyl.dy,cyl.z+len*cyl.dz);
	Eigen::Vector3f cp2(cyl.x-len*cyl.dx,cyl.y-len*cyl.dy,cyl.z-len*cyl.dz);
	Eigen::Vector3f cn1 = cp2 - cp1;
	cn1.normalize();
	Eigen::Vector3f cn2 = -cn1;
	for(int i=0;i<original->points.size();i++)
	{
		Eigen::Vector3f p(original->points[i].x,original->points[i].y,original->points[i].z);
		float dot1 = cn1.dot(p-cp1);
		if(dot1 > 0)
		{
			float dot2 = cn2.dot(p-cp2);
			if(dot2 > 0)
			{
				Eigen::Vector3f mp = p - (cn1 * cn1.dot(p-cp1));
				float dist = sqrt(pow(mp(0)-cp1(0),2)+pow(mp(1)-cp1(1),2)+pow(mp(2)-cp1(2),2));
				if(dist <= cyl.rad)
				{
					filtered->insert(filtered->end(),original->points[i]);
				}
			}
		}
	}
}

std::vector<std::vector<float>> dNNz(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int nnearest, float zstep)
{
	std::vector<std::vector<float>> results;
	Eigen::Vector4f min,max;
	pcl::getMinMax3D(*cloud,min,max);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
	for(float z=min[2];z<max[2];z+=zstep)
	{
		spatial1DFilter(cloud,"z",z,z+zstep,tmp);
		if(tmp->points.size() > nnearest)
		{
			std::vector<float> nn = dNN(tmp,nnearest);
			//float pos = z - min[2];
			float pos = z + zstep;
			std::vector<float> r;
			r.push_back(pos);
			r.push_back(nn[0]);
			results.push_back(r);
		}
		tmp->clear();
	}
	return results;
}

std::vector<std::vector<float>> getDemAndSlice(pcl::PointCloud<pcl::PointXYZ>::Ptr &plot, float resolution, float zmin, float zmax, pcl::PointCloud<pcl::PointXYZ>::Ptr &slice)
{
	std::vector<std::vector<float>> dem;
	std::vector<float> result;
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmpcloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tile(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tileslice(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector4f plotmin,plotmax;
	pcl::getMinMax3D(*plot,plotmin,plotmax);
	for(float x=plotmin[0];x<plotmax[0];x+=resolution)
	{
		spatial1DFilter(plot,"x",x,x+resolution,tmpcloud);
		for(float y=plotmin[1];y<plotmax[1];y+=resolution)
		{
			spatial1DFilter(tmpcloud,"y",y,y+resolution,tile);
			Eigen::Vector4f tilemin,tilemax;
			pcl::getMinMax3D(*tile,tilemin,tilemax);
			result.push_back(x);
			result.push_back(y);
			result.push_back(tilemin[2]);
			dem.push_back(result);
			spatial1DFilter(tile,"z",tilemin[2]+zmin,tilemin[2]+zmax,tileslice);
			*slice += *tileslice;
			result.clear();
			tile->clear();
			tileslice->clear();
		}
		tmpcloud->clear();
	}
	return dem;
}

void computePCA(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::Vector4f &centroid, Eigen::Matrix3f &covariancematrix, Eigen::Matrix3f &eigenvectors, Eigen::Vector3f &eigenvalues)
{
	pcl::compute3DCentroid(*cloud,centroid);
	pcl::computeCovarianceMatrix(*cloud,centroid,covariancematrix);
	pcl::eigen33(covariancematrix,eigenvectors,eigenvalues);
}

void estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int nnearest, pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	tree->setInputCloud(cloud);
	pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud);
	ne.setKSearch(nnearest);
	ne.compute(*normals);
}

void euclideanClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float dmax, int nmin, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters)
{
	std::vector<pcl::PointIndices> indices;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(dmax);
	ec.setMinClusterSize(nmin);
	ec.setMaxClusterSize(std::numeric_limits<int>().max());
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(indices);
	for(std::vector<pcl::PointIndices>::iterator it=indices.begin();it!=indices.end();it++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr tmpcloud(new pcl::PointCloud<pcl::PointXYZ>);
		for(std::vector<int>::iterator pit=it->indices.begin();pit!=it->indices.end();pit++)
		{
			tmpcloud->insert(tmpcloud->end(),cloud->points[*pit]);
		}
		clusters.push_back(tmpcloud);
	}
}

void regionSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int nnearest, int nmin, float smoothness, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &regions)
{
	std::vector<pcl::PointIndices> indices;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	tree->setInputCloud(cloud);
	estimateNormals(cloud,nnearest,normals);
	pcl::RegionGrowing<pcl::PointXYZ,pcl::Normal> reg;
	reg.setMinClusterSize(nmin);
	reg.setMaxClusterSize(10000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(30); //leaving this fixed for now
	reg.setInputCloud(cloud);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(smoothness/180*M_PI);
	reg.setCurvatureThreshold(1); //also fixed;
	reg.extract(indices);
	for(std::vector<pcl::PointIndices>::iterator it=indices.begin();it!=indices.end();it++)
	{       
		pcl::PointCloud<pcl::PointXYZ>::Ptr tmpcloud(new pcl::PointCloud<pcl::PointXYZ>);
		for(std::vector<int>::iterator pit=it->indices.begin();pit!=it->indices.end();pit++)
		{       
			tmpcloud->insert(tmpcloud->end(),cloud->points[*pit]);
		}
		regions.push_back(tmpcloud);
	}
}

void writeClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds, std::string fname, bool doPCA)
{
	pcl::PCDWriter writer;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>);
	for(int i=0;i<clouds.size();i++)
	{
		int r = rand()%256;
		int g = rand()%256;
		int b = rand()%256;
		for(int j=0;j<clouds[i]->points.size();j++)
		{
			pcl::PointXYZRGB point;
			point.x = clouds[i]->points[j].x;
			point.y = clouds[i]->points[j].y;
			point.z = clouds[i]->points[j].z;
			point.r = r;
			point.g = g;
			point.b = b;
			out->insert(out->end(),point);
		}
	}
	writer.write(fname,*out,true);
}

std::vector<float> fitCircle(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int nnearest)
{
	std::vector<float> nndata;
	nndata = dNN(cloud,nnearest);
	float nndist = nndata[0];
	pcl::PointIndices inliers;
	pcl::ModelCoefficients coefficients;
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000000);
	seg.setModelType(pcl::SACMODEL_CIRCLE2D);
	seg.setDistanceThreshold(nndist);
	seg.setInputCloud(cloud);
	seg.segment(inliers,coefficients);
	std::vector<float> results;
	results.push_back(coefficients.values[0]);
	results.push_back(coefficients.values[1]);
	results.push_back(coefficients.values[2]);
	return results;
}

void fitCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int nnearest, bool finite, bool diagnostics, cylinder &cyl)
{
	cyl.ismodel = false;
	if(cloud->points.size() >= 10)
	{
		std::vector<float> nndata;
		nndata  = dNN(cloud,nnearest);
		float nndist = nndata[0];
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		estimateNormals(cloud,nnearest,normals);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		tree->setInputCloud(cloud);
		pcl::PointIndices indices;
		pcl::ModelCoefficients coeff;
		pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_CYLINDER);
		seg.setNormalDistanceWeight(0.1);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(1000000);
		seg.setDistanceThreshold(nndist);
		seg.setInputCloud(cloud);
		seg.setInputNormals(normals);
		seg.segment(indices,coeff);
		if(indices.indices.size() > 0)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr inliers(new pcl::PointCloud<pcl::PointXYZ>);
			for(std::vector<int>::iterator pit=indices.indices.begin();pit!=indices.indices.end();pit++)
			{
				inliers->insert(inliers->end(),cloud->points[*pit]);
			}
			cyl.x = coeff.values[0];
			cyl.y = coeff.values[1];
			cyl.z = coeff.values[2];
			cyl.dx = coeff.values[3];
			cyl.dy = coeff.values[4];
			cyl.dz = coeff.values[5];
			cyl.rad = coeff.values[6];
			cyl.cloud = cloud;
			cyl.inliers = inliers;
			if(cyl.rad > 0 && finite == false && diagnostics == false) cyl.ismodel = true;
			if(cyl.rad > 0 && finite == true)
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr inliers_transformed(new pcl::PointCloud<pcl::PointXYZ>);
				Eigen::Vector3f point(coeff.values[0],coeff.values[1],coeff.values[2]);
				Eigen::Vector3f direction(coeff.values[3],coeff.values[4],coeff.values[5]);
				Eigen::Affine3f transform;
				Eigen::Vector3f world(0,direction[2],-direction[1]);
				direction.normalize();
				pcl::getTransformationFromTwoUnitVectorsAndOrigin(world,direction,point,transform);
				pcl::transformPointCloud(*inliers,*inliers_transformed,transform);
				Eigen::Vector4f min,max;
				pcl::getMinMax3D(*inliers_transformed,min,max);
				cyl.len = max[2]-min[2];
				if(cyl.len > 0) cyl.ismodel = true;
			}
			if(cyl.rad > 0 && diagnostics == true)
			{
				cylinderDiagnostics(cyl,nnearest);
				if(cyl.steprad > 0) cyl.ismodel = true;
			}
		}
	}
}

void fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int nnearest, pcl::PointIndices::Ptr &inliers)
{
	std::vector<float> nndata = dNN(cloud,nnearest);
	float nndist = nndata[0];
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	//pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(nndist);
	seg.setInputCloud(cloud);
	seg.segment(*inliers,*coefficients);	
}

void cylinderDiagnostics(cylinder &cyl, int nnearest)
{
	int NSTEP = 6;
	pcl::PointCloud<pcl::PointXYZ>::Ptr inliers_transformed(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector3f point(cyl.x,cyl.y,cyl.z);
	Eigen::Vector3f direction(cyl.dx,cyl.dy,cyl.dz);
	Eigen::Affine3f transform;
	Eigen::Vector3f world(0,direction[2],-direction[1]);
	direction.normalize();
	pcl::getTransformationFromTwoUnitVectorsAndOrigin(world,direction,point,transform);
	pcl::transformPointCloud(*cyl.inliers,*inliers_transformed,transform);
	Eigen::Vector4f min,max;
	pcl::getMinMax3D(*inliers_transformed,min,max);
	float zstep = (max[2]-min[2]) / NSTEP;
	std::vector<float> zrads;
	for(int i=0;i<NSTEP;i++)
	{
		float zmin = min[2] + i * zstep;
		float zmax = min[2] + (i+1) * zstep;
		pcl::PointCloud<pcl::PointXYZ>::Ptr slice(new pcl::PointCloud<pcl::PointXYZ>);
		spatial1DFilter(inliers_transformed,"z",zmin,zmax,slice);
		cylinder zcyl;
		fitCylinder(slice,nnearest,false,false,zcyl);
		if(zcyl.ismodel == true) zrads.push_back(zcyl.rad);
	}
	if(zrads.size() >= NSTEP - 2)
	{
		float sum = std::accumulate(zrads.begin(),zrads.end(),0.0);
		float mean = sum/zrads.size();
		std::vector<float> diff(zrads.size());
		std::transform(zrads.begin(),zrads.end(),diff.begin(),std::bind2nd(std::minus<float>(),mean));
		float stddev = std::sqrt(std::inner_product(diff.begin(),diff.end(),diff.begin(),0.0) / zrads.size());
		cyl.steprad = mean;
		cyl.stepcov = stddev/mean;
		cyl.radratio = std::min(cyl.rad,cyl.steprad)/std::max(cyl.rad,cyl.steprad);
	}
}

bool intersectionTest3DBox(Eigen::Vector4f amin, Eigen::Vector4f amax, Eigen::Vector4f bmin, Eigen::Vector4f bmax)
{
	bool intersection = false;
	if(amin[0] < bmax[0] && amax[0] > bmin[0] &&
		amin[1] < bmax[1] && amax[1] > bmin[1] &&
			amin[2] < bmax[2] && amax[2] > bmin[2])
	{
		intersection = true;
	}
	return intersection;
}

void catIntersectingClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds)
{
	//this is poorly optimised and currently the primary bottleneck of findstems.
	bool donesomething = true;
	while(donesomething == true)
	{
		int idx;
		std::vector<float> duplicates;
		for(int i=0;i<clouds.size();i++)
		{
			Eigen::Vector4f amin,amax;
			pcl::getMinMax3D(*clouds[i],amin,amax);
			for(int j=0;j<clouds.size();j++)
			{
				if(j != i)
				{
					Eigen::Vector4f bmin,bmax;
					pcl::getMinMax3D(*clouds[j],bmin,bmax);
					bool intersects = intersectionTest3DBox(amin,amax,bmin,bmax);
					if(intersects == true) duplicates.push_back(j);
				}
			}
			if(duplicates.size() > 0)
			{
				idx = i;
				break;
			}
		}
		if(duplicates.size() > 0) 
		{
			std::sort(duplicates.begin(),duplicates.end(),std::greater<int>());
			for(int k=0;k<duplicates.size();k++)
			{
				*clouds[idx] += *clouds[duplicates[k]];				
				clouds.erase(clouds.begin()+duplicates[k]);
			}
		}
		else donesomething = false;
	}
}

void correctStem(pcl::PointCloud<pcl::PointXYZ>::Ptr &stem, float nnearest, float zstart, float zstep, float stepcovmax, float radchangemin, pcl::PointCloud<pcl::PointXYZ>::Ptr &corrected)
{
	Eigen::Vector4f min, max;
	pcl::getMinMax3D(*stem,min,max);
	float zstop;
	bool broken = false;
	for(float z=min[2]+zstart;z<max[2];z+=zstep)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr slice(new pcl::PointCloud<pcl::PointXYZ>);	
		spatial1DFilter(stem,"z",z,z+zstep,slice);
		std::vector<float> circle = fitCircle(slice,nnearest);
		Eigen::Vector4f zmin,zmax;
		pcl::getMinMax3D(*slice,zmin,zmax);
		int steps = 5;
		float dz = (zmax[2]-zmin[2])/float(steps);
		std::vector<float> zrad;
		for(int i=0;i<steps;i++)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr zslice(new pcl::PointCloud<pcl::PointXYZ>);	
			spatial1DFilter(slice,"z",z+i*dz,z+(i+1)*dz,zslice);
			if(zslice->points.size() > 10)
			{
				std::vector<float> zcircle = fitCircle(zslice,nnearest);
				zrad.push_back(zcircle[2]);
			}
		}
		float sum = std::accumulate(zrad.begin(),zrad.end(),0.0);
		float mean = sum/zrad.size();
		std::vector<float> diff(zrad.size());
		std::transform(zrad.begin(),zrad.end(),diff.begin(),std::bind2nd(std::minus<float>(),mean));
		float stdev = std::sqrt(std::inner_product(diff.begin(),diff.end(),diff.begin(),0.0) / zrad.size());
		float cov = stdev / mean;
		float radchange = std::min(circle[2],mean) / std::max(circle[2],mean);
		//std::cout << circle[2] << " " << mean << " " << cov << " " << radchange << std::endl;
		if(cov > stepcovmax || radchange < radchangemin)
		{
			zstop = z-zstep*1.5;
			broken = true;
			break;
		}
	}
	if(broken == false) spatial1DFilter(stem,"z",min[2],zstop,corrected);
	else spatial1DFilter(stem,"z",min[2],max[2]-zstep,corrected);
}

float getDBH(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int nnearest, float zstep, float diffmax)
{       
	Eigen::Vector4f min,max;
	pcl::getMinMax3D(*cloud,min,max);
	float z = min[2] + 1.3;
	float dbh;
	bool stable = false;
	int i=0;
	while(stable == false)
	{
		if(z >= max[2]-zstep*1.5) break;
		else
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr slice(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr bslice(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr fslice(new pcl::PointCloud<pcl::PointXYZ>);
			spatial1DFilter(cloud,"z",z-zstep/2,z+zstep/2,slice);
			spatial1DFilter(cloud,"z",z-zstep*1.5,z-zstep/2,bslice);
			spatial1DFilter(cloud,"z",z+zstep/2,z+zstep*1.5,fslice);
			cylinder cyl,bcyl,fcyl;
			fitCylinder(slice,nnearest,false,true,cyl);
			fitCylinder(bslice,nnearest,false,false,bcyl);
			fitCylinder(fslice,nnearest,false,false,fcyl);
			float d = (cyl.rad + bcyl.rad + fcyl.rad) / 3 * 2;
			float bdiff = fabs(cyl.rad - bcyl.rad) / cyl.rad;
			float fdiff =  fabs(cyl.rad - fcyl.rad) / cyl.rad;
			float diff = (bdiff + fdiff) / 2;
			//std::cout << cyl.rad*2 << " " << bcyl.rad*2 << " " << fcyl.rad*2 << " " << d << " " << diff << std::endl;
			if(i == 0) dbh = d;
			if(cyl.ismodel == true && diff <= diffmax)
			{
				dbh = d;
				stable = true;
			}
		}
		z += 0.1;
		i++;
	}
	return dbh;
}

bool sortCol(const std::vector<int>& v1, const std::vector<int>& v2)
{
        return v1[1] < v2[1];
}

int findPrincipalCloudIdx(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds)
{
	std::vector<std::vector<int>> info;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for(int i=0;i<clouds.size();i++) *cloud += *clouds[i];
	Eigen::Vector4f min,max;
	pcl::getMinMax3D(*cloud,min,max);
	for(int i=0;i<clouds.size();i++)
	{
		Eigen::Vector4f cmin, cmax;
		pcl::getMinMax3D(*clouds[i],cmin,cmax);
		if(cmin[2] < min[2]+2)
		{
			std::vector<int> in;
			in.push_back(i);
			in.push_back(clouds[i]->points.size());
			info.push_back(in);
		}
	}
	std::sort(info.begin(),info.end(),sortCol);
	int idx = info[info.size()-1][0];
	return idx;
}

float interpolatedNNZ(float x, std::vector<std::vector<float>> nndata, bool extrapolate)
{
	std::vector<float> xData;
	std::vector<float> yData;
	for(int m=0;m<nndata.size();m++)
	{
		xData.push_back(nndata[m][0]);
		yData.push_back(nndata[m][1]);
	}
	int size = xData.size();
	int i = 0;
	if(x >= xData[size-2]) i = size - 2;
	else
	{
		while ( x > xData[i+1] ) i++;
	}
	double xL = xData[i], yL = yData[i], xR = xData[i+1], yR = yData[i+1];
	if(!extrapolate)
	{
		if ( x < xL ) yR = yL;
		if ( x > xR ) yL = yR;
	}
	double dydx = ( yR - yL ) / ( xR - xL );
	return yL + dydx * ( x - xL );
}

void removeFarRegions(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for(int i=0;i<clusters.size();i++) *cloud += *clusters[i];
	Eigen::Vector4f min,max;
	pcl::getMinMax3D(*cloud,min,max);
	float xm = (max[0] + min[0]) / 2;
	float ym = (max[1] + min[1]) / 2;
	std::vector<float> zloc(clusters.size());
	for(int i=0;i<clusters.size();i++)
	{
		Eigen::Vector4f cmin,cmax;
		pcl::getMinMax3D(*clusters[i],cmin,cmax);
		zloc[i] = cmin[2];
	}
	std::vector<float> tmp = zloc;
	std::sort(tmp.begin(),tmp.end());
	int pos = static_cast<int>(static_cast<float>(clusters.size()) * 0.1);
	float zmax = tmp[pos]; 
	std::vector<int> remove_list;
	for(int i=0;i<clusters.size();i++)
	{
		if(zloc[i] < zmax)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr zslice(new pcl::PointCloud<pcl::PointXYZ>);
			spatial1DFilter(clusters[i],"z",-10000000,zmax,zslice);
			Eigen::Vector4f smin,smax;
			pcl::getMinMax3D(*zslice,smin,smax);
			float sxm = (smax[0] + smin[0]) / 2;
			float sym = (smax[1] + smin[1]) / 2;
			float d = sqrt(pow((sxm-xm),2) + pow((sym-ym),2));
			if(d > 0.5) remove_list.push_back(i);
		}
	}
	std::sort(remove_list.begin(),remove_list.end(),std::greater<int>());
	for(int k=0;k<remove_list.size();k++) clusters.erase(clusters.begin()+remove_list[k]);
}

void buildTree(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters, pcl::PointCloud<pcl::PointXYZ>::Ptr &tree)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmpcloud(new pcl::PointCloud<pcl::PointXYZ>);
	for(int a=0;a<clusters.size();a++) *tmpcloud += *clusters[a];
	std::vector<std::vector<float>> nndata = dNNz(tmpcloud,50,2); //careful here
	int idx = findPrincipalCloudIdx(clusters);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> treeclusters;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> outer;
	treeclusters.push_back(clusters[idx]);
	outer.push_back(clusters[idx]);
	clusters.erase(clusters.begin()+idx);
	int count = 0;
	bool donesomething = true;
	while(donesomething == true)
	{
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> tmp;
		for(int i=0;i<outer.size();i++)
		{
			std::vector<int> member;
			Eigen::Vector4f outercentroid;
			Eigen::Matrix3f outercovariancematrix;
			Eigen::Matrix3f outereigenvectors;
			Eigen::Vector3f outereigenvalues;
			pcl::PointCloud<pcl::PointXYZ>::Ptr outertransformed(new pcl::PointCloud<pcl::PointXYZ>);
			Eigen::Vector4f outermin,outermax;
			float outerlength;
			computePCA(outer[i],outercentroid,outercovariancematrix,outereigenvectors,outereigenvalues);
			Eigen::Vector3f outerpoint(outercentroid[0],outercentroid[1],outercentroid[2]);
			Eigen::Vector3f outerdirection(outereigenvectors(0,2),outereigenvectors(1,2),outereigenvectors(2,2));
			Eigen::Affine3f outertransform;
			Eigen::Vector3f outerworld(0,outerdirection[2],-outerdirection[1]);
			outerdirection.normalize();
			pcl::getTransformationFromTwoUnitVectorsAndOrigin(outerworld,outerdirection,outerpoint,outertransform);
			pcl::transformPointCloud(*outer[i],*outertransformed,outertransform);
			pcl::getMinMax3D(*outertransformed,outermin,outermax);
			outerlength = outermax[2] - outermin[2];
			for(int j=0;j<clusters.size();j++)
			{
				float d;
				Eigen::Vector4f clustercentroid;
				if(outer[i]->points.size() >= clusters[j]->points.size()) d = minDistBetweenClouds(outer[i],clusters[j]);
				else d = minDistBetweenClouds(clusters[j],outer[i]);
				pcl::compute3DCentroid(*clusters[j],clustercentroid);
				float mind = interpolatedNNZ((clustercentroid[2]+outercentroid[2])/2,nndata,true);
				if(d <= mind)
				{
					Eigen::Matrix3f clustercovariancematrix;
					Eigen::Matrix3f clustereigenvectors;
					Eigen::Vector3f clustereigenvalues;
					pcl::PointCloud<pcl::PointXYZ>::Ptr clustertransformed(new pcl::PointCloud<pcl::PointXYZ>);
					Eigen::Vector4f clustermin,clustermax;
					float clusterlength;
					computePCA(clusters[j],clustercentroid,clustercovariancematrix,clustereigenvectors,clustereigenvalues);
					Eigen::Vector3f clusterpoint(clustercentroid[0],clustercentroid[1],clustercentroid[2]);
					Eigen::Vector3f clusterdirection(clustereigenvectors(0,2),clustereigenvectors(1,2),clustereigenvectors(2,2));
					Eigen::Affine3f clustertransform;
					Eigen::Vector3f clusterworld(0,clusterdirection[2],-clusterdirection[1]);
					clusterdirection.normalize();
					pcl::getTransformationFromTwoUnitVectorsAndOrigin(clusterworld,clusterdirection,clusterpoint,clustertransform);
					pcl::transformPointCloud(*clusters[j],*clustertransformed,clustertransform);
					pcl::getMinMax3D(*clustertransformed,clustermin,clustermax);
					clusterlength = clustermax[2] - clustermin[2];
					Eigen::Vector4f outervector(outereigenvectors(0,2),outereigenvectors(1,2),outereigenvectors(2,2),0);
					Eigen::Vector4f clustervector(clustereigenvectors(0,2),clustereigenvectors(1,2),clustereigenvectors(2,2),0);
					float angle = pcl::getAngle3D(outervector,clustervector) * (180/M_PI);
					if(clusterlength < outerlength) member.push_back(j);				
				}
			}
			std::sort(member.begin(),member.end(),std::greater<int>());
			for(int k=0;k<member.size();k++)
			{
				tmp.push_back(clusters[member[k]]);
				clusters.erase(clusters.begin()+member[k]);
			}
		}
		if(tmp.size() !=0)
		{
			outer.clear();
			for(int m=0;m<tmp.size();m++)
			{
				treeclusters.push_back(tmp[m]);
				outer.push_back(tmp[m]);
			}
		}
		else donesomething = false;
		count++;
		std::cout << "." << std::flush;
	}
	for(int n=0;n<treeclusters.size();n++) *tree += *treeclusters[n];
}
