/*
* treeseg.cpp
*
* MIT License
*
* Copyright 2017-2021 Andrew Burt - a.burt@ucl.ac.uk
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

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/octree/octree.h>

//File IO

std::vector<std::string> getFileID(std::string filename)
{
	//Inflexible naming convention:
	//From rxp2pcd: ./DIR/PLOT.tile.number.pcd (plot can include hyphen e.g., FGC01 or FGC-01)
	//From downsample/thin: ./DIR/PLOT.tile.downsample(thin).number.pcd
	//Onwards: ./DIR/PLOT.X.number.pcd
	std::string pid,tid;
	std::vector<std::string> info;
	std::vector<std::string> tmp1,tmp2,tmp3;
	boost::split(tmp1,filename,boost::is_any_of("/"));
	boost::split(tmp2,tmp1[tmp1.size()-1],boost::is_any_of("."));
	if(tmp2[1] == "tile")
	{
		pid = tmp2[0];
		if(tmp2[2] == "downsample" || tmp2[2] == "thin") tid = tmp2[3];
		else tid = tmp2[2];
		info.push_back(pid);
		info.push_back(tid);
		return info;
	}
	else
	{
		pid = tmp2[0];
		tid = tmp2[tmp2.size()-2];
		info.push_back(pid);
		info.push_back(tid);
		return info;
	}
}

void readTiles(const std::vector<std::string> &args, pcl::PointCloud<PointTreeseg>::Ptr &cloud)
{
	pcl::PCDReader reader;
	pcl::PointCloud<PointTreeseg>::Ptr tmp(new pcl::PointCloud<PointTreeseg>);
	for(int i=0;i<args.size();i++)
	{
		std::string filename = args[i];
		std::vector<std::string> tmp1,tmp2;
		boost::split(tmp1,filename,boost::is_any_of("/"));
		boost::split(tmp2,tmp1[tmp1.size()-1],boost::is_any_of("."));
		if(tmp2.size() > 1)
		{
			if(tmp2[1] == "tile")
			{
				reader.read(args[i],*tmp);
				*cloud += *tmp;
				tmp->clear();	
			}
		}
	}
}

int getTilesStartIdx(const std::vector<std::string> &args)
{
	int start_tiles;
	for(int i=0;i<args.size();i++)
	{
		std::string filename = args[i];
		std::vector<std::string> tmp1,tmp2;
		boost::split(tmp1,filename,boost::is_any_of("/"));
		boost::split(tmp2,tmp1[tmp1.size()-1],boost::is_any_of("."));
		if(tmp2.size() > 1)
		{
			if(tmp2[1] == "tile")
			{
				start_tiles = i;
				break;
			}

		}
	}
	return start_tiles;
}

void writeClouds(const std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clouds, std::string fname, bool doPCA)
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

//Nearest neighbour analysis

std::vector<float> dNN(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest)
{
	std::vector<float> dist;
	pcl::KdTreeFLANN<PointTreeseg> tree;
	tree.setInputCloud(cloud);
	int k = nnearest + 1;
	for(pcl::PointCloud<PointTreeseg>::iterator it=cloud->begin();it!=cloud->end();it++)
	{
		std::vector<float> p_dist;
		PointTreeseg searchPoint;
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

std::vector<std::vector<float>> dNNz(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest, float zstep)
{
	std::vector<std::vector<float>> results;
	Eigen::Vector4f min,max;
	pcl::getMinMax3D(*cloud,min,max);
	pcl::PointCloud<PointTreeseg>::Ptr tmp(new pcl::PointCloud<PointTreeseg>);
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

float interpolatedNNZ(float x, const std::vector<std::vector<float>> &nndata, bool extrapolate)
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

//Cloud metrics

void getCloudMetrics(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, cloudmetrics metrics)
{
	Eigen::Vector4f min3D;
	Eigen::Vector4f max3D;
	Eigen::Vector4f centroid;
	Eigen::Matrix3f covariancematrix;
	Eigen::Matrix3f eigenvectors;
	Eigen::Vector3f eigenvalues;
	pcl::getMinMax3D(*cloud,min3D,max3D);
	computePCA(cloud,centroid,covariancematrix,eigenvectors,eigenvalues);
	float length = getCloudLength(cloud,centroid,eigenvectors);
	Eigen::Vector3f vector3D(eigenvectors(0,2),eigenvectors(1,2),eigenvectors(2,2));
	metrics.count = cloud->points.size();
	metrics.min3D = min3D;
	metrics.max3D = max3D;
	metrics.centroid = centroid;
	metrics.covariancematrix = covariancematrix;
	metrics.eigenvectors = eigenvectors;
	metrics.eigenvalues = eigenvalues;
	metrics.vector3D = vector3D; 
	metrics.length = length;
}

float getCloudLength(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, const Eigen::Vector4f &centroid, const Eigen::Matrix3f &eigenvectors)
{
	Eigen::Vector3f point(centroid[0],centroid[1],centroid[2]);
	Eigen::Vector3f direction(eigenvectors(0,2),eigenvectors(1,2),eigenvectors(2,2));
	Eigen::Affine3f transform;
	Eigen::Vector3f world(0,direction[2],-direction[1]);
	direction.normalize();
	pcl::getTransformationFromTwoUnitVectorsAndOrigin(world,direction,point,transform);
	pcl::PointCloud<PointTreeseg>::Ptr transformedcloud(new pcl::PointCloud<PointTreeseg>);
	pcl::transformPointCloud(*cloud,*transformedcloud,transform);
	Eigen::Vector4f min,max;
	pcl::getMinMax3D(*transformedcloud,min,max);
	float length = max[2]-min[2];
	return length;
}

void getBasicCloudMetrics(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, basiccloudmetrics metrics)
{
	Eigen::Vector4f min3D;
	Eigen::Vector4f max3D;
	pcl::getMinMax3D(*cloud,min3D,max3D);
	metrics.count = cloud->points.size();
	metrics.min3D = min3D;
	metrics.max3D = max3D;
}

//Downsampling

void downsample(const pcl::PointCloud<PointTreeseg>::Ptr &original, float edgelength, pcl::PointCloud<PointTreeseg>::Ptr &filtered, bool octree)
{
	if(octree == false)
	{
		pcl::VoxelGrid<PointTreeseg> downsample;
		downsample.setInputCloud(original);
		downsample.setLeafSize(edgelength,edgelength,edgelength);
		downsample.filter(*filtered);
	}
	else
	{
		pcl::octree::OctreePointCloudSearch<PointTreeseg> octree(edgelength);
		octree.setInputCloud(original);
		octree.defineBoundingBox();
		octree.addPointsFromInputCloud();
		pcl::PointCloud<PointTreeseg>::VectorType voxelcentres;
		octree.getOccupiedVoxelCenters(voxelcentres);
		for(int i=0;i<voxelcentres.size();i++)
		{
			std::vector<int> voxelinliersidx;
			octree.voxelSearch(voxelcentres[i],voxelinliersidx);
			pcl::PointCloud<PointTreeseg>::Ptr voxelinliers(new pcl::PointCloud<PointTreeseg>);
			for(int j=0;j<voxelinliersidx.size();j++) voxelinliers->insert(voxelinliers->end(),original->points[voxelinliersidx[j]]);
			PointTreeseg centroid;
			pcl::computeCentroid(*voxelinliers,centroid);
			filtered->insert(filtered->end(),centroid);
		}
	}
}

//Spatial Filters

void spatial1DFilter(const pcl::PointCloud<PointTreeseg>::Ptr &original, std::string dimension, float min, float max, pcl::PointCloud<PointTreeseg>::Ptr &filtered)
{
	pcl::PassThrough<PointTreeseg> pass;
	pass.setInputCloud(original);
	pass.setFilterFieldName(dimension);
	pass.setFilterLimits(min,max);
	pass.filter(*filtered);
}

void spatial3DCylinderFilter(const pcl::PointCloud<PointTreeseg>::Ptr &original, cylinder cyl, pcl::PointCloud<PointTreeseg>::Ptr &filtered)
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

//Clustering

void euclideanClustering(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, float dmax, int nmin, std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clusters)
{
	std::vector<pcl::PointIndices> indices;
	pcl::search::KdTree<PointTreeseg>::Ptr tree(new pcl::search::KdTree<PointTreeseg>);
	tree->setInputCloud(cloud);
	pcl::EuclideanClusterExtraction<PointTreeseg> ec;
	ec.setClusterTolerance(dmax);
	ec.setMinClusterSize(nmin);
	ec.setMaxClusterSize(std::numeric_limits<int>().max());
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(indices);
	for(std::vector<pcl::PointIndices>::iterator it=indices.begin();it!=indices.end();it++)
	{
		pcl::PointCloud<PointTreeseg>::Ptr tmpcloud(new pcl::PointCloud<PointTreeseg>);
		for(std::vector<int>::iterator pit=it->indices.begin();pit!=it->indices.end();pit++)
		{
			tmpcloud->insert(tmpcloud->end(),cloud->points[*pit]);
		}
		clusters.push_back(tmpcloud);
	}
}

//Principal component analysis

void computePCA(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, Eigen::Vector4f &centroid, Eigen::Matrix3f &covariancematrix, Eigen::Matrix3f &eigenvectors, Eigen::Vector3f &eigenvalues)
{
	pcl::compute3DCentroid(*cloud,centroid);
	pcl::computeCovarianceMatrix(*cloud,centroid,covariancematrix);
	pcl::eigen33(covariancematrix,eigenvectors,eigenvalues);
}

//Surface normals

void estimateNormals(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest, pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
	pcl::search::KdTree<PointTreeseg>::Ptr tree(new pcl::search::KdTree<PointTreeseg>());
	tree->setInputCloud(cloud);
	pcl::NormalEstimation<PointTreeseg,pcl::Normal> ne;
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud);
	ne.setKSearch(nnearest);
	//ne.setRadiusSearch(0.03);
	ne.compute(*normals);
}

//Segmentation

void regionSegmentation(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals, int nneighbours, int nmin, int nmax, float smoothness, float curvature, std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &regions)
{
	std::vector<pcl::PointIndices> indices;
	pcl::search::KdTree<PointTreeseg>::Ptr tree(new pcl::search::KdTree<PointTreeseg> ());
	tree->setInputCloud(cloud);
	pcl::RegionGrowing<PointTreeseg,pcl::Normal> reg;
	reg.setInputCloud(cloud);
	reg.setInputNormals(normals);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(nneighbours);
	reg.setMinClusterSize(nmin);
	reg.setMaxClusterSize(nmax);
	reg.setSmoothnessThreshold(smoothness * (M_PI / 180.0));
	reg.setCurvatureThreshold(curvature);
	reg.extract(indices);
	for(std::vector<pcl::PointIndices>::iterator it=indices.begin();it!=indices.end();it++)
	{       
		pcl::PointCloud<PointTreeseg>::Ptr tmpcloud(new pcl::PointCloud<PointTreeseg>);
		for(std::vector<int>::iterator pit=it->indices.begin();pit!=it->indices.end();pit++)
		{       
			tmpcloud->insert(tmpcloud->end(),cloud->points[*pit]);
		}
		regions.push_back(tmpcloud);
	}
}

//Shape fitting

void fitPlane(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals, float dthreshold, pcl::PointIndices::Ptr &inliers, float nweight, float angle, Eigen::Vector3f axis)
{
	pcl::SACSegmentationFromNormals<PointTreeseg,pcl::Normal> seg;
	seg.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setInputCloud(cloud);
	seg.setInputNormals(normals);
	seg.setOptimizeCoefficients(true);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(dthreshold);
	seg.setNormalDistanceWeight(nweight);
	seg.setAxis(axis);
	seg.setEpsAngle(angle * (M_PI / 180.0));
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	seg.segment(*inliers,*coefficients);	
}

std::vector<float> fitCircle(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest)
{
	std::vector<float> nndata;
	nndata = dNN(cloud,nnearest);
	float nndist = nndata[0];
	pcl::PointIndices inliers;
	pcl::ModelCoefficients coefficients;
	pcl::SACSegmentation<PointTreeseg> seg;
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

void fitCylinder(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest, bool finite, bool diagnostics, cylinder &cyl)
{
	cyl.ismodel = false;
	if(cloud->points.size() >= 10)
	{
		std::vector<float> nndata;
		nndata  = dNN(cloud,nnearest);
		float nndist = nndata[0];
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		estimateNormals(cloud,nnearest,normals);
		pcl::search::KdTree<PointTreeseg>::Ptr tree(new pcl::search::KdTree<PointTreeseg>());
		tree->setInputCloud(cloud);
		pcl::PointIndices indices;
		pcl::ModelCoefficients coeff;
		pcl::SACSegmentationFromNormals<PointTreeseg, pcl::Normal> seg;
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
			pcl::PointCloud<PointTreeseg>::Ptr inliers(new pcl::PointCloud<PointTreeseg>);
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
			
			cyl.steprad = 0;//init these to zero as we are not always setting in these in cylinderDiagnostics
			cyl.stepcov = 0;
			cyl.radratio = 0;
			
			cyl.cloud = cloud;
			cyl.inliers = inliers;
			if(cyl.rad > 0 && finite == false && diagnostics == false) cyl.ismodel = true;
			if(cyl.rad > 0 && finite == true)
			{
				pcl::PointCloud<PointTreeseg>::Ptr inliers_transformed(new pcl::PointCloud<PointTreeseg>);
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

void cylinderDiagnostics(cylinder &cyl, int nnearest)
{
	int NSTEP = 6;
	pcl::PointCloud<PointTreeseg>::Ptr inliers_transformed(new pcl::PointCloud<PointTreeseg>);
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
		pcl::PointCloud<PointTreeseg>::Ptr slice(new pcl::PointCloud<PointTreeseg>);
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

//Generic

bool sort2DFloatVectorByCol1(const std::vector<float> &v1, const std::vector<float> &v2)
{
        return v1[1] < v2[1];
}

bool sort2DFloatVectorByCol2(const std::vector<float> &v1, const std::vector<float> &v2)
{
        return v1[2] < v2[2];
}

bool sortCloudByZ(const PointTreeseg &p1, const PointTreeseg &p2)
{
	return p1.z < p2.z;
}

std::vector<int> nearestIdx(const pcl::PointCloud<PointTreeseg>::Ptr &searchpoints, const pcl::PointCloud<PointTreeseg>::Ptr &cloud)
{
	std::vector<int> idxs;
	pcl::KdTreeFLANN<PointTreeseg> tree;
	tree.setInputCloud(cloud);
	for(int i=0;i<searchpoints->points.size();i++)
	{
		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);
		tree.nearestKSearch(searchpoints->points[i],1,pointIdxNKNSearch,pointNKNSquaredDistance);
		idxs.push_back(pointIdxNKNSearch[0]);
	}
	return idxs;
}

int findClosestIdx(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, const std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clouds, bool biggest)
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

int findPrincipalCloudIdx(const std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clouds)
{
	std::vector<std::vector<float>> info;
	for(int i=0;i<clouds.size();i++)
	{
		Eigen::Vector4f min,max;
		pcl::getMinMax3D(*clouds[i],min,max);
		std::vector<float> tmp;
		tmp.push_back(i);
		tmp.push_back(clouds[i]->points.size());
		tmp.push_back(min[2]);
		info.push_back(tmp);
	}
	std::sort(info.begin(),info.end(),sort2DFloatVectorByCol2);
	float zrange = abs(info[info.size()-1][2]-info[0][2]);
	float zpercentile = (5.0/100.0)*zrange;
	float zmax = info[0][2]+zpercentile;
	int idx = 0;
	int pcount = 0;
	for(int i=0;i<info.size();i++)
	{
		if(info[i][1] > pcount)
		{
			if(info[i][2] < zmax)
			{
				idx = info[i][0];
				pcount = info[i][1];
			}
		}
	}
	return idx;
}

void extractIndices(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, const pcl::PointIndices::Ptr &inliers, bool invert, pcl::PointCloud<PointTreeseg>::Ptr &filtered)
{
	pcl::ExtractIndices<PointTreeseg> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(invert);
	extract.filter(*filtered);
}

float minDistBetweenClouds(const pcl::PointCloud<PointTreeseg>::Ptr &a, const pcl::PointCloud<PointTreeseg>::Ptr &b)
{
	pcl::KdTreeFLANN<PointTreeseg> kdtree;
	kdtree.setInputCloud(a);
	return minDistBetweenClouds(a,b,kdtree);
}

float minDistBetweenClouds(const pcl::PointCloud<PointTreeseg>::Ptr &a, const pcl::PointCloud<PointTreeseg>::Ptr &b, const pcl::KdTreeFLANN<PointTreeseg> &kdtree)
{
	//assuming a is the larger of the two clouds
	float distance = std::numeric_limits<float>::infinity();
	int K = 1;
	for(pcl::PointCloud<PointTreeseg>::iterator it=b->begin();it!=b->end();it++)
	{
		std::vector<float> p_dist;
		PointTreeseg searchPoint;
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

bool intersectionTest3DBox(const Eigen::Vector4f &amin, const Eigen::Vector4f &amax, const Eigen::Vector4f &bmin, const Eigen::Vector4f &bmax)
{
	bool intersection = false;
	if(amin[0] <= bmax[0] && amax[0] >= bmin[0])
	{
		if(amin[1] <= bmax[1] && amax[1] >= bmin[1])
		{
			if(amin[2] <= bmax[2] && amax[2] >= bmin[2])
			{
				intersection = true;
			}
		}
	}
	return intersection;
}

void catIntersectingClouds(std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clouds)
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

//treeseg specific

std::vector<std::vector<float>> getDemAndSlice(const pcl::PointCloud<PointTreeseg>::Ptr &plot, float resolution, float percentile, float zmin, float zmax, pcl::PointCloud<PointTreeseg>::Ptr &slice)
{
	std::vector<std::vector<float>> dem;
	std::vector<float> result;
	pcl::PointCloud<PointTreeseg>::Ptr tmpcloud(new pcl::PointCloud<PointTreeseg>);
	pcl::PointCloud<PointTreeseg>::Ptr tile(new pcl::PointCloud<PointTreeseg>);
	pcl::PointCloud<PointTreeseg>::Ptr tileslice(new pcl::PointCloud<PointTreeseg>);
	Eigen::Vector4f plotmin,plotmax;
	pcl::getMinMax3D(*plot,plotmin,plotmax);
	for(float x=plotmin[0];x<plotmax[0];x+=resolution)
	{
		spatial1DFilter(plot,"x",x,x+resolution,tmpcloud);
		for(float y=plotmin[1];y<plotmax[1];y+=resolution)
		{
			spatial1DFilter(tmpcloud,"y",y,y+resolution,tile);
			std::sort(tile->points.begin(),tile->points.end(),sortCloudByZ);
			int idx = (percentile / 100) * tile->points.size();
			float ground = tile->points[idx].z;
			result.push_back(x);
			result.push_back(y);
			result.push_back(ground);
			dem.push_back(result);
			spatial1DFilter(tile,"z",ground+zmin,ground+zmax,tileslice);
			*slice += *tileslice;
			result.clear();
			tile->clear();
			tileslice->clear();
		}
		tmpcloud->clear();
	}
	return dem;
}

void correctStem(const pcl::PointCloud<PointTreeseg>::Ptr &stem, float nnearest, float zstart, float zstep, float stepcovmax, float radchangemin, pcl::PointCloud<PointTreeseg>::Ptr &corrected)
{
	Eigen::Vector4f min, max;
	pcl::getMinMax3D(*stem,min,max);
	float zstop;
	bool broken = false;
	for(float z=min[2]+zstart;z<max[2];z+=zstep)
	{
		pcl::PointCloud<PointTreeseg>::Ptr slice(new pcl::PointCloud<PointTreeseg>);	
		spatial1DFilter(stem,"z",z,z+zstep,slice);
		std::vector<float> circle = fitCircle(slice,nnearest);
		Eigen::Vector4f zmin,zmax;
		pcl::getMinMax3D(*slice,zmin,zmax);
		int steps = 5;
		float dz = (zmax[2]-zmin[2])/float(steps);
		std::vector<float> zrad;
		for(int i=0;i<steps;i++)
		{
			pcl::PointCloud<PointTreeseg>::Ptr zslice(new pcl::PointCloud<PointTreeseg>);	
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
	if(broken == true) spatial1DFilter(stem,"z",min[2],zstop,corrected);
	else spatial1DFilter(stem,"z",min[2],max[2]-zstep,corrected);
}

void removeFarRegions(float dmin, std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &regions)
{
	std::vector<pcl::KdTreeFLANN<PointTreeseg>> kdtrees;
	for(int i=0;i<regions.size();i++)
	{
		pcl::KdTreeFLANN<PointTreeseg> tree;
		tree.setInputCloud(regions[i]);
		kdtrees.push_back(tree);
	}
	int principalidx = findPrincipalCloudIdx(regions);
	Eigen::Vector4f pmin,pmax;
	pcl::getMinMax3D(*regions[principalidx],pmin,pmax);
	int mincount = static_cast<int>(static_cast<float>(regions[principalidx]->points.size() * 0.25));
	std::vector<int> remove_list;
	for(int i=0;i<regions.size();i++)
	{
		if(i != principalidx && regions[i]->points.size() > mincount)
		{
			Eigen::Vector4f cmin,cmax;
			pcl::getMinMax3D(*regions[i],cmin,cmax);
			if(cmin[2] < pmax[2])
			{
				float d = 0;
				if(regions[principalidx]->points.size() >= regions[i]->points.size()) d = minDistBetweenClouds(regions[principalidx],regions[i],kdtrees[principalidx]);
				else d = minDistBetweenClouds(regions[i],regions[principalidx],kdtrees[i]);
				if(d > dmin) remove_list.push_back(i);
			}
		}

	}
	std::sort(remove_list.begin(),remove_list.end(),std::greater<int>());
	for(int k=0;k<remove_list.size();k++)
	{
		regions.erase(regions.begin()+remove_list[k]);
	}
}

void buildTree(const std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &regions, int cyclecount, int firstcount, float firstdist, int nnearest,  float seconddist, pcl::PointCloud<PointTreeseg>::Ptr &tree)
{
	std::vector<int> unallocated;
	std::vector<int> allocated;
	std::vector<int> previouslyallocated;
	std::vector<int> newlyallocated;
	std::vector<pcl::KdTreeFLANN<PointTreeseg>> kdtrees;
	for(int i=0;i<regions.size();i++)
	{
		unallocated.push_back(i);
		pcl::KdTreeFLANN<PointTreeseg> tree;
		tree.setInputCloud(regions[i]);
		kdtrees.push_back(tree);
	}
	int idx = findPrincipalCloudIdx(regions);
	allocated.push_back(unallocated[idx]);
	previouslyallocated.push_back(allocated[0]);
	unallocated.erase(std::remove(unallocated.begin(),unallocated.end(),allocated[0]),unallocated.end());
	bool donesomething = true;
	int count = 0;
	while(donesomething == true && count < cyclecount)
	{
		for(int i=0;i<previouslyallocated.size();i++)
		{
			std::vector<std::vector<float>> dinfo;
			for(int j=0;j<unallocated.size();j++)
			{
				float d = std::numeric_limits<int>::max();
				if(regions[previouslyallocated[i]]->points.size() >= regions[unallocated[j]]->points.size())
				{
					d = minDistBetweenClouds(regions[previouslyallocated[i]],regions[unallocated[j]],kdtrees[previouslyallocated[i]]);
				}	
				else
				{
					d = minDistBetweenClouds(regions[unallocated[j]],regions[previouslyallocated[i]],kdtrees[unallocated[j]]);
				}
				std::vector<float> tmp;
				tmp.push_back(unallocated[j]);
				tmp.push_back(d);
				dinfo.push_back(tmp);
			}
			if(dinfo.size() > 0)
			{
				std::sort(dinfo.begin(),dinfo.end(),sort2DFloatVectorByCol1);
				if(count < firstcount)
				{
					for(int j=0;j<dinfo.size();j++)
					{
						if(dinfo[j][1] < firstdist)
						{
							int tmpidx = static_cast<int>(dinfo[j][0]);
							newlyallocated.push_back(tmpidx);
						}						
					}
				}
				else
				{
					for(int j=0;j<dinfo.size();j++)
					{
						if(j == nnearest) break;
						else
						{
							if(dinfo[j][1] < seconddist)
							{
								int tmpidx = static_cast<int>(dinfo[j][0]);
								newlyallocated.push_back(tmpidx);
							}
						}
					}
				}
			}
		}
		previouslyallocated.clear();
		if(newlyallocated.size() > 0)
		{
			std::sort(newlyallocated.begin(),newlyallocated.end());
			newlyallocated.erase(std::unique(newlyallocated.begin(),newlyallocated.end()),newlyallocated.end());
			for(int i=0;i<newlyallocated.size();i++)
			{
				allocated.push_back(newlyallocated[i]);
				previouslyallocated.push_back(newlyallocated[i]);
				unallocated.erase(std::remove(unallocated.begin(),unallocated.end(),newlyallocated[i]),unallocated.end());
			}
			newlyallocated.clear();
		}
		else donesomething = false;
		count++;
		std::cout << "." << std::flush;
	}
	for(int i=0;i<allocated.size();i++) *tree += *regions[allocated[i]];
}

treeparams getTreeParams(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest, float zstep, float diffmax)
{       
	treeparams params;
	Eigen::Vector4f min,max;
	pcl::getMinMax3D(*cloud,min,max);
	float z = min[2] + 1.3;
	int i=0;
	bool stable = false;
	while(stable == false)
	{
		if(z >= max[2]-zstep*1.5) break;
		else
		{
			pcl::PointCloud<PointTreeseg>::Ptr slice(new pcl::PointCloud<PointTreeseg>);
			pcl::PointCloud<PointTreeseg>::Ptr bslice(new pcl::PointCloud<PointTreeseg>);
			pcl::PointCloud<PointTreeseg>::Ptr fslice(new pcl::PointCloud<PointTreeseg>);
			spatial1DFilter(cloud,"z",z-zstep/2,z+zstep/2,slice);
			spatial1DFilter(cloud,"z",z-zstep*1.5,z-zstep/2,bslice);
			spatial1DFilter(cloud,"z",z+zstep/2,z+zstep*1.5,fslice);
			if(i == 0)
			{
				Eigen::Vector4f smin,smax;
				pcl::getMinMax3D(*slice,smin,smax);
				params.x = (smax[0] + smin[0]) / 2;
				params.y = (smax[1] + smin[1]) / 2;
				params.h = max[2]-min[2];
        			params.c = sqrt(pow(max[0]-min[0],2)+pow(max[1]-min[1],2));
			}
			cylinder cyl,bcyl,fcyl;
			fitCylinder(slice,nnearest,false,true,cyl);
			fitCylinder(bslice,nnearest,false,false,bcyl);
			fitCylinder(fslice,nnearest,false,false,fcyl);
			float d = (cyl.rad + bcyl.rad + fcyl.rad) / 3 * 2;
			float bdiff = fabs(cyl.rad - bcyl.rad) / cyl.rad;
			float fdiff =  fabs(cyl.rad - fcyl.rad) / cyl.rad;
			float diff = (bdiff + fdiff) / 2;
			if(cyl.ismodel == true && diff <= diffmax)
			{
				params.d = d;
				stable = true;
			}
		}
		z += 0.1;
		i++;
	}
	return params;
}
