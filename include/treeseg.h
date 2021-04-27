/*
* treeseg.h
*
* MIT License
*
* Copyright 2017 Andrew Burt
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

#include "treeseg_pointtype.h"

#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/kdtree/kdtree_flann.h>

struct treeparams
{
	float x,y;
	float d;
	float h;
	float c;
};

struct cylinder
{
        bool ismodel;
        float x,y,z;
        float dx,dy,dz;
        float rad;
        float len;
        float steprad;
        float stepcov;
        float radratio;
	pcl::PointCloud<PointTreeseg>::Ptr cloud;
        pcl::PointCloud<PointTreeseg>::Ptr inliers;
};

struct cloudmetrics
{
	int count;
	Eigen::Vector4f min3D;
	Eigen::Vector4f max3D;
	Eigen::Vector4f centroid;
	Eigen::Matrix3f covariancematrix;
	Eigen::Matrix3f eigenvectors;
	Eigen::Vector3f eigenvalues;
	Eigen::Vector3f vector3D;
	float length;
};

struct basiccloudmetrics
{
	int count;
	Eigen::Vector4f min3D;
	Eigen::Vector4f max3D;
};

//File IO

std::vector<std::string> getFileID(std::string filename);
void readTiles(const std::vector<std::string> &args, pcl::PointCloud<PointTreeseg>::Ptr &cloud);
int getTilesStartIdx(const std::vector<std::string> &args);
void writeClouds(const std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clusters, std::string fname, bool doPCA);

//Nearest neighbour analysis

std::vector<float> dNN(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest);
std::vector<std::vector<float>> dNNz(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest, float zstep);
float interpolatedNNZ(float x, const std::vector<std::vector<float>> &nndata, bool extrapolate);

//Cloud metrics

void getCloudMetrics(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, cloudmetrics metrics);
float getCloudLength(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, const Eigen::Vector4f &centroid, const Eigen::Matrix3f &eigenvectors);
void getBasicCloudMetrics(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, basiccloudmetrics metrics);

//Downsampling

void downsample(const pcl::PointCloud<PointTreeseg>::Ptr &original, float edgelength, pcl::PointCloud<PointTreeseg>::Ptr &filtered, bool octree=true);
void thin(const pcl::PointCloud<PointTreeseg>::Ptr &original, float edgelength, pcl::PointCloud<PointTreeseg>::Ptr &filtered, bool preservePointClosestToVoxelCentroid=false);

//Spatial filters

void spatial1DFilter(const pcl::PointCloud<PointTreeseg>::Ptr &original, std::string dimension, float min, float max, pcl::PointCloud<PointTreeseg>::Ptr &filtered);
void spatial3DCylinderFilter(const pcl::PointCloud<PointTreeseg>::Ptr &original, cylinder cyl, pcl::PointCloud<PointTreeseg>::Ptr &filtered);

//Clustering

void euclideanClustering(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, float dmax, int nmin, std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clusters);

//Principal component analysis

void computePCA(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, Eigen::Vector4f &centroid, Eigen::Matrix3f &covariancematrix, Eigen::Matrix3f &eigenvectors, Eigen::Vector3f &eigenvalues);

//Surface normals

void estimateNormals(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest, pcl::PointCloud<pcl::Normal>::Ptr &normals);

//Segmentation

void regionSegmentation(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals, int nneighbours, int nmin, int nmax, float smoothness, float curvature, std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &regions);

//Shape fitting

void fitPlane(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals, float dthreshold, pcl::PointIndices::Ptr &inliers, float nweight=0, float angle=0, Eigen::Vector3f axis={0,0,1});
std::vector<float> fitCircle(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest);
void fitCylinder(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest, bool finite, bool diagnostics, cylinder &cyl);
void cylinderDiagnostics(cylinder &cyl, int nnearest);

//Generic

bool sortCloudByX(const PointTreeseg &p1, const PointTreeseg &p2);
bool sortCloudByY(const PointTreeseg &p1, const PointTreeseg &p2);
bool sortCloudByZ(const PointTreeseg &p1, const PointTreeseg &p2);
bool sortCloudByDescEuclidDist(const PointTreeseg &p1, const PointTreeseg &p2);
#if XYZRRDRS == true
bool sortCloudByRange(const PointTreeseg &p1, const PointTreeseg &p2);
bool sortCloudByReflectance(const PointTreeseg &p1, const PointTreeseg &p2);
bool sortCloudByDeviation(const PointTreeseg &p1, const PointTreeseg &p2);
bool sortCloudByReturnNumber(const PointTreeseg &p1, const PointTreeseg &p2);
bool sortCloudByScanNumber(const PointTreeseg &p1, const PointTreeseg &p2);
#endif
bool equalPoint(const PointTreeseg &p1, const PointTreeseg &p2);
bool sort2DFloatVectorByCol1(const std::vector<float> &v1, const std::vector<float> &v2);
bool sort2DFloatVectorByCol2(const std::vector<float> &v1, const std::vector<float> &v2);
int findClosestIdx(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, const std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clouds, bool biggest);
int findPrincipalCloudIdx(const std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clouds);
void extractIndices(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, const pcl::PointIndices::Ptr &inliers, bool invert, pcl::PointCloud<PointTreeseg>::Ptr &filtered);
float minDistBetweenClouds(const pcl::PointCloud<PointTreeseg>::Ptr &a, const pcl::PointCloud<PointTreeseg>::Ptr &b);
float minDistBetweenClouds(const pcl::PointCloud<PointTreeseg>::Ptr &a, const pcl::PointCloud<PointTreeseg>::Ptr &b, const pcl::KdTreeFLANN<PointTreeseg> &kdtree);
bool intersectionTest3DBox(const Eigen::Vector4f &amin, const Eigen::Vector4f &amax, const Eigen::Vector4f &bmin, const Eigen::Vector4f &bmax);
void catIntersectingClouds(std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clouds);
void removeDuplicatePoints(pcl::PointCloud<PointTreeseg>::Ptr &cloud);

//treeseg specific

std::vector<std::vector<float>> getDtmAndSlice(const pcl::PointCloud<PointTreeseg>::Ptr &plot, float resolution, float percentile, float zmin, float zmax, pcl::PointCloud<PointTreeseg>::Ptr &slice);
void correctStem(const pcl::PointCloud<PointTreeseg>::Ptr &stem, float nnearest, float zstart, float zstep, float stepcovmax, float radchangemin, pcl::PointCloud<PointTreeseg>::Ptr &corrected);
void removeFarRegions(float dmin, std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &regions);
void buildTree(const std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &regions, int cyclecount, int firstcount, float firstdistance, int nnearest, float seconddist, pcl::PointCloud<PointTreeseg>::Ptr &tree);
treeparams getTreeParams(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest, float zstep, float diffmax);
