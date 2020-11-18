/*
* treeseg.h
*
* MIT License
*
* Copyright 2017-2020 Andrew Burt - a.burt@ucl.ac.uk
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

//File IO

std::vector<std::string> getFileID(std::string filename);
void readTiles(const std::vector<std::string> &args, pcl::PointCloud<PointTreeseg>::Ptr &cloud);
int getTilesStartIdx(const std::vector<std::string> &args);
void writeClouds(const std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clusters, std::string fname, bool doPCA);

//Nearest neighbour analysis

std::vector<float> dNN(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest);
std::vector<std::vector<float>> dNNz(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest, float zstep);
float interpolatedNNZ(float x, const std::vector<std::vector<float>> &nndata, bool extrapolate);

//Downsampling

void downsample(const pcl::PointCloud<PointTreeseg>::Ptr &original, float edgelength, pcl::PointCloud<PointTreeseg>::Ptr &filtered);

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

void regionSegmentation(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest, int nmin, float smoothness, std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &regions);

//Shape fitting

void fitPlane(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest, pcl::PointIndices::Ptr &inliers);
std::vector<float> fitCircle(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest);
void fitCylinder(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest, bool finite, bool diagnostics, cylinder &cyl);
void cylinderDiagnostics(cylinder &cyl, int nnearest);

//Generic

bool sortCol(const std::vector<int> &v1, const std::vector<int> &v2);
bool sortCloudByZ(const PointTreeseg &p1, const PointTreeseg &p2);
std::vector<int> nearestIdx(const pcl::PointCloud<PointTreeseg>::Ptr &searchpoints, const pcl::PointCloud<PointTreeseg>::Ptr &cloud);
int findClosestIdx(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, const std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clouds, bool biggest);
int findPrincipalCloudIdx(const std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clouds);
void extractIndices(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, const pcl::PointIndices::Ptr &inliers, bool invert, pcl::PointCloud<PointTreeseg>::Ptr &filtered);
float minDistBetweenClouds(const pcl::PointCloud<PointTreeseg>::Ptr &a, const pcl::PointCloud<PointTreeseg>::Ptr &b);
float minDistBetweenClouds(const pcl::PointCloud<PointTreeseg>::Ptr &a, const pcl::PointCloud<PointTreeseg>::Ptr &b, const pcl::KdTreeFLANN<PointTreeseg> &kdtree);
bool intersectionTest3DBox(const Eigen::Vector4f &amin, const Eigen::Vector4f &amax, const Eigen::Vector4f &bmin, const Eigen::Vector4f &bmax);
void catIntersectingClouds(std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clouds);

//treeseg specific

std::vector<std::vector<float>> getDemAndSlice(const pcl::PointCloud<PointTreeseg>::Ptr &plot, float resolution, float percentile, float zmin, float zmax, pcl::PointCloud<PointTreeseg>::Ptr &slice);
void correctStem(const pcl::PointCloud<PointTreeseg>::Ptr &stem, float nnearest, float zstart, float zstep, float stepcovmax, float radchangemin, pcl::PointCloud<PointTreeseg>::Ptr &corrected);
void removeFarRegions(std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clusters);
void buildTree(const std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clusters, pcl::PointCloud<PointTreeseg>::Ptr &tree);
treeparams getTreeParams(const pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest, float zstep, float diffmax);
