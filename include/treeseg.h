/*
* treeseg.h
*
* MIT License
*
* Copyright 2020 Andrew Burt - a.burt@ucl.ac.uk
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

std::vector<std::string> getFileID(char *fname);

std::vector<float> dNN(pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest);
std::vector<std::vector<float>> dNNz(pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest, float zstep);

float minDistBetweenClouds(pcl::PointCloud<PointTreeseg>::Ptr &a, pcl::PointCloud<PointTreeseg>::Ptr &b);

void downsample(pcl::PointCloud<PointTreeseg>::Ptr &original, float edgelength, pcl::PointCloud<PointTreeseg>::Ptr &filtered);

void extractIndices(pcl::PointCloud<PointTreeseg>::Ptr &cloud, pcl::PointIndices::Ptr &inliers, bool invert, pcl::PointCloud<PointTreeseg>::Ptr filtered);
int findClosestIdx(pcl::PointCloud<PointTreeseg>::Ptr &cloud, std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clouds, bool biggest);

void spatial1DFilter(pcl::PointCloud<PointTreeseg>::Ptr &original, std::string dimension, float min, float max, pcl::PointCloud<PointTreeseg>::Ptr &filtered);
void spatial3DCylinderFilter(pcl::PointCloud<PointTreeseg>::Ptr &original, cylinder cyl, pcl::PointCloud<PointTreeseg>::Ptr &filtered);

std::vector<std::vector<float>> getDemAndSlice(pcl::PointCloud<PointTreeseg>::Ptr &plot, float resolution, float zmin, float zmax, pcl::PointCloud<PointTreeseg>::Ptr &slice);

void computePCA(pcl::PointCloud<PointTreeseg>::Ptr &cloud, Eigen::Vector4f &centroid, Eigen::Matrix3f &covariancematrix, Eigen::Matrix3f &eigenvectors, Eigen::Vector3f &eigenvalues);
void estimateNormals(pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest, pcl::PointCloud<pcl::Normal>::Ptr &normals);

void euclideanClustering(pcl::PointCloud<PointTreeseg>::Ptr &cloud, float dmax, int nmin, std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clusters);
void regionSegmentation(pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest, int nmin, float smoothness, std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &regions);

void writeClouds(std::vector<pcl::PointCloud<PointTreeseg>::Ptr> clusters, std::string fname, bool doPCA);

std::vector<float> fitCircle(pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest);
void fitCylinder(pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest, bool finite, bool diagnostics, cylinder &cyl);
void cylinderDiagnostics(cylinder &cyl, int nnearest);
void fitPlane(pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest, pcl::PointIndices::Ptr &inliers);

bool intersectionTest3DBox(Eigen::Vector4f amin, Eigen::Vector4f amax, Eigen::Vector4f bmin, Eigen::Vector4f bmax);
void catIntersectingClouds(std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clouds);

void correctStem(pcl::PointCloud<PointTreeseg>::Ptr &stem, float nnearest, float zstart, float zstep, float stepcovmax, float radchangemin, pcl::PointCloud<PointTreeseg>::Ptr &corrected);

treeparams getTreeParams(pcl::PointCloud<PointTreeseg>::Ptr &cloud, int nnearest, float zstep, float diffmax);

bool sortCol(const std::vector<int>& v1, const std::vector<int>& v2);
int findPrincipalCloudIdx(std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clouds);

float interpolatedNNZ(float x, std::vector<std::vector<float>> nndata, bool extrapolate);
void removeFarRegions(std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clusters);
void buildTree(std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clusters, pcl::PointCloud<PointTreeseg>::Ptr &tree);
