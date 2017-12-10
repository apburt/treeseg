/*
* treeseg.h
*
* MIT License
*
* Copyright 2017 Andrew Burt - a.burt@ucl.ac.uk
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

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>

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
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr inliers;
};

struct treeinfo
{
	float x,y,z;
	float d;
	float h;
	float c;
};

std::vector<std::string> getFileID(char *fname);

std::vector<float> dNN(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int nnearest);
std::vector<std::vector<float>> dNNz(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int nnearest, float zstep);

float minDistBetweenClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr &a, pcl::PointCloud<pcl::PointXYZ>::Ptr &b);

void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &original, float edgelength, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered);

void extractIndices(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointIndices::Ptr &inliers, bool invert, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered);
int findClosestIdx(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds);

void spatial1DFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &original, std::string dimension, float min, float max, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered);
void spatial3DCylinderFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &original, cylinder cyl, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered);

std::vector<std::vector<float>> getDemAndSlice(pcl::PointCloud<pcl::PointXYZ>::Ptr &plot, float resolution, float zmin, float zmax, pcl::PointCloud<pcl::PointXYZ>::Ptr &slice);

void computePCA(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::Vector4f &centroid, Eigen::Matrix3f &covariancematrix, Eigen::Matrix3f &eigenvectors, Eigen::Vector3f &eigenvalues);
void estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int nnearest, pcl::PointCloud<pcl::Normal>::Ptr &normals);

void euclideanClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float dmax, int nmin, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters);
void regionSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int nnearest, int nmin, float smoothness, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &regions);

void writeClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters, std::string fname, bool doPCA);

std::vector<float> fitCircle(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int nnearest);
void fitCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int nnearest, bool finite, bool diagnostics, cylinder &cyl);
void cylinderDiagnostics(cylinder &cyl, int nnearest);
void fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int nnearest, pcl::PointIndices::Ptr &inliers);

bool intersectionTest3DBox(Eigen::Vector4f amin, Eigen::Vector4f amax, Eigen::Vector4f bmin, Eigen::Vector4f bmax);
void catIntersectingClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds);

void correctStem(pcl::PointCloud<pcl::PointXYZ>::Ptr &stem, float nnearest, float zstart, float zstep, float stepcovmax, float radchangemin, pcl::PointCloud<pcl::PointXYZ>::Ptr &corrected);

float getDBH(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int nnearest, float zstep, float diffmax);

bool sortCol(const std::vector<int>& v1, const std::vector<int>& v2);
int findPrincipalCloudIdx(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds);

float interpolatedNNZ(float x, std::vector<std::vector<float>> nndata, bool extrapolate);
void buildTree(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters, pcl::PointCloud<pcl::PointXYZ>::Ptr &tree);
