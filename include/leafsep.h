/*
* leafsep.h
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

#include "treeseg.h"

#include <armadillo>

void gmmByPoint(pcl::PointCloud<PointTreeseg>::Ptr &cloud, int knn, arma::uword n_gaus, int dist_mode, int seed_mode, int km_iter, int em_iter, arma::mat &featmatrix, arma::gmm_diag &model);
void gmmByCluster(std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clouds, arma::uword n_gaus, int dist_mode, int seed_mode, int km_iter, int em_iter, arma::mat &featmatrix, arma::gmm_full &model);

std::vector<int> classifyGmmPointModel(pcl::PointCloud<PointTreeseg>::Ptr &cloud, int n_gaus, arma::mat &featmatrix, arma::gmm_diag &model);
std::vector<int> classifyGmmClusterModel(std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clouds, int n_gaus, arma::mat &featmatrix, arma::gmm_full &model);

void separateCloudsClassifiedByPoint(pcl::PointCloud<PointTreeseg>::Ptr &cloud, std::vector<int> classifications, std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &classifiedclouds);
void separateCloudsClassifiedByCluster(std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clouds, std::vector<int> classifications, std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &classifiedclouds);

void writeCloudClassifiedByPoint(pcl::PointCloud<PointTreeseg>::Ptr &cloud, std::vector<int> &classifications, std::string fname);
void writeCloudClassifiedByCluster(std::vector<pcl::PointCloud<PointTreeseg>::Ptr> &clouds, std::vector<int> &classifications, std::string fname);
