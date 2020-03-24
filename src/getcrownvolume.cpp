//Andrew Burt - a.burt@ucl.ac.uk

#include "treeseg.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

float maxheight(float dbh)
{
	//m    -> 41.22 * dbh ^ 0.3406
	//ci_u -> 42.30 * dbh ^ 0.3697
	float height = 42.30 * pow(dbh,0.3697) + 5;
	return height;
}

float maxcrown(float dbh)
{
	//m    -> 29.40 * dbh ^ 0.6524
	//ci_u -> 30.36 * dbh ^ 0.6931
	float extent = 30.36 * pow(dbh,0.6931) + 5; 
	return extent;
}

int main (int argc, char** argv)
{
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	std::cout << "Reading plot-level cloud: " << std::flush;
	pcl::PointCloud<PointTreeseg>::Ptr plot(new pcl::PointCloud<PointTreeseg>);
	reader.read(argv[1],*plot);
	std::cout << "complete" << std::endl;
	for(int i=2;i<argc;i++)
	{
		std::cout << "---------------" << std::endl;
		//
		std::vector<std::string> id = getFileID(argv[i]);
		pcl::PointCloud<PointTreeseg>::Ptr stem(new pcl::PointCloud<PointTreeseg>);
		reader.read(argv[i],*stem);
		//
		std::cout << "Estimating DBH: " << std::flush;
		int nnearest = 90;
		float zstep = 0.75;
		float diffmax = 0.1;
		treeparams params = getTreeParams(stem,nnearest,zstep,diffmax);
		std::cout << params.d << std::endl;
		//
		std::cout << "Crown dimensions: " << std::flush;
		float h = maxheight(params.d);
		float c = maxcrown(params.d);
		std::cout << h << "m x " << c << "m (HxW)" << std::endl;
		//
		std::cout << "Segmenting volume: " << std::flush;
		pcl::PointCloud<PointTreeseg>::Ptr xslice(new pcl::PointCloud<PointTreeseg>);
		pcl::PointCloud<PointTreeseg>::Ptr yslice(new pcl::PointCloud<PointTreeseg>);
		pcl::PointCloud<PointTreeseg>::Ptr zslice(new pcl::PointCloud<PointTreeseg>);
		pcl::PointCloud<PointTreeseg>::Ptr volume(new pcl::PointCloud<PointTreeseg>);
		Eigen::Vector4f min,max,centroid;
		pcl::getMinMax3D(*stem,min,max);
		pcl::compute3DCentroid(*stem,centroid);
		spatial1DFilter(plot,"x",centroid[0]-c/2,centroid[0]+c/2,xslice);
		spatial1DFilter(xslice,"y",centroid[1]-c/2,centroid[1]+c/2,yslice);
		spatial1DFilter(yslice,"z",max[2],min[2]+h,zslice);
		*volume += *stem;
		*volume += *zslice;
		std::stringstream ss;
		ss << "volume_" << id[0] << ".pcd";
		writer.write(ss.str(),*volume,true);	
		std::cout << ss.str() << std::endl;
	}
	return 0;
}
