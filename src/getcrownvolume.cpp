#include "treeseg.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

float maxheight(float dbh)
{
	//m    -> 41.22 * dbh ^ 0.3406
	//ci_u -> 42.30 * dbh ^ 0.3697
	float height = 42.30 * pow(dbh,0.3697) + 50;
	return height;
}

float maxcrown(float dbh)
{
	//m    -> 29.40 * dbh ^ 0.6524
	//ci_u -> 30.36 * dbh ^ 0.6931
	float extent = 30.36 * pow(dbh,0.6931) + 30; 
	return extent;
}

int main(int argc, char **argv)
{
	std::vector<std::string> args(argv+1,argv+argc);
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	std::cout << "Reading plot-level cloud: " << std::flush;
	pcl::PointCloud<PointTreeseg>::Ptr plot(new pcl::PointCloud<PointTreeseg>);
	readTiles(args,plot);
	std::cout << "complete" << std::endl;
	//
	for(int i=1;i<getTilesStartIdx(args);i++)
	{
		std::cout << "---------------" << std::endl;
		//
		std::vector<std::string> id = getFileID(args[i]);
		pcl::PointCloud<PointTreeseg>::Ptr stem(new pcl::PointCloud<PointTreeseg>);
		reader.read(args[i],*stem);
		//
		std::cout << "Estimating DBH: " << std::flush;
		int nnearest = 90;
		float zstep = 0.75;
		float diffmax = 0.1;
		treeparams params = getTreeParams(stem,nnearest,zstep,diffmax);
		std::cout << params.d << std::endl;
		//
		std::cout << "Volume dimensions: " << std::flush;
		float h = maxheight(params.d);
		float r = maxcrown(params.d)/2;
		std::cout << h << "m x " << r << "m (height + radius)" << std::endl;
		//
		std::cout << "Segmenting volume: " << std::flush;
		pcl::PointCloud<PointTreeseg>::Ptr volume(new pcl::PointCloud<PointTreeseg>);
		pcl::PointCloud<PointTreeseg>::Ptr tmp1(new pcl::PointCloud<PointTreeseg>);
		pcl::PointCloud<PointTreeseg>::Ptr tmp2(new pcl::PointCloud<PointTreeseg>);
		Eigen::Vector4f min,max,centroid;
		cylinder cyl;
		pcl::getMinMax3D(*stem,min,max);
		pcl::compute3DCentroid(*stem,centroid);
		cyl.x = centroid[0];
		cyl.y = centroid[1];
		cyl.z = centroid[2];
		cyl.rad = r;
		cyl.dx = 0;
		cyl.dy = 0;
		cyl.dz = 1;
		spatial3DCylinderFilter(plot,cyl,tmp1);
		float stempos = std::stof(args[0]);
		spatial1DFilter(tmp1,"z",max[2]*stempos,min[2]+h,tmp2);
		*volume += *stem;
		*volume += *tmp2;
		std::stringstream ss;
		ss << id[0] << ".v." << id[1] << ".pcd";
		writer.write(ss.str(),*volume,true);
		std::cout << ss.str() << std::endl;
		//
		std::cout << "Removing duplicate points: " << std::flush;
		removeDuplicatePoints(volume);
		ss.str("");
		ss << id[0] << ".volume." << id[1] << ".pcd";
		writer.write(ss.str(),*volume,true);
		std::cout << ss.str() << std::endl;
	}
	return 0;
}
