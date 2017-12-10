//Andrew Burt - a.burt@ucl.ac.uk

#include <treeseg.hpp>

struct info
{
	float mean;
	float stddev;
};

info nearestNeighbour(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int NSearches)
{
	std::vector<float> dist;
	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);
	int K = NSearches + 1;
	for(pcl::PointCloud<pcl::PointXYZ>::iterator it=cloud->begin();it!=cloud->end();it++)
	{
		std::vector<float> p_dist;
		pcl::PointXYZ searchPoint;
		searchPoint.x = it->x;
		searchPoint.y = it->y;
		searchPoint.z = it->z;
		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);
		tree.nearestKSearch(searchPoint,K,pointIdxNKNSearch,pointNKNSquaredDistance);
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
	info a;
	a.mean = dist_mean;
	a.stddev = stddev;
	return a;
}

int main (int argc, char** argv)
{
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read(argv[3],*cloud);
	float delta_z = atof(argv[1]);
	int neighbours = atoi(argv[2]);
	Eigen::Vector4f min,max;
	pcl::getMinMax3D(*cloud,min,max);
	for(float z=min[2];z<max[2];z+=delta_z)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(z,z+delta_z);
		pass.filter(*tmp);
		if(tmp->points.size() > neighbours)
		{
			info d;
			d = nearestNeighbour(tmp,neighbours);
			float pos = z - min[2];
			std::cout << pos << " " << d.mean << " " << d.stddev << std::endl;
		}
	}
}
