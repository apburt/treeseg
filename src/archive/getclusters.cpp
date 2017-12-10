//Andrew Burt - a.burt@ucl.ac.uk

#include <treeseg.hpp>

struct cylParams
{
	bool ismodel;
	float x,y,z;
	float dx,dy,dz;
	float rad;
	float len;
	float ccount,icount;
	float steprad;
	float stepcov;
	float radratio;
	float ransac_dist;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

void xyfilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float x_min, float x_max, float y_min, float y_max, float extension)
{
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(x_min-extension,x_max+extension);
	pass.filter(*cloud);
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(y_min-extension,y_max+extension);
	pass.filter(*cloud);
}

float nearestNeighbour(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int NSearches)
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
	return dist_mean;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusterSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr &plot_cloud, int NSearch, int min_count)
{
	float dist = nearestNeighbour(plot_cloud,NSearch);
	std::vector<pcl::PointIndices> indices;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(dist);
	ec.setMinClusterSize(min_count);
	ec.setMaxClusterSize(std::numeric_limits<int>().max());
	ec.setSearchMethod(tree);
	ec.setInputCloud(plot_cloud);
	ec.extract(indices);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
	for(std::vector<pcl::PointIndices>::iterator it=indices.begin();it!=indices.end();it++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		for(std::vector<int>::iterator pit=it->indices.begin();pit!=it->indices.end();pit++)
		{
			cloud->insert(cloud->end(),plot_cloud->points[*pit]);
		}
		clusters.push_back(cloud);
	}
	return clusters;
}

void clusterCorrection(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters, int NSearch, int min_count, float smoothness)
{
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> corrected_clusters;
	for(int i=0;i<clusters.size();i++)
	{
//		KSearch results = nearestNeighbour(clusters[i],NSearch);
//		float dist = results.mean;
		std::vector<pcl::PointIndices> indices;
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
		pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
		ne.setSearchMethod(tree);
		ne.setInputCloud(clusters[i]);
//		ne.setRadiusSearch(dist);
		ne.setKSearch(NSearch);
		ne.compute(*normals);
		pcl::RegionGrowing<pcl::PointXYZ,pcl::Normal> reg;
		reg.setMinClusterSize(min_count);
		reg.setMaxClusterSize(1000000);
		reg.setSearchMethod(tree);
		reg.setNumberOfNeighbours(30); //this is a sensitive value - keep fixed and vary smoothness and curvature
		reg.setInputCloud(clusters[i]);
		reg.setInputNormals(normals);
		reg.setSmoothnessThreshold(smoothness/ 180 * M_PI);
		reg.extract(indices);
		for(std::vector<pcl::PointIndices>::iterator it=indices.begin();it!=indices.end();it++)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			for(std::vector<int>::iterator pit=it->indices.begin();pit!=it->indices.end();pit++)
			{
				cloud->insert(cloud->end(),clusters[i]->points[*pit]);
			}
			corrected_clusters.push_back(cloud);
		}
	}
	clusters.clear();
	clusters = corrected_clusters;
}

void fitCylinder(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters, int NSearch, float x_min, float x_max, float y_min, float y_max, float diameter_min, float diameter_max, float length_min, float stepcov_max, float radratio_min)
{
	std::vector<int> remove;
	for(int i=0;i<clusters.size();i++)
	{
		cylParams cylinder;
		float dist = nearestNeighbour(clusters[i],NSearch);
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
		pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
		ne.setSearchMethod(tree);
		ne.setInputCloud(clusters[i]);
//		ne.setRadiusSearch(dist);
		ne.setKSearch(NSearch);
		ne.compute(*normals);
		cylParams tmp_cylinder;
		pcl::PointIndices indices;
		pcl::ModelCoefficients coeff;
		pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_CYLINDER);
		seg.setNormalDistanceWeight(0.1);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(1000000);
		seg.setDistanceThreshold(dist);
		seg.setInputCloud(clusters[i]);
		seg.setInputNormals(normals);
		seg.segment(indices,coeff);
		pcl::PointCloud<pcl::PointXYZ>::Ptr inliers(new pcl::PointCloud<pcl::PointXYZ>);
		if(indices.indices.size() != 0)
		{
			//get x,y,z,dx,dy,dz,rad,len,indices
			pcl::PointCloud<pcl::PointXYZ>::Ptr inliers_transformed(new pcl::PointCloud<pcl::PointXYZ>);		
			for(std::vector<int>::iterator pit=indices.indices.begin();pit!=indices.indices.end();pit++)
			{
				inliers->insert(inliers->end(),clusters[i]->points[*pit]);
			}
			Eigen::Vector3f point(coeff.values[0],coeff.values[1],coeff.values[2]);
			Eigen::Vector3f direction(coeff.values[3],coeff.values[4],coeff.values[5]); 
			Eigen::Affine3f transform,inverse_transform;
			Eigen::Vector3f world(0,direction[2],-direction[1]); 
			direction.normalize(); 
			pcl::getTransformationFromTwoUnitVectorsAndOrigin(world,direction,point,transform); 
			pcl::transformPointCloud(*inliers,*inliers_transformed,transform); 
			Eigen::Vector4f min,max,centroid;
			pcl::getMinMax3D(*inliers_transformed,min,max);
			pcl::compute3DCentroid(*inliers_transformed,centroid);
			pcl::PointXYZ center_w,center;
			center_w.x = centroid[0];
			center_w.y = centroid[1];
			center_w.z = centroid[2];
			inverse_transform = transform.inverse();
			center = pcl::transformPoint(center_w,inverse_transform);
			cylinder.ismodel = true;
			cylinder.x = center.x;
			cylinder.y = center.y;
			cylinder.z = center.z;
			cylinder.dx = coeff.values[3];
			cylinder.dy = coeff.values[4];
			cylinder.dz = coeff.values[5];
			cylinder.rad = coeff.values[6];
			cylinder.len = max[2] - min[2];
			cylinder.ccount = clusters[i]->points.size();
			cylinder.icount = inliers->points.size();
			cylinder.ransac_dist = dist;
			cylinder.cloud = inliers;
			//get radcov
			int RADCOVSTEPS = 6;
			float delta_z = (max[2]-min[2])/RADCOVSTEPS;
			std::vector<float> rad;
			for(int j=0;j<RADCOVSTEPS;j++)
			{
				float z_min = min[2] + j * delta_z;
				float z_max = min[2] + (j+1) * delta_z;
				pcl::PointCloud<pcl::PointXYZ>::Ptr z_cloud(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::PassThrough<pcl::PointXYZ> passz;
				passz.setInputCloud(inliers_transformed);
				passz.setFilterFieldName("z");
				passz.setFilterLimits(z_min,z_max);
				passz.filter(*z_cloud);
				if(z_cloud->points.size() > 1)
				{
					pcl::PointCloud<pcl::Normal>::Ptr z_normals(new pcl::PointCloud<pcl::Normal>);
					pcl::search::KdTree<pcl::PointXYZ>::Ptr z_tree (new pcl::search::KdTree<pcl::PointXYZ> ());
					pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> z_ne;
					z_ne.setSearchMethod(z_tree);
					z_ne.setInputCloud(z_cloud);
					//z_ne.setRadiusSearch(dist);
					z_ne.setKSearch(NSearch);
					z_ne.compute(*z_normals);
					pcl::PointIndices z_indices;
					pcl::ModelCoefficients z_coeff;
					pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> z_seg;
					z_seg.setOptimizeCoefficients(true);
					z_seg.setModelType(pcl::SACMODEL_CYLINDER);
					z_seg.setNormalDistanceWeight(0.1);
					z_seg.setMethodType(pcl::SAC_RANSAC);
					z_seg.setMaxIterations(1000000);
					z_seg.setDistanceThreshold(dist);
					z_seg.setInputCloud(z_cloud);
					z_seg.setInputNormals(z_normals);
					z_seg.segment(z_indices,z_coeff);
					if(z_indices.indices.size() != 0)
					{
						if(z_coeff.values[6] > 0)
						{
							rad.push_back(z_coeff.values[6]);
						}
					}
				}
			}
			if(rad.size() >= RADCOVSTEPS - 2)
			{
				float sum = std::accumulate(rad.begin(),rad.end(),0.0);
				float mean = sum/rad.size();
				std::vector<float> diff(rad.size());
				std::transform(rad.begin(),rad.end(),diff.begin(),std::bind2nd(std::minus<float>(),mean));
				float stdev = std::sqrt(std::inner_product(diff.begin(),diff.end(),diff.begin(),0.0) / rad.size());
				float cov = stdev / mean;
				cylinder.steprad = mean; 
				cylinder.stepcov = cov;
				cylinder.radratio = std::min(cylinder.rad,cylinder.steprad)/std::max(cylinder.rad,cylinder.steprad);
			}
			else cylinder.ismodel = false;
		}
		else cylinder.ismodel = false;
		//perform checks on cylinder model
		bool isStem = false;
		if(cylinder.ismodel == true)
		{
			if(cylinder.x >= x_min && cylinder.x <= x_max)
			{
				if(cylinder.y >= y_min && cylinder.y <= y_max)
				{
					if(cylinder.rad*2 >= diameter_min && cylinder.rad*2 <= diameter_max)
					{
						if(cylinder.len >= length_min)
						{
							if(cylinder.stepcov <= stepcov_max)
							{
								if(cylinder.radratio >= radratio_min)
								{
									*clusters[i] = *inliers;
									isStem = true;
								}
							}
						}
					}
				}
			}
		}
		if(isStem == false) remove.push_back(i);
	}
	std::sort(remove.begin(),remove.end(),std::greater<int>());
	for(int j=0;j<remove.size();j++) clusters.erase(clusters.begin()+remove[j]);
}

void clusterPrinciple(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters, float max_cluster_ground_angle)
{
	std::vector<int> remove;
	for(int i=0;i<clusters.size();i++)
	{
		Eigen::Vector4f min,max;
		Eigen::Vector4f centroid;
		Eigen::Matrix3f covariancematrix;
		Eigen::Matrix3f eigenvectors;
		Eigen::Vector3f eigenvalues;
		pcl::getMinMax3D(*clusters[i],min,max);
		pcl::compute3DCentroid(*clusters[i],centroid);
		pcl::computeCovarianceMatrix(*clusters[i],centroid,covariancematrix);
		pcl::eigen33(covariancematrix,eigenvectors,eigenvalues);
		Eigen::Vector4f ground_vector(eigenvectors(0,2),eigenvectors(1,2),0,0);
		Eigen::Vector4f cluster_vector(eigenvectors(0,2),eigenvectors(1,2),eigenvectors(2,2),0);
		float cluster_ground_angle = pcl::getAngle3D(ground_vector,cluster_vector) * (180/M_PI);
		if(cluster_ground_angle < (90 - max_cluster_ground_angle) || cluster_ground_angle > (90 + max_cluster_ground_angle)) remove.push_back(i);
	}
	std::sort(remove.begin(),remove.end(),std::greater<int>());
	for(int j=0;j<remove.size();j++) clusters.erase(clusters.begin()+remove[j]);
}

void clusterConcatenate(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters, float bounding_box_expansion)
{
	//this whole function is slow and messy - primary bottleneck of getclusters
	bool done_something = true;
	while(done_something == true)
	{
		int inner_idx;
		int outer_idx;
		bool found_duplicate = false;
		for(int outer=0;outer<clusters.size();outer++)
		{
			Eigen::Vector4f min_outer,max_outer;
			pcl::getMinMax3D(*clusters[outer],min_outer,max_outer);
			float xmin_outer = min_outer[0];
			float xmax_outer = max_outer[0];
			float ymin_outer = min_outer[1];
			float ymax_outer = max_outer[1];
			float zmin_outer = min_outer[2];
			float zmax_outer = max_outer[2];
			for(int inner=0;inner<clusters.size();inner++)
			{
				if(outer != inner)
				{
					Eigen::Vector4f min_inner,max_inner;
					pcl::getMinMax3D(*clusters[inner],min_inner,max_inner);
					float xmin_inner = min_inner[0]-((max_inner[0]-min_inner[0]))*bounding_box_expansion;
					float xmax_inner = max_inner[0]+((max_inner[0]-min_inner[0]))*bounding_box_expansion;
					float ymin_inner = min_inner[1]-((max_inner[1]-min_inner[1]))*bounding_box_expansion;
					float ymax_inner = max_inner[1]+((max_inner[1]-min_inner[1]))*bounding_box_expansion;
					float zmin_inner = min_inner[2]-((max_inner[2]-min_inner[2]))*bounding_box_expansion;
					float zmax_inner = max_inner[2]+((max_inner[2]-min_inner[2]))*bounding_box_expansion;
					if(xmin_inner < xmax_outer && xmax_inner > xmin_outer)
					{
						if(ymin_inner < ymax_outer && ymax_inner > ymin_outer)
						{
							if(zmin_inner < zmax_outer && zmax_inner > zmin_outer)
							{
								outer_idx = outer;
								inner_idx = inner;
								found_duplicate = true;
								break;	
							}
						}
					}
				}
			}
			if(found_duplicate == true) break;
		}
		if(found_duplicate == true)
		{
			*clusters[outer_idx] += *clusters[inner_idx];
			clusters.erase(clusters.begin()+inner_idx);
		}
		else done_something = false;
	}
}

void writeClusters(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters, std::string fname, bool individual)
{
	pcl::PCDWriter writer;
	if(individual == false)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		for(int i=0;i<clusters.size();i++)
		{
			int r = rand() % 256;
			int g = rand() % 256;
			int b = rand() % 256;
			for(int j=0;j<clusters[i]->points.size();j++)
			{
				pcl::PointXYZRGB point;
				point.x = clusters[i]->points[j].x;
				point.y = clusters[i]->points[j].y;
				point.z = clusters[i]->points[j].z;
				point.r = r;
				point.g = g;
				point.b = b;
				out_cloud->insert(out_cloud->end(),point);
			}
		}
		writer.write(fname,*out_cloud,true);
	}
	if(individual == true)
	{
		int count = 0;
		for(int i=0;i<clusters.size();i++)
		{
			std::stringstream ss;
			ss << "cluster_" << count << ".pcd";
			writer.write(ss.str(),*clusters[i],true);
			count++;
		}
	}
}

int main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr plot_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	reader.read(argv[5],*plot_cloud);
	std::stringstream ss;
	//
	std::cout << "Spatial filtering: " << std::flush;
	std::ifstream cfile;
	cfile.open(argv[1]);
	float coordfile[4];
	int no_count = 0;
	if(cfile.is_open())
	{
		while(!cfile.eof())
		{
			cfile >> coordfile[no_count];
			no_count++;
		}
	}
	cfile.close();
	float x_min = coordfile[0];
	float x_max = coordfile[1];
	float y_min = coordfile[2];
	float y_max = coordfile[3];
	xyfilter(plot_cloud,x_min,x_max,y_min,y_max,2.5);
	std::cout << "complete" << std::endl;
	//
	std::cout << "Cluster extraction: " << std::flush;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
	int NSearch = 9; //1.5x voxel neighbours
	int min_count = 100;
	clusters = clusterSegmentation(plot_cloud,NSearch,min_count);
	ss.str("");
	ss << "slice_clusters.pcd";
	writeClusters(clusters,ss.str(),false);
	std::cout << ss.str() << " | " << clusters.size() << std::endl;
	//
	std::cout << "Cluster correction: " << std::flush;
	NSearch = 9; //1.5x voxel neighbours
	float smoothness = atof(argv[2]);
	clusterCorrection(clusters,NSearch,min_count,smoothness);
	ss.str("");
	ss << "slice_clusters_corrected.pcd";
	writeClusters(clusters,ss.str(),false);
	std::cout << ss.str() << " | " << clusters.size() << std::endl;
	//
	std::cout << "Cluster RANSAC cylinder fit: " << std::flush;
	NSearch = 60; //10x voxel neighbours
	float diameter_min = atof(argv[3]);
	float diameter_max = atof(argv[4]);
	float length_min = 2.25; //assuming 3m slice!!!
	float stepcov_max = 0.2;
	float radratio_min = 0.8;
	fitCylinder(clusters,NSearch,x_min,x_max,y_min,y_max,diameter_min,diameter_max,length_min,stepcov_max,radratio_min);
	ss.str("");
	ss << "slice_clusters_corrected_fitted.pcd";
	writeClusters(clusters,ss.str(),false);
	std::cout << ss.str() << " | " << clusters.size() << std::endl;
	//
	std::cout << "Principle component trimming: " << std::flush;
	float max_angle = 35; //assuming more than 35 degrees off horizontal is not a stem 
	clusterPrinciple(clusters,max_angle);
	ss.str("");
	ss << "slice_clusters_corrected_fitted_principle.pcd";
	writeClusters(clusters,ss.str(),false);
	std::cout << ss.str() << " | " << clusters.size() << std::endl;
	//
	std::cout << "Concatenating clusters: " << std::flush;
	float bounding_box_expansion = 0;
	clusterConcatenate(clusters,bounding_box_expansion);
	ss.str("");
	ss << "slice_clusters_corrected_fitted_principle_concatenated.pcd";
	writeClusters(clusters,ss.str(),false);
	std::cout << ss.str() << " | " << clusters.size() << std::endl;
	//
	writeClusters(clusters,"",true);
	return 0;
}
