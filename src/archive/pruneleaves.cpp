//Andrew Burt - a.burt@ucl.ac.uk

#include <treeseg.hpp>

float maxdist(float height)
{
//	float p1 = 0.0000006244;
//	float p2 = -0.00003327;
//	float p3 = 0.0005605;
//	float p4 = -0.0003874;
//	float p5 = 0.04986;
//	float distance = p1 * pow(height,4) + p2 * pow(height,3) + p3 * pow(height,2) + p4 * height + p5;
	float distance = std::max(0.000253 * pow(height,2.242) + 0.02666,0.055);
	return distance;
}

float maxheight(float dbh)
{
	float p1 = 23.61;
	float p2 = 50;
	float height = p1 * dbh + p2;
	return height;
}

float maxcrown(float dbh)
{
	float p1 = 28.71;
	float p2 = 4.886;
	float extent = p1 * dbh + p2 + 5;
	return extent;
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

cylParams fitCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,int NSearch)
{
	cylParams cylinder;
	float dist = nearestNeighbour(cloud,NSearch);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud);
//	ne.setRadiusSearch(dist);
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
	seg.setInputCloud(cloud);
	seg.setInputNormals(normals);
	seg.segment(indices,coeff);
	if(indices.indices.size() != 0)
	{
		//get x,y,z,dx,dy,dz,rad,len,indices
		pcl::PointCloud<pcl::PointXYZ>::Ptr inliers(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr inliers_transformed(new pcl::PointCloud<pcl::PointXYZ>);		
		for(std::vector<int>::iterator pit=indices.indices.begin();pit!=indices.indices.end();pit++)
		{
			inliers->insert(inliers->end(),cloud->points[*pit]);
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
		cylinder.ismodel = true;
		cylinder.x = center.x;
		cylinder.y = center.y;
		cylinder.z = center.z;
		cylinder.dx = coeff.values[3];
		cylinder.dy = coeff.values[4];
		cylinder.dz = coeff.values[5];
		cylinder.rad = coeff.values[6];
		cylinder.len = max[2] - min[2];
		cylinder.ccount = cloud->points.size();
		cylinder.icount = inliers->points.size();
		cylinder.ransac_dist = dist;
		cylinder.cloud = inliers;
		//get radcov
		int RADCOVSTEPS = 5;
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
			//	z_ne.setRadiusSearch(dist);
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
		if(rad.size() >= RADCOVSTEPS-2) //i.e. wouldnt expect more than two steps to fail + needs to be bigger than 3
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
	return cylinder;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> getslices(pcl::PointCloud<pcl::PointXYZ>::Ptr tree, float step)
{
	Eigen::Vector4f min,max;
	pcl::getMinMax3D(*tree,min,max);
	pcl::PassThrough<pcl::PointXYZ> pass;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
	for(float z=min[2];z<max[2];z+=step)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
		pass.setInputCloud(tree);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(z,z+step);
		pass.filter(*tmp);
		if(tmp->points.size() != 0) clusters.push_back(tmp);
	}	
	return clusters;
}

void segmentclusters(int NSearch, int min_count, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters)
{
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> tmp;
	for(int i=0;i<clusters.size();i++)
	{
		float dist = nearestNeighbour(clusters[i],NSearch);
		std::vector<pcl::PointIndices> indices;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance(dist);
		ec.setMinClusterSize(min_count);
		ec.setMaxClusterSize(std::numeric_limits<int>().max());
		ec.setSearchMethod(tree);
		ec.setInputCloud(clusters[i]);
		ec.extract(indices);
		for(std::vector<pcl::PointIndices>::iterator it=indices.begin();it!=indices.end();it++)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
			for(std::vector<int>::iterator pit=it->indices.begin();pit!=it->indices.end();pit++)
			{
				cluster->insert(cluster->end(),clusters[i]->points[*pit]);
			}
			tmp.push_back(cluster);
		}
	}
	clusters.clear();
	for(int j=0;j<tmp.size();j++) clusters.push_back(tmp[j]);
}

void correctclusters(int NSearch, int min_count, float smoothness, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters)
{
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> tmp;
	for(int i=0;i<clusters.size();i++)
	{
		float dist = nearestNeighbour(clusters[i],NSearch);
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
//		reg.setCurvatureThreshold(curvature);
		reg.extract(indices);
		for(std::vector<pcl::PointIndices>::iterator it=indices.begin();it!=indices.end();it++)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
			for(std::vector<int>::iterator pit=it->indices.begin();pit!=it->indices.end();pit++)
			{
				cluster->insert(cluster->end(),clusters[i]->points[*pit]);
			}
			tmp.push_back(cluster);
		}
	}
	clusters.clear();
	for(int j=0;j<tmp.size();j++) clusters.push_back(tmp[j]);
}

struct findices
{
	float sphericity;
	float eigenratio_a;
	float eigenratio_b;
};

findices getformindices(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	Eigen::Vector4f min,max;
	Eigen::Vector4f centroid;
	Eigen::Matrix3f covariancematrix;
	Eigen::Matrix3f eigenvectors;
	Eigen::Vector3f eigenvalues;
	pcl::getMinMax3D(*cloud,min,max);
	pcl::compute3DCentroid(*cloud,centroid);
	pcl::computeCovarianceMatrix(*cloud,centroid,covariancematrix);
	pcl::eigen33(covariancematrix,eigenvectors,eigenvalues);
	float sphericity = cbrt(pow(eigenvalues(0),2) / (eigenvalues(2)*eigenvalues(1)));
	float eigenratio_a = eigenvalues(0) / eigenvalues(2);
	float eigenratio_b = (eigenvalues(2) - eigenvalues(1)) / (eigenvalues(2) - eigenvalues(0));
	findices results;
	results.sphericity = sphericity;
	results.eigenratio_a = eigenratio_a;
	results.eigenratio_b = eigenratio_b;
	return results;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pruneclusters(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters, float sphericity_max, float eigenratio_a_max, float eigenratio_b_min)
{
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> leaf_clusters;
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
		float sphericity = cbrt(pow(eigenvalues[0],2)/(eigenvalues[2]*eigenvalues[1]));
		float eigenratio_a = eigenvalues[0] / eigenvalues[2];
		float eigenratio_b = (eigenvalues[2] - eigenvalues[1]) / (eigenvalues[2] - eigenvalues[0]);

		if(clusters[i]->points.size() < 1000)
		{


			if(sphericity > sphericity_max) remove.push_back(i);
			else if(eigenratio_a > eigenratio_a_max) remove.push_back(i);
			else if(eigenratio_b < eigenratio_b_min) remove.push_back(i);

		}

	}
	std::sort(remove.begin(),remove.end(),std::greater<int>());
	for(int j=0;j<remove.size();j++)
	{
		leaf_clusters.insert(leaf_clusters.end(),clusters[remove[j]]);
		clusters.erase(clusters.begin()+remove[j]);
	}
	return leaf_clusters;
}

float getdistance(pcl::PointCloud<pcl::PointXYZ>::Ptr a, pcl::PointCloud<pcl::PointXYZ>::Ptr b)
{
	float distance = std::numeric_limits<float>::max();
	for(int i=0;i<a->points.size();i++)
	{
		for(int j=0;j<b->points.size();j++)
		{
			float d = pcl::euclideanDistance(a->points[i],b->points[j]);
			if(d < distance) distance = d;
		}
	}
	return distance;
}

void buildtree(pcl::PointCloud<pcl::PointXYZ>::Ptr stem, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters, pcl::PointCloud<pcl::PointXYZ>::Ptr &t)
{
	Eigen::Vector4f gmin,gmax;
	pcl::getMinMax3D(*stem,gmin,gmax);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> tree;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> outer;
	tree.push_back(stem);
	outer.push_back(stem);
	bool done_something = true;
	int count = 0;
	while(done_something == true)
	{
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> tmp;
		for(int i=0;i<outer.size();i++)
		{
			Eigen::Vector4f min,max;
			Eigen::Vector4f o_centroid;
			pcl::compute3DCentroid(*outer[i],o_centroid);
			pcl::getMinMax3D(*outer[i],min,max);
			float height = max[2] - gmin[2];
			float distance_max = maxdist(height); 
			std::vector<int> tree_member;
			for(int j=0;j<clusters.size();j++)
			{
				Eigen::Vector4f i_centroid;
				pcl::compute3DCentroid(*clusters[j],i_centroid);
				float centroid_distance = pcl::distances::l2(o_centroid,i_centroid);
				if(centroid_distance < 2.5) //have to be careful here of large lateral changes
				{
					float distance = getdistance(outer[i],clusters[j]);
					if(distance <= distance_max) tree_member.push_back(j);
				}
			}
			std::sort(tree_member.begin(),tree_member.end(),std::greater<int>());
			for(int k=0;k<tree_member.size();k++)
			{
				tmp.push_back(clusters[tree_member[k]]);
				clusters.erase(clusters.begin()+tree_member[k]);
			}
		}
		if(tmp.size() !=0)
		{
			outer.clear();
			for(int l=0;l<tmp.size();l++)
			{
				tree.push_back(tmp[l]);
				outer.push_back(tmp[l]);
			}
		}
		else done_something = false;
		std::cout << "." << std::flush;
		count++;
		if(count == 35)
		{
			std::cout << "break" << std::endl;
			break;
		}
	}
	for(int m=0;m<tree.size();m++) *t += *tree[m];
}

void writeclusters(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters, std::string fname, bool dopca)
{
	pcl::PCDWriter writer;
	if(dopca == false)
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
	if(dopca == true)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
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
			float length = sqrt(pow(max[0]-min[0],2) + pow(max[1]-min[1],2) + pow(max[2]-min[2],2));
			for(float b= -length/2; b < length/2; b+=0.01)
			{
				pcl::PointXYZRGB point;
				point.x = centroid[0] + b * eigenvectors(0,2);
				point.y = centroid[1] + b * eigenvectors(1,2);
				point.z = centroid[2] + b * eigenvectors(2,2);
				point.r = 253;
				point.g = 215;
				point.b = 228;
				out_cloud->insert(out_cloud->end(),point);
			}
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
}

int main (int argc, char** argv)
{
	for(int i=5;i<argc;i++)
	{
		std::cout << "---------------" << std::endl;
		std::stringstream ss;
		pcl::PointCloud<pcl::PointXYZ>::Ptr tree(new pcl::PointCloud<pcl::PointXYZ>);
		//pcl::PCDReader reader;
		//reader.read(argv[i],*tree);
		pcl::ASCIIReader reader;
		reader.read(argv[i],*tree);
		//
		std::cout << "Generating slices: " << std::flush;
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
		float step = 100;
		clusters = getslices(tree,step);
		ss.str("");
		ss << "slices_" << i << ".pcd";
		writeclusters(clusters,ss.str(),false);
		std::cout << ss.str() << std::endl;
		//
//		std::cout << "Cluster segmentation: " << std::flush;
		int NSearch = 9;
		int min_count = 3;
//		segmentclusters(NSearch,min_count,clusters);
//		ss.str("");
//		ss << "slices_clusters_" << i << ".pcd";
//		writeclusters(clusters,ss.str(),false);
//		std::cout << ss.str() << std::endl;
		//
		std::cout << "Cluster correction: " << std::flush;
		NSearch = 9;
		float smoothness = atof(argv[1]);
		correctclusters(NSearch,min_count,smoothness,clusters);
		ss.str("");
		ss << "slices_clusters_corrected_" << i << ".pcd";
		writeclusters(clusters,ss.str(),false);
		std::cout << ss.str() << std::endl;
		//
		std::cout << "Cluster pruning: " << std::flush;
		float sphericity_max = atof(argv[2]); 
		float eigenratio_a_max = atof(argv[3]);
		float eigenratio_b_min = atof(argv[4]);
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> leaf_clusters;
		leaf_clusters = pruneclusters(clusters,sphericity_max,eigenratio_a_max,eigenratio_b_min);
		ss.str("");
		ss << "slices_clusters_corrected_pruned_" << i << ".pcd";
		writeclusters(clusters,ss.str(),false);
		if(leaf_clusters.size() > 0)
		{
			ss.str("");
			ss << "slices_clusters_corrected_pruned_leaves_" << i << ".pcd";
			writeclusters(leaf_clusters,ss.str(),false);
		}
		std::cout << clusters.size() << " " << leaf_clusters.size() << std::endl;
	}
	return 0;
}
