//Andrew Burt - a.burt@ucl.ac.uk

#include <treeseg.hpp>

struct cylParams
{
	bool ismodel;
	float x,y,z;
	float dx,dy,dz;
	float rad;
	float len;
	int ccount,icount;
	float steprad;
	float stepcov;
	float radratio;
	float ransac_dist;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

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

int main (int argc, char** argv)
{
	int NSearch = atof(argv[1]);
	float z_step = atof(argv[2]);
	float diff_max = atof(argv[3]);
//	float radratio_min = atof(argv[4]);
//	float stepcov_max = atof(argv[5]);
	for(int i=4;i<argc;i++)
	{
		std::string fname;
		std::vector<std::string> name1;
		std::vector<std::string> name2;
		boost::split(name1,argv[i],boost::is_any_of("."));
		boost::split(name2,name1[name1.size()-2],boost::is_any_of("/"));
		fname = name2[name2.size()-1];
		float x,y,dbh,h,c;
		pcl::PointCloud<pcl::PointXYZ>::Ptr tree(new pcl::PointCloud<pcl::PointXYZ>);
//		pcl::PCDReader reader;
//		reader.read(argv[i],*tree);
		pcl::ASCIIReader reader;
		reader.read(argv[i],*tree);
		Eigen::Vector4f min,max;
		pcl::getMinMax3D(*tree,min,max);
		h = max[2]-min[2];	
		c = sqrt(pow(max[0]-min[0],2)+pow(max[1]-min[1],2)); 
		float z = min[2] + 1.3; //1.3m is centre of first slice
		bool stable = false;
		int j = 0;
		while(stable != true)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr slice(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr slice_back(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr slice_forward(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PassThrough<pcl::PointXYZ> pass;
			pass.setInputCloud(tree);
			pass.setFilterFieldName("z");
			pass.setFilterLimits(z-z_step/2,z+z_step/2);
			pass.filter(*slice);
			if(j == 0)
			{
				Eigen::Vector4f smin,smax;
				pcl::getMinMax3D(*slice,smin,smax);
				x = (smax[0] + smin[0]) / 2;
				y = (smax[1] + smin[1]) / 2;
			}
			pass.setFilterLimits(z-z_step*1.5,z-z_step/2);
			pass.filter(*slice_back);
			pass.setFilterLimits(z+z_step/2,z+z_step*1.5);
			pass.filter(*slice_forward);
			cylParams cyl,cyl_back,cyl_forward;
			cyl = fitCylinder(slice,NSearch);
			cyl_back = fitCylinder(slice_back,NSearch);
			cyl_forward = fitCylinder(slice_forward,NSearch);
			float dbh = (cyl.rad + cyl_back.rad + cyl_forward.rad) / 3 * 2;
			float diff_back = fabs(cyl.rad - cyl_back.rad) / cyl.rad;
			float diff_for =  fabs(cyl.rad - cyl_forward.rad) / cyl.rad;
			float diff_mean = (diff_back + diff_for) / 2;
			std::cout << dbh << " " << diff_mean << " " << cyl.radratio << " " << cyl.stepcov << std::endl;
			if(cyl.rad > 0 && diff_mean <= diff_max)
			{
				std::cout << fname << " " << x << " " << y << " "  << dbh << " " << h << " " << c << std::endl; 
				stable = true;
			}


			pcl::PCDWriter writer;
			writer.write("slice.pcd",*slice,true);
			writer.write("slice_back.pcd",*slice_back,true);
			writer.write("slice_forward.pcd",*slice_forward,true);



			z += 0.1; //thickness != step
			j++;
		}
	}
	return 0;
}
