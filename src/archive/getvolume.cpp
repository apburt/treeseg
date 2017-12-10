//Andrew Burt - a.burt@ucl.ac.uk

#include <treeseg.hpp>

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

struct treeinfo
{
	float x,y,d,h,c;
};
	
treeinfo getDBH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int NSearch, float z_step, float diff_max)
{
	treeinfo data;
	Eigen::Vector4f min,max;
	pcl::getMinMax3D(*cloud,min,max);
	float z = min[2] + 1.3; //1.3m is centre of first slice
	float diameter,x,y;
	bool stable = false;
	int j = 0;
	while(stable != true)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr slice(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr slice_back(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr slice_forward(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(cloud);
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
		if(cyl.rad > 0 && diff_mean <= diff_max)
		{
			data.d = dbh;
			data.x = x;
			data.y = y;
			stable = true;
		}
		z += 0.1; //thickness != step
		j++;
	}
	return data;
}

void segmentvolume(pcl::PointCloud<pcl::PointXYZ>::Ptr stem, pcl::PointCloud<pcl::PointXYZ>::Ptr plot, treeinfo tdata, pcl::PointCloud<pcl::PointXYZ>::Ptr &volume)
{
	Eigen::Vector4f min,max;
	pcl::getMinMax3D(*stem,min,max);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(plot);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(tdata.x-tdata.c/2,tdata.x+tdata.c/2);
	pass.filter(*volume);
	pass.setInputCloud(volume);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(tdata.y-tdata.c/2,tdata.y+tdata.c/2);
	pass.filter(*volume);
	pass.setInputCloud(volume);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(max[2],min[2]+tdata.h);
	pass.filter(*volume);
	*volume += *stem;
}

int main (int argc, char** argv)
{
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	pcl::PointCloud<pcl::PointXYZ>::Ptr plot(new pcl::PointCloud<pcl::PointXYZ>);
	int pos;
	for(int i=4;i<argc;i++)
	{
		std::string fname;
		std::vector<std::string> name1,name2,name3;
		boost::split(name1,argv[i],boost::is_any_of("/"));
		boost::split(name2,name1[name1.size()-1],boost::is_any_of("."));
		boost::split(name3,name2[0],boost::is_any_of("_"));
		fname = name3[0];
		if(fname == "stem")
		{
			pos = i;
			break;
		}
		else
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
			reader.read(argv[i],*tmp);
			*plot += *tmp;
		}
	}
	for(int i=pos;i<argc;i++)
	{
		std::cout << "---------------" << std::endl;
		//
		std::string fname;
		std::vector<std::string> name1;
		std::vector<std::string> name2;
		std::vector<std::string> name3;
		boost::split(name1,argv[i],boost::is_any_of("."));
		boost::split(name2,name1[name1.size()-2],boost::is_any_of("/"));
		boost::split(name3,name2[name2.size()-1],boost::is_any_of("_"));
		fname = name3[name3.size()-1];
		std::stringstream ss;
		ss << "volume_" << fname << ".pcd";
		std::cout << argv[i] << " " << ss.str() << std::endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr stem(new pcl::PointCloud<pcl::PointXYZ>);
		reader.read(argv[i],*stem);
		//
		std::cout << "Estimating DBH: " << std::flush;
		int NSearch = atoi(argv[1]);
		float z_step = atof(argv[2]);
		float diff_max = atof(argv[3]);
		treeinfo tdata;
		tdata = getDBH(stem,NSearch,z_step,diff_max);
		std::cout << tdata.d << " m" << std::endl;
		//
		std::cout << "Crown dimensions: " << std::flush;
		tdata.h = maxheight(tdata.d);
		tdata.c = maxcrown(tdata.d);
		std::cout << tdata.h << "m x " << tdata.c << "m (HxW)" << std::endl; 
		//
		std::cout << "Segmenting volume: " << std::flush;
		pcl::PointCloud<pcl::PointXYZ>::Ptr volume(new pcl::PointCloud<pcl::PointXYZ>);
		segmentvolume(stem,plot,tdata,volume);
		writer.write(ss.str(),*volume,true);
		std::cout << ss.str() << std::endl;
	}
	return 0;
}
