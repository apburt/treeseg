//Andrew Burt - a.burt@ucl.ac.uk

#include <treeseg.hpp>

float nnsearch(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int NSearches)
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
//	float sq_sum = std::inner_product(dist.begin(), dist.end(), dist.begin(), 0.0);
//	float stddev = std::sqrt(sq_sum / dist.size() - dist_mean * dist_mean);
	return dist_mean;
}

std::vector<std::vector<float>> nncloudstep(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,float delta_z,int NSearches)
{
	std::vector<std::vector<float>> nn;
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
		if(tmp->points.size() > NSearches)
		{
			float d = nnsearch(tmp,NSearches);
			float pos = z+delta_z;
			std::vector<float> results;
			results.push_back(pos);
			results.push_back(d);
			nn.push_back(results);
		}
	}
	return nn;
}

void euclideancluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float dist, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters)
{
	std::vector<pcl::PointIndices> indices;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(dist);
	ec.setMinClusterSize(3);
	ec.setMaxClusterSize(std::numeric_limits<int>().max());
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(indices);
	for(std::vector<pcl::PointIndices>::iterator it=indices.begin();it!=indices.end();it++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for(std::vector<int>::iterator pit=it->indices.begin();pit!=it->indices.end();pit++)
		{
			cluster->insert(cluster->end(),cloud->points[*pit]);
		}
		clusters.push_back(cluster);
	}
}

bool sortcol(const std::vector<int>& v1, const std::vector<int>& v2 )
{
	return v1[1] < v2[1];
}

int findstart(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters)
{
	std::vector<std::vector<int>> info;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for(int i=0;i<clusters.size();i++) *cloud += *clusters[i];
	Eigen::Vector4f min,max;
	pcl::getMinMax3D(*cloud,min,max);
	for(int i=0;i<clusters.size();i++)
	{
		Eigen::Vector4f c_min, c_max;
		pcl::getMinMax3D(*clusters[i],c_min,c_max);
		if(c_min[2] < min[2]+2)
		{
			std::vector<int> in;
			in.push_back(i);
			in.push_back(clusters[i]->points.size());
			info.push_back(in);
		}
	}
	std::sort(info.begin(),info.end(),sortcol);
	int idx = info[info.size()-1][0];	
	return idx;
}

void regiongrowing(int NSearch, int non, float smoothness, float curvature, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters)
{
	std::vector<pcl::PointIndices> indices;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
	pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud);
	ne.setKSearch(NSearch);
	ne.compute(*normals);
	pcl::RegionGrowing<pcl::PointXYZ,pcl::Normal> reg;
	reg.setMinClusterSize(3);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(non); //this is a sensitive value - keep fixed and vary smoothness and curvature
	reg.setInputCloud(cloud);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(smoothness/ 180 * M_PI);
	reg.setCurvatureThreshold(curvature);
	reg.extract(indices);
	for(std::vector<pcl::PointIndices>::iterator it=indices.begin();it!=indices.end();it++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for(std::vector<int>::iterator pit=it->indices.begin();pit!=it->indices.end();pit++)
		{
			cluster->insert(cluster->end(),cloud->points[*pit]);
		}       
		clusters.push_back(cluster);
	}
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> removeclusters(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters)
{
	std::vector<float> pos;
	Eigen::Vector4f min, max;
	for(int i=0;i<clusters.size();i++)
	{
		pcl::getMinMax3D(*clusters[i],min,max);
		pos.push_back(min[2]);
	}
	std::sort(pos.begin(),pos.end());
	std::vector<float> pos10;
	float len = pos.size() * 0.1;
	len = static_cast<int>(len);
	for(int j=1;j<len;j++) pos10.push_back(pos[j]);
	float pos10_sum = std::accumulate(pos10.begin(),pos10.end(),0.0);
	float pos10_mean = pos10_sum/pos10.size();
	std::vector<int> idx;
	for(int k=0;k<clusters.size();k++)
	{
		pcl::getMinMax3D(*clusters[k],min,max);
		if(min[2] < pos10_mean) idx.push_back(k); 
	}
	std::sort(idx.begin(),idx.end(),std::greater<int>());
	std::cout << clusters.size() << " " << idx.size() << std::endl;	
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_removed;
	clusters_removed = clusters;
	for(int l=0;l<idx.size();l++) clusters_removed.erase(clusters_removed.begin()+idx[l]);
	return clusters_removed; 
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

bool intersectionTest(Eigen::Vector4f min_a, Eigen::Vector4f max_a, Eigen::Vector4f min_b, Eigen::Vector4f max_b)
{
	bool intersection = true;
	if (min_a[0] > max_b[0]) intersection = false;
	if (max_a[0] < min_b[0]) intersection = false;
	if (min_a[1] > max_b[1]) intersection = false;
	if (max_a[1] < min_b[1]) intersection = false;
	if (min_a[2] > max_b[2]) intersection = false;
	if (max_a[2] < min_b[2]) intersection = false;
	return intersection;
}

float getmindistance(pcl::PointCloud<pcl::PointXYZ>::Ptr a, pcl::PointCloud<pcl::PointXYZ>::Ptr b)
{
	float distance = std::numeric_limits<float>::infinity();
	if(a->points.size() > b->points.size())
	{
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(a);
		int K = 1;
		for(pcl::PointCloud<pcl::PointXYZ>::iterator it=b->begin();it!=b->end();it++)
		{
			std::vector<float> p_dist;
			pcl::PointXYZ searchPoint;
			searchPoint.x = it->x;
			searchPoint.y = it->y;
			searchPoint.z = it->z;
			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);
			kdtree.nearestKSearch(searchPoint,K,pointIdxNKNSearch,pointNKNSquaredDistance);
			float d = sqrt(pointNKNSquaredDistance[0]);
			if(d < distance) distance = d;
		}
	}
	else
	{
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(b);
		int K = 1;
		for(pcl::PointCloud<pcl::PointXYZ>::iterator it=a->begin();it!=a->end();it++)
		{
			std::vector<float> p_dist;
			pcl::PointXYZ searchPoint;
			searchPoint.x = it->x;
			searchPoint.y = it->y;
			searchPoint.z = it->z;
			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);
			kdtree.nearestKSearch(searchPoint,K,pointIdxNKNSearch,pointNKNSquaredDistance);
			float d = sqrt(pointNKNSquaredDistance[0]);
			if(d < distance) distance = d;
		}
	}
	return distance;
}

float interpolate(float x, std::vector<std::vector<float>> data, bool extrapolate )
{
	std::vector<float> xData;
	std::vector<float> yData;
	for(int m=0;m<data.size();m++)
	{
		xData.push_back(data[m][0]);
		yData.push_back(data[m][1]);
	}
	int size = xData.size();
	int i = 0;                                                                  // find left end of interval for interpolation
	if(x >= xData[size-2])                                                 // special case: beyond right end
   	{
   	   i = size - 2;
   	}
	else
	{
		while ( x > xData[i+1] ) i++;
	   }
	double xL = xData[i], yL = yData[i], xR = xData[i+1], yR = yData[i+1];      // points on either side (unless beyond ends)
	if(!extrapolate)                                                         // if beyond ends of array and not extrapolating
	{
		if ( x < xL ) yR = yL;
		if ( x > xR ) yL = yR;
	}
	double dydx = ( yR - yL ) / ( xR - xL );                                    // gradient
	return yL + dydx * ( x - xL );                                              // linear interpolation
}

void buildtree(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters, std::vector<std::vector<float>> nn, pcl::PointCloud<pcl::PointXYZ>::Ptr &t)
{
	int idx = findstart(clusters);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> tree;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> outer;
	tree.push_back(clusters[idx]);
	outer.push_back(clusters[idx]);
	clusters.erase(clusters.begin()+idx);
	std::vector<float> dist;
	for(int m=0;m<nn.size();m++) dist.push_back(nn[m][1]);
	float r = interpolate(1,nn,true);
	float result = *std::max_element(std::begin(dist),std::end(dist));
	int count = 0;
	bool done_something = true;
	while(done_something == true)
	{
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> tmp;
		for(int i=0;i<outer.size();i++)
		{
			std::vector<int> tree_member;
			Eigen::Vector4f centroid_o;
			Eigen::Matrix3f covariancematrix_o;
			Eigen::Matrix3f eigenvectors_o;
			Eigen::Vector3f eigenvalues_o;
			pcl::PointCloud<pcl::PointXYZ>::Ptr outer_transformed(new pcl::PointCloud<pcl::PointXYZ>);
			Eigen::Vector4f min_o,max_o;
			float length_o;
			pcl::compute3DCentroid(*outer[i],centroid_o);
			pcl::computeCovarianceMatrix(*outer[i],centroid_o,covariancematrix_o);
			pcl::eigen33(covariancematrix_o,eigenvectors_o,eigenvalues_o);
			Eigen::Vector3f point_o(centroid_o[0],centroid_o[1],centroid_o[2]);
			Eigen::Vector3f direction_o(eigenvectors_o(0,2),eigenvectors_o(1,2),eigenvectors_o(2,2));
			Eigen::Affine3f transform_o;
			Eigen::Vector3f world_o(0,direction_o[2],-direction_o[1]); 
			direction_o.normalize(); 
			pcl::getTransformationFromTwoUnitVectorsAndOrigin(world_o,direction_o,point_o,transform_o); 
			pcl::transformPointCloud(*outer[i],*outer_transformed,transform_o); 
			pcl::getMinMax3D(*outer_transformed,min_o,max_o);
			length_o = max_o[2]-min_o[2];
			for(int j=0;j<clusters.size();j++)
			{
				//this loop is computationally expensive so only compute if intersection of outer[i] (expanded) and cluster[i] bounding boxes
				//Eigen::Vector4f min_a,max_a,min_b,max_b;
				//pcl::getMinMax3D(*outer[i],min_a,max_a);
				//pcl::getMinMax3D(*clusters[i],min_b,max_b);
				//min_a[0]=min_a[0]-result;min_a[1]=min_a[1]-result;min_a[2]=min_a[2]-result;
				//max_a[0]=max_a[0]+result;max_a[1]=max_a[1]+result;max_a[2]=max_a[2]+result;
				//min_b[0]=min_b[0]-result;min_b[1]=min_b[1]-result;min_b[2]=min_b[2]-result;
				//max_b[0]=max_b[0]+result;max_b[1]=max_b[1]+result;max_b[2]=max_b[2]+result;
				//bool intersect = intersectionTest(min_a,max_a,min_b,max_b);
				//if(intersect == true)
				//{
					float distance = getmindistance(outer[i],clusters[j]);
					Eigen::Vector4f centroid_c;
					pcl::compute3DCentroid(*clusters[j],centroid_c);
					float mind = interpolate((centroid_o[2]+centroid_c[2])/2,nn,true);
					if(distance <= mind)
					{
						Eigen::Matrix3f covariancematrix_c;
						Eigen::Matrix3f eigenvectors_c;
						Eigen::Vector3f eigenvalues_c;
						Eigen::Vector4f min_c,max_c;
						pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_transformed(new pcl::PointCloud<pcl::PointXYZ>);
						float length_c;
						pcl::computeCovarianceMatrix(*clusters[j],centroid_c,covariancematrix_c);
						pcl::eigen33(covariancematrix_c,eigenvectors_c,eigenvalues_c);
						Eigen::Vector3f point_c(centroid_c[0],centroid_c[1],centroid_c[2]);
						Eigen::Vector3f direction_c(eigenvectors_c(0,2),eigenvectors_c(1,2),eigenvectors_c(2,2));
						Eigen::Affine3f transform_c;
						Eigen::Vector3f world_c(0,direction_c[2],-direction_c[1]); 
						direction_c.normalize(); 
						pcl::getTransformationFromTwoUnitVectorsAndOrigin(world_c,direction_c,point_c,transform_c); 
						pcl::transformPointCloud(*clusters[j],*cluster_transformed,transform_c); 
						pcl::getMinMax3D(*cluster_transformed,min_c,max_c);
						length_c = max_c[2]-min_c[2];
						Eigen::Vector4f o_vector(eigenvectors_o(0,2),eigenvectors_o(1,2),eigenvectors_o(2,2),0);
						Eigen::Vector4f c_vector(eigenvectors_c(0,2),eigenvectors_c(1,2),eigenvectors_c(2,2),0);
						float angle = pcl::getAngle3D(o_vector,c_vector) * (180/M_PI);
						if(length_c < length_o)
						{
							tree_member.push_back(j);
						}	
					}
				//}
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
		count++;
		std::cout << "." << std::flush;
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
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	for(int i=2;i<argc;i++)
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
		ss << "tree_" << fname << ".pcd";
		std::cout << argv[i] << " " << ss.str() << std::endl;
		//
		std::cout << "Volume NN max: " << std::flush;
		pcl::PointCloud<pcl::PointXYZ>::Ptr volume(new pcl::PointCloud<pcl::PointXYZ>);
		reader.read(argv[i],*volume);
		std::vector<std::vector<float>> nnresults = nncloudstep(volume,2,50);
		std::vector<float> dist;
		for(int m=0;m<nnresults.size();m++) dist.push_back(nnresults[m][1]);
		float r = interpolate(1,nnresults,true);
		float result = *std::max_element(std::begin(dist),std::end(dist));
		std::cout << result << std::endl;
		//
		std::cout << "Cluster extraction: " << std::flush;
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
		euclideancluster(volume,result,clusters);
		ss.str("");
		ss << "clusters_" << fname << ".pcd";
		writeclusters(clusters,ss.str(),false);
		std::cout << ss.str() << std::endl;
		//
		std::cout << "Cluster correction: " << std::flush;	
		int idx = findstart(clusters);
		int NSearch = 50; //atoi(argv[1]);
		int non = 30; //atoi(argv[2]);
		float smoothness = atof(argv[1]);
		float curvature = 1; //atof(argv[4]);
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_corrected;
		regiongrowing(NSearch,non,smoothness,curvature,clusters[idx],clusters_corrected);
		ss.str("");
		ss << "clusters_corrected_" << fname << ".pcd";
		writeclusters(clusters_corrected,ss.str(),false);
		std::cout << ss.str() << std::endl;
		//
//		std::cout << "Removing neighbouring tree clusters: " << std::flush;
//		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_corrected_removed;
//		clusters_corrected_removed = removeclusters(clusters_corrected);
//		ss.str("");
//		ss << "clusters_corrected_removed_" << fname << ".pcd";
//		writeclusters(clusters_corrected_removed,ss.str(),false);
//		std::cout << ss.str() << std::endl;
		//
		std::cout << "Building tree: " << std::flush;
		pcl::PointCloud<pcl::PointXYZ>::Ptr tree(new pcl::PointCloud<pcl::PointXYZ>);
		buildtree(clusters_corrected,nnresults,tree);
		ss.str("");
		ss << "tree_" << fname << ".pcd";
		writer.write(ss.str(),*tree,true);
		std::cout << ss.str() << std::endl;
	}
	return 0;
}
