//Andrew Burt - a.burt@ucl.ac.uk

#include <fstream>
#include <treeseg.hpp>
#include <riegl/scanlib.hpp>

struct pcloud 
{
	std::vector<float> x,y,z;
	std::vector<float> range;
	std::vector<float> amplitude;
	std::vector<float> reflectance;
	std::vector<float> deviation;
	std::vector<float> return_number;
	float scan_number;
	std::vector<double> time;
	float matrix[16];
};

class importer : public scanlib::pointcloud
{	
	pcloud &pc;
	public:
		importer(pcloud &pc) : scanlib::pointcloud(false), pc(pc){}
	protected:
		void on_shot_end()
		{
			for(int i=0;i<targets.size();i++)
			{
				pc.x.push_back(targets[i].vertex[0]);
				pc.y.push_back(targets[i].vertex[1]);
				pc.z.push_back(targets[i].vertex[2]);
				pc.range.push_back(targets[i].echo_range);
				pc.amplitude.push_back(targets[i].amplitude);
				pc.reflectance.push_back(targets[i].reflectance);
				pc.deviation.push_back(targets[i].deviation);
				pc.return_number.push_back(i+1);
				pc.time.push_back(targets[i].time);
			}	
		}
};

int main(int argc,char** argv)
{
	std::string top_dir = argv[1];
	if(top_dir[top_dir.length()-1] != '/') top_dir = top_dir + "/";
	float resolution = atof(argv[2]);
	pcl::PCDReader reader;
	std::vector<std::string> fnames;
	std::vector<int> count;
	std::vector<float> x,y,z;
	for(int i=3;i<argc;i++)
	{
		std::string fname;
		std::vector<std::string> name1;
		std::vector<std::string> name2;
		std::vector<std::string> name3;
		boost::split(name1,argv[i],boost::is_any_of("."));
		boost::split(name2,name1[name1.size()-2],boost::is_any_of("/"));
		boost::split(name3,name2[name2.size()-1],boost::is_any_of("_"));
		fname = name3[name3.size()-2]+"_"+name3[name3.size()-1]+".txt";
		pcl::PointCloud<pcl::PointXYZ>::Ptr tree(new pcl::PointCloud<pcl::PointXYZ>);
		reader.read(argv[i],*tree);

		Eigen::Vector4f min,max;
		pcl::getMinMax3D(*tree,min,max);	

		pcl::PassThrough<pcl::PointXYZ> pass;

		pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
		pass.setInputCloud(tree);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(min[2]+1.2,min[2]+1.4);
		pass.filter(*tmp);

		pcl::getMinMax3D(*tmp,min,max);


		x.push_back((max[0]+min[0])/2);
		y.push_back((max[1]+min[1])/2);

/*
		Eigen::Vector4f min,max;
		float res_const = resolution*0.9;
		pcl::getMinMax3D(*tree,min,max);
		pcl::PassThrough<pcl::PointXYZ> pass;
		int c=0;
		for(float xt=min[0]-res_const; xt<max[0]+res_const; xt+=resolution)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr tmpx(new pcl::PointCloud<pcl::PointXYZ>);
			pass.setInputCloud(tree);
			pass.setFilterFieldName("x");
			pass.setFilterLimits(xt,xt+resolution);
			pass.filter(*tmpx);
			for(float yt=min[1]-res_const; yt<max[1]+res_const; yt+=resolution)
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr tmpy(new pcl::PointCloud<pcl::PointXYZ>);
				pass.setInputCloud(tmpx);
				pass.setFilterFieldName("y");
				pass.setFilterLimits(yt,yt+resolution);
				pass.filter(*tmpy);
				for(float zt=min[2]-res_const; zt<max[2]+res_const; zt+=resolution)
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr tmpz(new pcl::PointCloud<pcl::PointXYZ>);
					pass.setInputCloud(tmpy);
					pass.setFilterFieldName("z");
					pass.setFilterLimits(zt,zt+resolution);
					pass.filter(*tmpz);
					if(tmpz->points.size() != 0)
					{
						x.push_back(xt);
						y.push_back(yt);
						z.push_back(zt);
						c++;		
					}
				}
			}
		}
*/
		count.push_back(1);
		fnames.push_back(fname);
	}

	std::ofstream cloudfiles[fnames.size()];
	for(int j=0;j<fnames.size();j++)
	{
		cloudfiles[j].open(fnames[j]);
		cloudfiles[j].precision(3);
		cloudfiles[j].setf(std::ios::fixed);
	}
	std::stringstream ss;
	std::vector<std::string> positions;
	DIR *tdir = NULL;
	tdir = opendir(top_dir.c_str());
	struct dirent *tent = NULL;
	while(tent = readdir(tdir)) positions.push_back(tent->d_name);
	closedir(tdir);
	for(int k=0;k<positions.size();k++)
	{
		if(positions[k][0] == 'S' && positions[k][4] == 'P')
		{
			ss.str("");
			ss << top_dir << positions[k];
			std::string position;
			const char* c_position;
			position = ss.str();
			c_position = position.c_str();
			std::vector<std::string> position_contents;
			DIR *pdir = NULL;
			pdir = opendir(c_position);
			struct dirent *pent = NULL;
			while(pent = readdir(pdir)) position_contents.push_back(pent->d_name);
			closedir(pdir);
			std::string rxpname;
			for(int l=0;l<position_contents.size();l++)
			{
				if(position_contents[l][14] == 'r' && position_contents[l][15] == 'x' && position_contents[l][16] == 'p' && position_contents[l].length() == 17 )
				{
					ss.str("");
					ss << top_dir << positions[k] << "/" << position_contents[l];
					rxpname = ss.str();
				}
			}
			ss.str("");
			ss << top_dir << "matrix/" << positions[k][7] << positions[k][8] << positions[k][9] <<".dat";
			std::string matrixname = ss.str();
			std::cout << rxpname << " " << " " << matrixname << std::endl;
			std::shared_ptr<scanlib::basic_rconnection> rc;
			rc = scanlib::basic_rconnection::create(rxpname);
			rc->open();
			scanlib::decoder_rxpmarker dec(rc);
			pcloud pc;
			importer imp(pc);
			scanlib::buffer buf;
			for(dec.get(buf);!dec.eoi();dec.get(buf))
			{
				imp.dispatch(buf.begin(), buf.end());
			}
			rc->close();
			ss.str("");
			if(positions[k][7] == '0' && positions[k][8] == '0') ss << positions[k][9];
			else if(positions[k][7] == '0') ss << positions[k][8] << positions[k][9];
			else ss << positions[k][7] << positions[k][8] << positions[k][9];
			std::string scan_number = ss.str();
			pc.scan_number = atof(scan_number.c_str());
			std::ifstream mfile;
			mfile.open(matrixname);
			int no_count = 0;
			if(mfile.is_open())
			{
				while(!mfile.eof())
				{
					mfile >> pc.matrix[no_count];
					no_count++;
				}
			}
			for(int m=0;m<pc.x.size();m++)
			{
				float X = ((pc.x[m]*pc.matrix[0])+(pc.y[m]*pc.matrix[1])+(pc.z[m]*pc.matrix[2]))+pc.matrix[3];
				float Y = ((pc.x[m]*pc.matrix[4])+(pc.y[m]*pc.matrix[5])+(pc.z[m]*pc.matrix[6]))+pc.matrix[7];
				float Z = ((pc.x[m]*pc.matrix[8])+(pc.y[m]*pc.matrix[9])+(pc.z[m]*pc.matrix[10]))+pc.matrix[11];
				int sum = 0;
				for(int n=0;n<fnames.size();n++)
				{
					int pos = sum; 
					int end = sum + count[n] - 1; 
					sum += count[n];
					for(int p=pos;p<=end;p++)
					{
					//	if(X >= x[p] && X <= x[p]+resolution)
					//	{
					//		if(Y >= y[p] && Y <= y[p]+resolution)
					//		{
					//			if(Z >= z[p] && Z <= z[p]+resolution)
					//			{
					
						if(X >= x[p]-resolution/2 && X <= x[p]+resolution/2)
						{
							if(Y >= y[p]-resolution/2 && Y <= y[p]+resolution/2)
							{	
								if(pc.deviation[m] <= 15)
								{
									cloudfiles[n] << X << " " << Y << " " << Z << std::endl;
								}
							}
						}
					}
				}
			}
		}
	}
	for(int q=0;q<fnames.size();q++) cloudfiles[q].close();
	return 0;
}
