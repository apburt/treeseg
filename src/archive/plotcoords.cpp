//Andrew Burt - a.burt@ucl.ac.uk

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <fstream>
#include <algorithm>

#include <dirent.h>
#include <limits.h>

int main(int argc,char** argv)
{
	std::string matrix_dir = argv[1];
	if(matrix_dir[matrix_dir.length()-1] != '/') matrix_dir = matrix_dir + '/';
	std::vector<std::string> fnames;
	char buf[PATH_MAX + 1];
	DIR *dir = NULL;
	struct dirent *drnt = NULL;
	dir = opendir(matrix_dir.c_str());
	while(drnt = readdir(dir)) fnames.push_back(drnt->d_name);
	closedir(dir);
	std::vector<float> x,y;
	for(int i=0;i<fnames.size();i++)
	{
		std::stringstream ss;
		ss << matrix_dir << fnames[i];
		std::string fname = ss.str();
		if(fname[fname.length()-4] == '.' && fname[fname.length()-3] == 'd')
		{
			float matrix[16];
			std::fstream mfile;
			mfile.open(fname);
			int no_count = 0;
			if(mfile.is_open())
			{
				while(!mfile.eof())
				{
					mfile >> matrix[no_count];
					no_count++;
				}
			}
			x.push_back(matrix[3]);
			y.push_back(matrix[7]);
		}
	}
	float x_sum = 0;
	float y_sum = 0;
	for(int j=0;j<x.size();j++)
	{
		x_sum += x[j];
		y_sum += y[j];
	}
//	float x_mean = x_sum / x.size();
//	float y_mean = y_sum / y.size();
	auto xmm = std::minmax_element(x.begin(),x.end());
	auto ymm = std::minmax_element(y.begin(),y.end());
	float x_min = x[xmm.first-x.begin()];
	float x_max = x[xmm.second-x.begin()];
	float y_min = y[ymm.first-y.begin()];
	float y_max = y[ymm.second-y.begin()];
	std::cout << x_min << " " << x_max << " " << y_min << " " << y_max << std::endl;
//	std::cout << x_mean << " " << y_mean << std::endl; 
	return 0;
}
