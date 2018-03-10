//Andrew Burt - a.burt@ucl.ac.uk

#include <string>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <cmath>

#include <dirent.h>

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
	std::ifstream cfile;
	cfile.open(argv[2]);
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
	float x_min = coordfile[0]-25;
	float x_max = coordfile[1]+25;
	float y_min = coordfile[2]-25;
	float y_max = coordfile[3]+25;
	float area = atof(argv[3]);
	int length = ceil((x_max-x_min)/sqrt(area)) * ceil((y_max-y_min)/sqrt(area)) + 1;
	float coords[length][4];
	int c = 0;
	int count[length];
	for(float x=x_min;x<x_max;x+=sqrt(area))
	{
		for(float y=y_min;y<y_max;y+=sqrt(area))
		{
			coords[c][0] = x;
			coords[c][1] = x+sqrt(area);
			coords[c][2] = y;
			coords[c][3] = y+sqrt(area);
			count[c] = 0;
			c++;
		}
	}
	coords[length-1][0] = (x_max + x_min) / 2 - 15;
	coords[length-1][1] = (x_max + x_min) / 2 + 15;
	coords[length-1][2] = (y_max + y_min) / 2 - 15;
	coords[length-1][3] = (y_max + y_min) / 2 + 15;
	count[length-1] = 0;
	float deviation_max = atof(argv[4]);
	std::string fname = argv[5];
	std::stringstream ss;
	std::ofstream xyzfiles[length];
	std::string xyznames[length];
	std::string pcdnames[length];
	for(int j=0;j<length;j++)
	{
		if(j == length-1)
		{
			ss.str("");
			ss << fname << ".sample.xyz";
			xyzfiles[j].open(ss.str(),std::ios::binary);
			xyznames[j] = ss.str();
			ss.str("");
			ss << fname << ".sample.pcd";
			pcdnames[j] = ss.str();
		}
		else
		{
			ss.str("");
			ss << fname << "_" << j << ".xyz";
			xyzfiles[j].open(ss.str(),std::ios::binary);
			xyznames[j] = ss.str();
			ss.str("");
			ss << fname << "_" << j << ".pcd";
			pcdnames[j] = ss.str();
		}
	}
	std::vector<std::string> positions;
	DIR *tdir = NULL;
	tdir = opendir (top_dir.c_str());
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
			pcloud pc;
			try
			{

				std::shared_ptr<scanlib::basic_rconnection> rc;
				rc = scanlib::basic_rconnection::create(rxpname);
				rc->open();
				scanlib::decoder_rxpmarker dec(rc);
				importer imp(pc);
				scanlib::buffer buf;
				for(dec.get(buf);!dec.eoi();dec.get(buf))
				{
					imp.dispatch(buf.begin(), buf.end());
				}
				rc->close();
			}
			catch(...)
			{
				continue;
			}
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
				for(int n=0;n<length;n++)
				{
					if(X >= coords[n][0] && X < coords[n][1])
					{
						if(Y >=coords[n][2] && Y < coords[n][3])
						{
							if(pc.deviation[m] <= deviation_max)
							{
								xyzfiles[n].write(reinterpret_cast<const char*>(&X),sizeof(X));
								xyzfiles[n].write(reinterpret_cast<const char*>(&Y),sizeof(Y));
								xyzfiles[n].write(reinterpret_cast<const char*>(&Z),sizeof(Z));
							xyzfiles[n].write(reinterpret_cast<const char*>(&pc.reflectance[n]),sizeof(pc.reflectance[n]));
								count[n] += 1;
							}
						}
					}
				}
			}
		}
	}
	for(int p=0;p<length;p++)
	{
		xyzfiles[p].close();
	}
	for(int q=0;q<length;q++)
	{
		std::ofstream headerstream("header.tmp");
		headerstream << "VERSION 0.7" << std::endl << "FIELDS x y z" << std::endl << "SIZE 4 4 4" << std::endl << "TYPE F F F" << std::endl << "COUNT 1 1 1" << std::endl << "WIDTH " << count[q] << std::endl << "HEIGHT 1" << std::endl << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl << "POINTS " << count[q] << std::endl << "DATA binary" << std::endl;
		headerstream.close();
		ss.str("");
		ss << "cat header.tmp " << xyznames[q] << " > " << pcdnames[q] << "; rm header.tmp " << xyznames[q];
		std::string string;
		const char* cc;
		string = ss.str();
		cc = string.c_str();
		system(cc);
	}
	return 0;
}
