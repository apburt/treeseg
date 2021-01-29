//Andrew Burt - a.burt@ucl.ac.uk

#include <sstream>
#include <fstream>
#include <cmath>

#include <dirent.h>

#include <riegl/scanlib.hpp>

#include "treeseg.h"

struct pcloud 
{
	std::vector<float> x,y,z;
	std::vector<float> range;
	std::vector<float> amplitude;
	std::vector<float> reflectance;
	std::vector<std::uint16_t> deviation;
	std::vector<std::uint16_t> return_number;
	std::uint16_t scan_number;
	std::vector<float> time;
	float matrix[16];
};

class importer : public scanlib::pointcloud
{	
	pcloud &pc;
	public:
		importer(pcloud &pc) : scanlib::pointcloud(false), pc(pc){}
	protected:
		//void on_echo_transformed(echo_type echo)
		void on_shot_end()
		{
			for(int i=0;i<target_count;i++)
			{
				pc.x.push_back(targets[i].vertex[0]);
				pc.y.push_back(targets[i].vertex[1]);
				pc.z.push_back(targets[i].vertex[2]);
				pc.range.push_back(targets[i].echo_range);
				pc.amplitude.push_back(targets[i].amplitude);
				pc.reflectance.push_back(targets[i].reflectance);
				pc.deviation.push_back((std::uint16_t)targets[i].deviation);
				pc.return_number.push_back((std::uint16_t)i+1);
				pc.time.push_back((float)targets[i].time);
				//the following truncations have taken place:
				//deviation: float to uint16_t
				//return number: int to uint16_t
				//time: double to float
			}	
		}
};

int main(int argc, char **argv)
{
	std::vector<std::string> args(argv+1,argv+argc);
	std::string top_dir = args[0];
	if(top_dir[top_dir.length()-1] != '/') top_dir = top_dir + "/";
	std::ifstream cfile;
	cfile.open(args[1]);
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
	float plot_xmin = coordfile[0] - 20;
	float plot_xmax = coordfile[1] + 20;
	float plot_ymin = coordfile[2] - 20;
	float plot_ymax = coordfile[3] + 20;
	float tile_area = std::stof(args[2]);
	float tile_length = sqrt(tile_area);
	int tile_count = 0;
	for(float x=plot_xmin;x<plot_xmax;x+=tile_length)
	{
		for(float y=plot_ymin;y<plot_ymax;y+=tile_length) tile_count++;
	}
	float tile_coords[tile_count][4];
	unsigned long int tile_pointcount[tile_count] = {0};
	int c = 0;
	for(float x=plot_xmin;x<plot_xmax;x+=sqrt(tile_area))
	{
		for(float y=plot_ymin;y<plot_ymax;y+=sqrt(tile_area))
		{
			tile_coords[c][0] = x;
			tile_coords[c][1] = x+sqrt(tile_area);
			tile_coords[c][2] = y;
			tile_coords[c][3] = y+sqrt(tile_area);
			c++;
		}
	}
	float deviation_max = std::stof(args[3]);
	std::string fname = args[4];
	std::stringstream ss;
	std::ofstream xyzfiles[tile_count];
	std::string xyznames[tile_count];
	std::string pcdnames[tile_count];
	for(int j=0;j<tile_count;j++)
	{
		ss.str("");
		ss << fname << ".tile." << j << ".xyz";
		xyzfiles[j].open(ss.str(),std::ios::binary);
		xyznames[j] = ss.str();
		ss.str("");
		ss << fname << ".tile." << j << ".pcd";
		pcdnames[j] = ss.str();
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
				if(position_contents[l][14] == 'r' && position_contents[l][15] == 'x' && position_contents[l][16] == 'p' && position_contents[l].length() == 17)
				//if(position_contents[l][14] == 'm' && position_contents[l][15] == 'o' && position_contents[l][16] == 'n' && position_contents[l].length() == 21)
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
			pc.scan_number = (std::uint16_t)atoi(scan_number.c_str());
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
				for(int n=0;n<tile_count;n++)
				{
					if(X >= tile_coords[n][0] && X < tile_coords[n][1])
					{
						if(Y >= tile_coords[n][2] && Y < tile_coords[n][3])
						{
							if(pc.deviation[m] <= deviation_max)
							{
								xyzfiles[n].write(reinterpret_cast<const char*>(&X),sizeof(X));
								xyzfiles[n].write(reinterpret_cast<const char*>(&Y),sizeof(Y));
								xyzfiles[n].write(reinterpret_cast<const char*>(&Z),sizeof(Z));
								#if XYZRRDRS == true
									xyzfiles[n].write(reinterpret_cast<const char*>(&pc.range[m]),sizeof(pc.range[m]));
									xyzfiles[n].write(reinterpret_cast<const char*>(&pc.reflectance[m]),sizeof(pc.reflectance[m]));
									xyzfiles[n].write(reinterpret_cast<const char*>(&pc.deviation[m]),sizeof(pc.deviation[m]));
									xyzfiles[n].write(reinterpret_cast<const char*>(&pc.return_number[m]),sizeof(pc.return_number[m]));
									xyzfiles[n].write(reinterpret_cast<const char*>(&pc.scan_number),sizeof(pc.scan_number));
								#endif
								tile_pointcount[n] += 1;
							}
						}
					}
				}
			}
		}
	}
	for(int p=0;p<tile_count;p++)
	{
		xyzfiles[p].close();
	}
	for(int q=0;q<tile_count;q++)
	{
		std::ofstream headerstream("header.tmp");		
		#if XYZRRDRS == true
			headerstream << "VERSION 0.7" << std::endl << "FIELDS x y z range reflectance deviation return_number scan_number" << std::endl << "SIZE 4 4 4 4 4 2 2 2" << std::endl << "TYPE F F F F F U U U" << std::endl << "COUNT 1 1 1 1 1 1 1 1" << std::endl << "WIDTH " << tile_pointcount[q] << std::endl << "HEIGHT 1" << std::endl << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl << "POINTS " << tile_pointcount[q] << std::endl << "DATA binary" << std::endl;
		#else
			headerstream << "VERSION 0.7" << std::endl << "FIELDS x y z" << std::endl << "SIZE 4 4 4" << std::endl << "TYPE F F F" << std::endl << "COUNT 1 1 1" << std::endl << "WIDTH " << tile_pointcount[q] << std::endl << "HEIGHT 1" << std::endl << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl << "POINTS " << tile_pointcount[q] << std::endl << "DATA binary" << std::endl;
		#endif
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
