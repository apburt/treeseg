/*
* treeseg_pointtype.h
*
* MIT License
*
* Copyright 2020 Andrew Burt - a.burt@ucl.ac.uk
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>

struct PointTreeseg
{
	PCL_ADD_POINT4D;
/*
	union
	{
		struct
		{
			float range;
			float reflectance;
		};
		float data_rr[4]; // 16bytes
	};
	union
	{
		struct
		{
			uint16_t deviation;
			uint16_t return_number;
			uint16_t scan_number;
		};
		uint16_t data_drs[4]; // 8bytes
	};
	//out of my depth here wrt memory alignment - unsure for second union
*/
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
POINT_CLOUD_REGISTER_POINT_STRUCT
(
	PointTreeseg,
	(float,x,x)
	(float,y,y)
	(float,z,z)
/*
	(float,range,range)
	(float,reflectance,reflectance)
	(uint16_t,deviation,deviation)
	(uint16_t,return_number,return_number)
	(uint16_t,scan_number,scan_number)
*/
)
