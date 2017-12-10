//Andrew Burt - a.burt.12@ucl.ac.uk

#include <numeric>

#include <pcl/io/ascii_io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/registration/distances.h>
