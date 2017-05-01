#include "globalMap.hpp"
#include <iostream>

using namespace std;
using namespace cartographer;

// creator
GlobalMap::GlobalMap() 
{
	heightmap_set = 0;
}

void GlobalMap::setMapParameters(float size, float resolution, float safety_offset)
{
	global_map_size = size;
	histogram_resolution = resolution; 
    histogram_cells = floor(global_map_size / histogram_resolution);

	this->safety_offset = safety_offset;

	// prepare maps
	global_map.create(histogram_cells, histogram_cells, CV_32FC1);
	global_map = cv::Scalar(std::numeric_limits<float>::quiet_NaN());
	count_map.create(histogram_cells, histogram_cells, CV_16UC1);
	count_map = cv::Scalar(1); //starts at one, so its easier to takle the case when one is newly seen

	
	heightmap_set = 1;
}

void GlobalMap::realWorldOrientation(cv::Mat relative_map, cv::Mat relative_mask_map, float yaw)
{
	// create enlarged map with double the size of the input map
	insert_rows = relative_map.rows*2;
	insert_cols = relative_map.cols*2; 
	
	rotated_map.create(insert_rows, insert_cols, CV_32FC1); 
	rotated_mask_map.create(insert_rows, insert_cols, CV_8UC1); 
	rotated_map.setTo(0.0);
	rotated_mask_map.setTo(0);
	
	// copy relative map in the middle of enlarged map todo create a  roi with rect to ease readability
	cv::Mat tmp;
	relative_map.convertTo(tmp,CV_32FC1); //force trav maps to be 32F so they can have Nan. TODO Find better way (or kill RAM)
	tmp.copyTo(rotated_map(cv::Rect(relative_map.cols/2,relative_map.rows,relative_map.cols, relative_map.rows)));
	relative_mask_map.copyTo(rotated_mask_map(cv::Rect(relative_mask_map.cols/2,relative_mask_map.rows,relative_mask_map.cols, relative_mask_map.rows)));
		
	rotated_map.setTo(std::numeric_limits<float>::quiet_NaN(),rotated_mask_map==0);
	
	// rotation center
	cv::Point2f rot_center((float)insert_rows/2.0,(float)insert_cols/2.0);
	
	// yaw rotation matrix
	cv::Mat transform = cv::getRotationMatrix2D(rot_center,(yaw/3.14159*180.0),1.0);

	// apply yaw rotation
	cv::warpAffine(rotated_map,rotated_map,transform,rotated_map.size(),CV_INTER_LINEAR);
	cv::warpAffine(rotated_mask_map,rotated_mask_map,transform,rotated_mask_map.size(),CV_INTER_LINEAR);
}

void GlobalMap::addToWorld(float y, float x, float z) // todo clear frame stuff. inversed here because of opencv
{
	x+=safety_offset;
	y+=safety_offset;
	
	float offsetX = rotated_map.cols/2.0;  
	float offsetY = rotated_map.rows/2.0;    
	

	// relative origin, center of the image to be extracted from global
	cv::Point2f relative_origin(floor(x/histogram_resolution), floor(y/histogram_resolution));

	// is this a deep operation? Check todo
	rotated_map += z;
	
	rotated_map.setTo(std::numeric_limits<float>::quiet_NaN(),rotated_mask_map==0); // todo check if first this inverses the map and if there is a better way
			
	// extract roi from previous snapshot
	cv::Mat old_map;
	old_map.create(insert_rows, insert_cols, CV_32FC1); 
	old_map.setTo(std::numeric_limits<float>::quiet_NaN());
	global_map(cv::Rect(floor(x/histogram_resolution)-offsetX, floor(y/histogram_resolution)-offsetY,old_map.cols, old_map.rows)).copyTo(old_map);

	cv::Mat old_count;
	old_count.create(insert_rows, insert_cols, CV_16UC1); 
	count_map(cv::Rect(floor(x/histogram_resolution)-offsetX, floor(y/histogram_resolution)-offsetY,old_map.cols, old_map.rows)).copyTo(old_count);

	// points valid only in new
	cv::Mat valid_old_mask = cv::Mat(old_map == old_map);
	//points valid only in old
	cv::Mat valid_new_mask = cv::Mat(rotated_map == rotated_map);
	// list of points where a value is present in both maps. 
	cv::Mat valid_both_mask;
	cv::bitwise_and(valid_new_mask,valid_old_mask, valid_both_mask);
	// list of points where a value is present in new but not old 
	cv::Mat valid_only_new_mask;
	cv::bitwise_and(valid_new_mask,valid_old_mask==0, valid_only_new_mask);
	// list of points where a value is present in old but not new 
	cv::Mat valid_only_old_mask;
	cv::bitwise_and(valid_new_mask==0,valid_old_mask, valid_only_new_mask);
    
	//old_count.setTo(1,valid_only_old_mask); // so the values that where only in the old are not multiplied/divided but still they
    // update height of points that were present in old map (mean) TO BE DONE BEFORE ADDING NEW ONES
    cv::multiply(old_map,old_count,old_map,1,CV_32FC1); // multiply old by viewcount
    cv::add(old_map,rotated_map,old_map,valid_both_mask); // add new to old where both are valid
    cv::add(old_count,1,old_count,valid_both_mask); // increase amount of count where both are valid
    cv::divide(old_map,old_count,old_map,1,CV_32FC1); // divide by new viewcount
        
    // directly copy all values where they were previously nan. If they where nan they stay nan
	rotated_map.copyTo(old_map,valid_old_mask==0);
	
	// copy updated old map in the global map
	old_map.copyTo(global_map(cv::Rect(floor(x/histogram_resolution)-offsetX, floor(y/histogram_resolution)-offsetY,old_map.cols, old_map.rows)));
	old_count.copyTo(count_map(cv::Rect(floor(x/histogram_resolution)-offsetX, floor(y/histogram_resolution)-offsetY,old_map.cols, old_map.rows)));
	
	// copy for display TODO fix this huge display by reference mess
	global_map.copyTo(display_map);
}

cv::Mat GlobalMap::getGlobalMap()
{
	return display_map;
}

cv::Mat GlobalMap::getDebugMap()
{
	return rotated_mask_map;
}

cv::Mat GlobalMap::getDebugMap2()
{
	return rotated_map;
}

cv::Mat GlobalMap::getValidMap()
{
	return (global_map==global_map); // non nan values are true
}
