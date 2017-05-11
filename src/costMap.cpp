#include "costMap.hpp"
#include <iostream>

using namespace std;
using namespace cartographer;

// creator
CostMap::CostMap() 
{
	
}

void CostMap::setMapParameters(float size, float resolution)
{
	cost_map_size = size;
	histogram_resolution = resolution; 
    histogram_cells = floor(cost_map_size / histogram_resolution);
    
	// prepare maps
	cost_map.create(histogram_cells, histogram_cells, CV_32FC1);
	cost_map = cv::Scalar(std::numeric_limits<float>::quiet_NaN());	
}

void CostMap::setObstacleDilation(int kernel_size, int iterations)
{
	dilation_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
											cv::Size(kernel_size,kernel_size)); // round kernel;
	dilation_iterations = iterations;
}

void CostMap::setMapBlurring(int kernel_size, float max_cost)
{
	blur_kernel_size = kernel_size;
	max_blur_cost = max_cost;
}

void CostMap::setAdditionalSlopePenalty(float max_slope, float max_cost)
{
	slope_upper_limit = max_slope;
	max_slope_cost = max_cost/max_slope;
}

void CostMap::setCostConstants(float cost_base, float cost_offset, float cost_max)
{
	this->cost_base = cost_base;
	this->cost_offset = cost_offset;
	this->cost_max = cost_max;
}

// Area around the rover where the cost should be updated
void CostMap::setUpdateArea(float size)
{
	float cost_update_size = size;
	if(cost_update_size > cost_map_size)
		cost_update_size = cost_map_size;
	
	cost_update_cells = (int)(cost_update_size/histogram_resolution);
}

void CostMap::calculateCostMap(cv::Mat slope_map, cv::Mat obstacle_map, cv::Mat slope_thresh_map, cv::Point2f corner_low, cv::Point2f corner_up)
{
		
	// input untraversable parts and dilate them (robot is now point size)
	cv::Mat t1, local_cost;
	cv::bitwise_or(obstacle_map, slope_thresh_map, local_cost);

	cv::dilate(local_cost, local_cost, dilation_kernel, cv::Point( -1, -1 ), 
				dilation_iterations);

	local_cost.copyTo(t1); // save untraversable
	
	// todo find a better solution for this (unseen/occluded cells)	
	// for the moment they are assumed traversable		
	local_cost.setTo(0,local_cost!=local_cost);
	
	// blur untraversable element in map
	cv::blur(local_cost, local_cost, cv::Size(blur_kernel_size,blur_kernel_size), cv::Point(-1,-1), cv::BORDER_DEFAULT);

	// todo verify the type
	cv::multiply(local_cost,max_blur_cost,local_cost,1,CV_32FC1);
	
	// add additional slope penalty
	cv::Mat temp_slope; // todo find a better memory way to do this 
	slope_map.copyTo(temp_slope);
	temp_slope.setTo(0,temp_slope!=temp_slope); // todo similar remark to occulsion above
	temp_slope.setTo(slope_upper_limit,slope_map>slope_upper_limit); // saturate map
	cv::multiply(temp_slope,max_slope_cost,temp_slope);
	cv::add(temp_slope,local_cost,local_cost);

	// add global offset and cast to integer
	cv::add(local_cost,cost_offset+0.5+cost_base,local_cost,local_cost!=0);

	// cast to integer, make sure the highest value is 21 and non traversable is also 21
	local_cost.convertTo(local_cost, CV_8UC1);
	local_cost.setTo((int)cost_max,local_cost>(int)cost_max);
	local_cost.setTo((int)cost_max,t1==1);

	// add back into global cost map
	local_cost.copyTo(cost_map(cv::Rect(corner_low, corner_up)));

}

cv::Mat CostMap::getCostMap()
{
	return cost_map;
}

int CostMap::getCostUpdateAreaCells()
{
	return cost_update_cells;
}

