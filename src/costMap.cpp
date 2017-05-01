#include "costMap.hpp"
#include <iostream>

using namespace std;
using namespace cartographer;

// creator
CostMap::CostMap() 
{
	
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

void CostMap::calculateCostMap(cv::Mat slope_map, cv::Mat obstacle_map, cv::Mat slope_thresh_map)
{
	// input untraversable parts and dilate them (robot is now point size)
	cv::Mat t1;// todo find better memory solution
	cv::bitwise_or(obstacle_map, slope_thresh_map, cost_map);
	cv::dilate(cost_map, cost_map, dilation_kernel, cv::Point( -1, -1 ), 
				dilation_iterations);
	cost_map.copyTo(t1);
				
	// todo find a better solution for this (unseen/occluded cells)	
	// for the moment they are assumed traversable		
	cost_map.setTo(0,cost_map!=cost_map);
	
	// blur untraversable element in map
	cv::blur(cost_map, cost_map, cv::Size(blur_kernel_size,blur_kernel_size), cv::Point(-1,-1), cv::BORDER_DEFAULT);

	// todo verify the type
	cv::multiply(cost_map,max_blur_cost,cost_map,1,CV_32FC1);
	
	// add additional slope penalty
	cv::Mat temp_slope; // todo find a better memory way to do this 
	slope_map.copyTo(temp_slope);
	temp_slope.setTo(0,temp_slope!=temp_slope); // todo similar remark to occulsion above
	temp_slope.setTo(slope_upper_limit,slope_map>slope_upper_limit); // saturate map
	cv::multiply(temp_slope,max_slope_cost,temp_slope);
	cv::add(temp_slope,cost_map,cost_map);
	
	// add global offset and cast to integer
	cv::add(cost_map,cost_offset+0.5+cost_base,cost_map,cost_map!=0);

	
	// cast to integer, make sure the highest value is 21 and non traversable is also 21
	cost_map.convertTo(cost_map, CV_8UC1);
	cost_map.setTo((int)cost_max,cost_map>(int)cost_max);
	cost_map.setTo((int)cost_max,t1==1);

}

cv::Mat CostMap::getCostMap()
{
	return cost_map;
}

