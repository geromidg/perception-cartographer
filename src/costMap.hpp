#ifndef _COSTMAP_HPP_
#define _COSTMAP_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>       

#include <sys/time.h>              

namespace cartographer
{
    class CostMap
    {
        public: 
			CostMap();
			void setMapParameters(float size, float resolution);
			void setObstacleDilation(float robot_size, int iterations);
			void setMapBlurring(int kernel_size, float max_cost);
			void setAdditionalSlopePenalty(float max_slope, float max_cost);
			void setCostConstants(float cost_base, float cost_offset, float cost_max);
			void setUpdateArea(float size);

			void calculateCostMap(cv::Mat slope_map, cv::Mat obstacle_map, cv::Mat slope_thresh_map, cv::Point2f corner_low, cv::Point2f corner_up);
			cv::Mat getCostMap();
			int getCostUpdateAreaCells();




		private:

			cv::Mat cost_map;
			float cost_map_size;
			float histogram_resolution; 
			float histogram_cells;
			int cost_update_cells;
			
			cv::Mat dilation_kernel;
			int dilation_iterations;
			
			float max_blur_cost;
			int blur_kernel_size;
			
			float slope_upper_limit;
			float max_slope_cost;
			
			float cost_base, cost_offset, cost_max;

	};
}

#endif
