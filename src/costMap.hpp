#ifndef _COSTMAP_HPP_
#define _COSTMAP_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>       

namespace cartographer
{
    class CostMap
    {
        public: 
			CostMap();
			void setObstacleDilation(int kernel_size, int iterations);
			void setMapBlurring(int kernel_size, float max_cost);
			void setAdditionalSlopePenalty(float max_slope, float max_cost);
			void setCostConstants(float cost_base, float cost_offset, float cost_max);


			void calculateCostMap(cv::Mat slope_map, cv::Mat obstacle_map, cv::Mat slope_thresh_map);
			cv::Mat getCostMap();




		private:

			cv::Mat cost_map;
			
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
