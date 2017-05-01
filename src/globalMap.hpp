#ifndef _GLOBALMAP_HPP_
#define _GLOBALMAP_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>       

namespace cartographer
{
    class GlobalMap
    {
        public: 
			GlobalMap();
			void setMapParameters(float size, float resolution, float safet_offset);
			
			void realWorldOrientation(cv::Mat relative_map, cv::Mat relative_mask_map, float yaw);
			void addToWorld(float x, float y, float z);
			
			cv::Mat getGlobalMap();
			cv::Mat getDebugMap();
			cv::Mat getDebugMap2();
			cv::Mat getValidMap();



		private:

			float local_map_size;
			float global_map_size;
			float histogram_resolution; 
			float histogram_cells;
			float safety_offset;
			int insert_rows, insert_cols; // nrows and ncols of the map local mpa being integrated in the global one
			
			cv::Mat global_map;
			cv::Mat rotated_map; // rotated input mapp to add
			cv::Mat rotated_mask_map;
			cv::Mat display_map;
			cv::Mat count_map;
			int heightmap_set;
	};
}

#endif
