#ifndef _LOCALMAP_HPP_
#define _LOCALMAP_HPP_

//#include <opencv2/opencv.hpp>


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>

#include <sys/time.h>              


namespace cartographer
{
    class LocalMap
    {
        public: 
			LocalMap();
            /**
             * Print a welcome to stdout
             * \return nothing
             */
            void welcome();
            
            // parameters setters
            void setCameraParameters(int width, int height, float cx, float cy, float fx, float fy);
            void setMapParameters(float size, float resolution, int scale);
            void setPcFiltersParameters(float leaf_size, int k_points, bool use_statistical_filter);
            void setPcLimitsParameters(Eigen::Vector4f min, Eigen::Vector4f max);
			void setPointCloud(std::vector<float> pc_vector);
            void setObstacleLaplacian(int kernel_size, float threshold);
			void setObstacleDetection(int kernel_size_o, int iteration_o,
									int kernel_size_s, int iteration_s);

            // functionality
            void distance2pointCloud(std::vector<float> distance);
            void pointColudFiltering();
            void pointCloud2flatRobotReference(Eigen::Quaterniond imu_orientation,
											Eigen::Vector3d camera_to_ptu,
											Eigen::Quaterniond ptu_orientation,
											Eigen::Vector3d ptu_to_body);
            void pointCloud2heightMap();
            void heightMapInterpolate();
			void heightMap2SlopeMap();
			void detectObstacles(float height_threshold);
            void thresholdSlopeMap(float slope_threshold);
            void computeTraversability();

			// images/pcl/data getters
            cv::Mat getHeightMap();
			cv::Mat getHeightMapInterp();
			cv::Mat getSlopeMap();
			cv::Mat getSlopeMapThresholded();
			cv::Mat getLaplacian();
			cv::Mat getLaplacianThresholded();
			cv::Mat getObstacles();
			cv::Mat getTraversability();
			cv::Mat getMask();

            pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud();
            pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudFiltered();
            
            
		private:
			
			int camera_set; // has the camera been configured
			int heightmap_set;
			
			// camera parameters
			float width;
			float height;
			float cx;
			float cy;
			float fx;
			float fy;
			
			// arrays used to compute the pointcloud
			std::vector<float> xArray_const, yArray_const, xArray, yArray;
			
			// map parameters
			float local_map_size;
			float histogram_resolution; 
			float histogram_cells;
			int slope_map_scale;

			// pc filter parameters
			float filter_leaf_size;
			int filter_k_points;
			Eigen::Vector4f	filter_box_min, filter_box_max;
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_p;
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input_p;
			bool use_statistical_filter;
			
			// Obstacle laplacian parameters
			int laplacian_kernel_size;
			float laplacian_threshold;
			
			// Obstacle processing parameters
			int obstacle_kernel_size;
			int obstacle_iterations;
			int obstacle_vicinity_kernel_size;
			int obstacle_vicinity_iterations;
			
			cv::Mat height_map; 	// heightmap as converted from pc
			cv::Mat height_map_mask; // heightmap pixels without data mask
			cv::Mat height_map_mask_scaled; // heightmap pixels without data mask scaled to slope map
			cv::Mat height_map_interpolated; 
			cv::Mat height_map_gradient_x; 
			cv::Mat height_map_gradient_y;
			
			cv::Mat slope_map;		// slopemap built from subsampled gradient
			cv::Mat slope_map_mask; // slopemap pixels without data mask
			cv::Mat slope_map_thresholded; // slopemap thresholded 
			
			cv::Mat height_map_laplacian; // laplacian of the heightmap
			cv::Mat height_map_laplacian_thresholded; // laplacian thresholded
			
			cv::Mat obstacle_map; // binary map of unsurmontable obstacles

			cv::Mat traversability_map; // accessible and inaccessible map
			
    };

} // end namespace cartographer

#endif // 
