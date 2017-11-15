#ifndef _LOCALMAP_HPP_
#define _LOCALMAP_HPP_

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

        // parameters setters
        void setCameraParameters(int width, int height, float cx, float cy, float fx, float fy);
        void setMapParameters(float size, float resolution);
        void setPcFiltersParameters(float leaf_size, int k_points, bool use_statistical_filter);
        void setPcLimitsParameters(Eigen::Vector4f min, Eigen::Vector4f max);

        // functionality
        void distance2pointCloud(std::vector<float> distance);
        void pointCloudFiltering();
        void pointCloud2flatRobotReference(Eigen::Quaterniond imu_orientation,
                Eigen::Vector3d camera_to_ptu,
                Eigen::Quaterniond ptu_orientation,
                Eigen::Vector3d ptu_to_body);
        void pointCloud2heightMap();
        void heightMapInterpolate();

        // images/pcl/data getters
        cv::Mat getHeightMap();
        cv::Mat getHeightMapInterp();
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

        // pc filter parameters
        float filter_leaf_size;
        int filter_k_points;
        Eigen::Vector4f	filter_box_min, filter_box_max;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_p;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input_p;
        bool use_statistical_filter;

        cv::Mat height_map; 	// heightmap as converted from pc
        cv::Mat height_map_mask; // heightmap pixels without data mask
        cv::Mat height_map_interpolated;
    };

} // end namespace cartographer

#endif //
