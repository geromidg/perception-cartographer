#include "localMap.hpp"

using namespace std;
using namespace cartographer;

LocalMap::LocalMap()
{
    camera_set = 0;
    heightmap_set = 0;

    cloud_input_p.reset( new pcl::PointCloud<pcl::PointXYZ> );
    cloud_filtered_p.reset( new pcl::PointCloud<pcl::PointXYZ> );
}

void LocalMap::setCameraParameters(int width, int height, float cx, float cy, float fx, float fy)
{
    this->width = width;
    this->height = height;
    this->cx = cx;
    this->cy = cy;
    this->fx = fx;
    this->fy = fy;

    // reserve space
    cloud_input_p->width    = width;
    cloud_input_p->height   = height;
    cloud_input_p->is_dense = false;
    cloud_input_p->points.resize (width*height);

    xArray_const.reserve(height*width);
    yArray_const.reserve(height*width);
    xArray.reserve(height*width);
    yArray.reserve(height*width);

    // fill x and y array
    int ix = 1;
    for(int i = 1; i<=height; i++)
    {
        for(int j = 1; j<=width; j++)
        {
            yArray_const.push_back(ix);
            xArray_const.push_back(j);
        }
        ix++;
    }

    // prepare x and y arrays
    transform(xArray_const.begin(), xArray_const.end(), xArray_const.begin(),
            bind1st(std::plus<float>(), -cx));
    transform(yArray_const.begin(), yArray_const.end(), yArray_const.begin(),
            bind1st(std::plus<float>(), -cy));

    transform(xArray_const.begin(), xArray_const.end(), xArray_const.begin(),
            bind1st(std::multiplies<float>(), 1/fx));
    transform(yArray_const.begin(), yArray_const.end(), yArray_const.begin(),
            bind1st(std::multiplies<float>(), 1/fy));

    camera_set = 1;
}

void LocalMap::setMapParameters(float size, float resolution)
{
    local_map_size = size;
    histogram_resolution = resolution;
    histogram_cells = floor(local_map_size / histogram_resolution);

    // prepare height map
    height_map.create(histogram_cells, histogram_cells, CV_32FC1);
    height_map_mask.create(histogram_cells, histogram_cells, CV_8UC1);

    heightmap_set = 1;
}

void LocalMap::setPcFiltersParameters(float leaf_size, int k_points, bool use_statistical_filter)
{
    filter_leaf_size = leaf_size;
    filter_k_points = k_points;
    this->use_statistical_filter = use_statistical_filter;
}

void LocalMap::setPcLimitsParameters(Eigen::Vector4f min, Eigen::Vector4f max)
{
    filter_box_min = min;
    filter_box_max = max;
}

void LocalMap::distance2pointCloud(std::vector<float> distance) // todo specify that it is not in camera frame!
{
    if(!camera_set)
        std::cerr << "The camera properties have not been set yet!\n";

    // reserve space tbd is this needed here??
    cloud_input_p->width    = width;
    cloud_input_p->height   = height;
    cloud_input_p->is_dense = false;
    cloud_input_p->points.resize (width*height);

    std::transform( xArray_const.begin(), xArray_const.end(),
            distance.begin(), xArray.begin(),
            std::multiplies<float>() );

    std::transform( yArray_const.begin(), yArray_const.end(),
            distance.begin(), yArray.begin(),
            std::multiplies<float>() );

    for(int i = 0; i<cloud_input_p->size(); i++)
    {
        cloud_input_p->points[i].y = -xArray[i];
        cloud_input_p->points[i].z = -yArray[i];
        cloud_input_p->points[i].x = distance[i];
    }

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_input_p,*cloud_input_p, indices);
}

void LocalMap::pointColudFiltering()
{
    double width = cloud_input_p->width;
    double height = cloud_input_p->height;
    cloud_filtered_p->width    = width;
    cloud_filtered_p->height   = height;
    cloud_filtered_p->points.resize (width*height);

    // remove out of bound points
    pcl::CropBox<pcl::PointXYZ> cb;
    cb.setInputCloud(cloud_input_p);
    cb.setMin(filter_box_min);
    cb.setMax(filter_box_max);
    cb.filter(*cloud_filtered_p);

    // Apply filter to reduce size of the point cloud by averaging
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_filtered_p);
    vg.setLeafSize(filter_leaf_size, filter_leaf_size, filter_leaf_size);
    vg.filter(*cloud_filtered_p);

    // Apply statistical filter
    if(use_statistical_filter)
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_filtered_p);
        sor.setMeanK(filter_k_points);
        sor.setStddevMulThresh(1.0);
        sor.filter(*cloud_filtered_p);
    }
}

void LocalMap::pointCloud2flatRobotReference(Eigen::Quaterniond imu_orientation,
        Eigen::Vector3d camera_to_ptu,
        Eigen::Quaterniond ptu_orientation,
        Eigen::Vector3d ptu_to_body)
{
    Eigen::Translation<double,3> cam2ptu(camera_to_ptu);
    Eigen::Translation<double,3> ptu2Bd(ptu_to_body);

    Eigen::Transform<double,3,Eigen::Affine> combined = imu_orientation * ptu2Bd * ptu_orientation * cam2ptu;

    // apply transfrom
    pcl::transformPointCloud(*cloud_filtered_p, *cloud_filtered_p, combined);
}

void LocalMap::pointCloud2heightMap()
{
    if(!heightmap_set)
        std::cerr << "The camera properties have not been set yet!\n";

    float local_map_size_half = local_map_size / 2.0;

    // Create a integer vector storing the number of points inside a bin
    // in order to apply a running average
    std::vector<std::vector<int> > height_map_bin_count(histogram_cells,
            std::vector<int>(histogram_cells, 0));

    // initialize height map
    height_map = cv::Scalar(std::numeric_limits<float>::quiet_NaN());

    pcl::PointCloud<pcl::PointXYZ>::iterator iPCL;
    float x, y, z, bin_count;
    int16_t index_x, index_y;

    // Iterate over all points in the cloud and store them in the histogram
    for(iPCL = cloud_filtered_p->points.begin(); iPCL < cloud_filtered_p->points.end(); iPCL++)
    {
        // Store point in the histogram
        // Permutate y and z to have a more consistent reference system
        x = iPCL->x;
        y = iPCL->y;
        z = iPCL->z;

        // find where the points fits in the histogram
        // Reverse y as the image 0 coordinate to up left instead of down left
        index_x = floor((x) / histogram_resolution);
        index_y = floor((y+local_map_size_half)/ histogram_resolution);

        // discount the point if it doesnt fit inside the histogram map
        // this is extra safety. Should be related to how the pc is cut
        // in a previous function to remove far out values
        if(index_x < 0 || index_x > histogram_cells - 1 || index_y < 0 || index_y > histogram_cells - 1)
            continue;

        // Evaluate a running average as there might be more than one point per pixel
        bin_count = (float)height_map_bin_count[index_x][index_y];
        if(!bin_count) // first time we register a point in this position
            height_map.at<float>(index_x, index_y) = z;
        else // running average
            height_map.at<float>(index_x, index_y) = (height_map.at<float>(index_x, index_y) * bin_count + z) / (bin_count + 1.0f);

        height_map_bin_count[index_x][index_y]++;
    }

    // AD HOC
    // The disparity has a tendency to create weird edges on the left side,
    // so remove 2 pixels on the right side of this projection for each row
    for(int nRow = 0; nRow < height_map.rows; nRow++)
    {
        for(int nCol = height_map.cols-1; nCol >= 0; nCol--)
        {
            if(height_map.at<float>(nRow, nCol) == height_map.at<float>(nRow, nCol)) // that means it isnt nan
            {
                height_map.at<float>(nRow, nCol) = std::numeric_limits<float>::quiet_NaN();
                height_map.at<float>(nRow, nCol-1) = std::numeric_limits<float>::quiet_NaN();
                break;
            }
        }
    }

    // keep original mask
    height_map_mask = cv::Mat(height_map == height_map);
    // replace nan values with 0 *todo check if necessary)
    height_map.setTo(0.0f, height_map != height_map);
}

void LocalMap::heightMapInterpolate()
{
    // Do linear interpolation with the matrix in the y direction
    // TODO perhaps it is more justified to do it in polar coordinates with the center being at the stereo camera position
    // TODO find a way to do dilation using nearest neighbor instead of this approach which presents boundary issues
    int16_t column, row, p, start_index, end_index;
    float value, value_previous, fraction, start_value, end_value;

    height_map.copyTo(height_map_interpolated); // TODO unsure this is good idea

    for(column = 0; column < height_map.cols; column++)
    {
        value_previous = 0.0f;
        start_index = 0;
        end_index = 0;

        for(row = 0; row < height_map.rows; row++)
        {
            // Get the pixel value in the matrix
            value = height_map_interpolated.at<float>(row, column);

            if(row == 0 && value == 0.0f)
            {
                // First value in row is missing, assign 0.0f for start
                start_index = 0;
                start_value = 0.0f;
            }
            else if(start_index == -1 && value == 0.0f && value_previous != 0.0f)
            {
                // Start of the missing data is the previous cell
                start_index = row - 1;
                start_value = value_previous;
            }
            else if((value != 0.0f && value_previous == 0.0f) || (value == 0.0f && row == height_map.rows - 1 && start_index != -1))
            {
                // End of the missing data
                end_index = row;
                end_value = value;

                // Interpolate
                for(p = start_index; p <= end_index; p++)
                {
                    // Evaluate the linear interpolation
                    fraction = (float)(p - start_index) / (float)(end_index - start_index);
                    height_map_interpolated.at<float>(p, column) = (start_value * (1.0f - fraction) + end_value * fraction);
                }
                start_index = -1;
                end_index = -1;
            }

            // Save the values for next iteration
            value_previous = value;
        }
    }

    //same along the other direction to prevent approximation errors when the image is taken with a ptu
    for(row = 0; row < height_map.rows; row++)
    {
        value_previous = 0.0f;
        start_index = 0;
        end_index = 0;

        for(column = 0; column < height_map.cols; column++)
        {
            // Get the pixel value in the matrix
            value = height_map_interpolated.at<float>(row, column);

            if(column == 0 && value == 0.0f)
            {
                // First value in row is missing, assign 0.0f for start
                start_index = 0;
                start_value = 0.0f;
            }
            else if(start_index == -1 && value == 0.0f && value_previous != 0.0f)
            {
                // Start of the missing data is the previous cell
                start_index = column - 1;
                start_value = value_previous;
            }
            else if((value != 0.0f && value_previous == 0.0f) || (value == 0.0f && column == height_map.cols - 1 && start_index != -1))
            {
                // End of the missing data
                end_index = column;
                end_value = value;

                // Interpolate
                for(p = start_index; p <= end_index; p++)
                {
                    // Evaluate the linear interpolation
                    fraction = (float)(p - start_index) / (float)(end_index - start_index);
                    height_map_interpolated.at<float>(row, p) = (start_value * (1.0f - fraction) + end_value * fraction);
                }
                start_index = -1;
                end_index = -1;
            }

            // Save the values for next iteration
            value_previous = value;
        }
    }
}

cv::Mat LocalMap::getHeightMap()
{
    return height_map;
}

cv::Mat LocalMap::getHeightMapInterp()
{
    return height_map_interpolated;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LocalMap::getPointCloud()
{
    return cloud_input_p;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LocalMap::getPointCloudFiltered()
{
    return cloud_filtered_p;
}

