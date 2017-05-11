#include "localMap.hpp"
#include <iostream>


using namespace std;
using namespace cartographer;

// creator
LocalMap::LocalMap() 
   // : gaussian_kernel(0), calibrationInitialized( false )
{
	// configure maps
	camera_set = 0;
	heightmap_set = 0;

	cloud_input_p.reset( new pcl::PointCloud<pcl::PointXYZ> );
	cloud_filtered_p.reset( new pcl::PointCloud<pcl::PointXYZ> );
}


void LocalMap::welcome()
{
    cout << "Welcome!" << endl;
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

void LocalMap::setPcFiltersParameters(float leaf_size, int k_points)
{
	filter_leaf_size = leaf_size;
	filter_k_points = k_points;
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

void LocalMap::setPointCloud(std::vector<float> pc_vector)
{
	; // TODO implement
}

void LocalMap::pointColudFiltering()
{   
	struct timeval tp;
	
	long int ms[4]; 
	gettimeofday(&tp, NULL);
	ms[0] = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds

	// remove out of bound points
	pcl::CropBox<pcl::PointXYZ> cb;
	cb.setInputCloud(cloud_input_p);
	cb.setMin(filter_box_min);
	cb.setMax(filter_box_max);
	cb.filter(*cloud_filtered_p);

	//gettimeofday(&tp, NULL);
	//ms[1] = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds

	// Apply filter to reduce size of the point cloud by averaging
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_filtered_p);
    vg.setLeafSize(filter_leaf_size, filter_leaf_size, filter_leaf_size);
    vg.filter(*cloud_filtered_p);

	//gettimeofday(&tp, NULL);
	//ms[2] = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds

    // Apply statistical filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_filtered_p);
    sor.setMeanK(filter_k_points);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered_p);
    
    //gettimeofday(&tp, NULL);
	//ms[3] = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds
	
	//for(int iii = 1; iii<4; iii++)
	//			std::cout << "time at pc filter " << iii << ": " << (ms[iii]-ms[iii-1]) << std::endl;

}

// todo modify this function so that it can accept a random pc too
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

void LocalMap::setMapParameters(float size, float resolution, int scale)
{
	local_map_size = size;
	histogram_resolution = resolution; 
    histogram_cells = floor(local_map_size / histogram_resolution);

	// prepare height map
	height_map.create(histogram_cells, histogram_cells, CV_32FC1);
	height_map_mask.create(histogram_cells, histogram_cells, CV_8UC1);
	
	heightmap_set = 1;
	
	slope_map_scale = scale;
}

void LocalMap::setObstacleLaplacian(int kernel_size, float threshold)
{
	laplacian_kernel_size = kernel_size;
	laplacian_threshold = threshold;
}

void LocalMap::setObstacleDetection(int kernel_size_o, int iteration_o,
									int kernel_size_s, int iteration_s)
{
	obstacle_kernel_size = kernel_size_o;
	obstacle_iterations = iteration_o;
	obstacle_vicinity_kernel_size = kernel_size_s;
	obstacle_vicinity_iterations = iteration_s;
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
        {
            continue;
        }

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
    
    height_map.copyTo(height_map_interpolated); // TODO unsure this is good idea..
    
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
}

void LocalMap::heightMap2SlopeMap()
{
	float sample_scale = 1.0/slope_map_scale;
	cv::Mat interpSlope;
	
	// Scale interpolated heightMap and mask
	cv::resize(height_map_mask, height_map_mask_scaled, cv::Size(), sample_scale, sample_scale, cv::INTER_NEAREST);
	cv::resize(height_map_interpolated, interpSlope, cv::Size(), sample_scale, sample_scale, cv::INTER_LINEAR);

	// Find gradient along x and y directions
	cv::Mat height_map_gradient_x, height_map_gradient_y;
	cv::Sobel(interpSlope, height_map_gradient_x, CV_32FC1, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
	cv::Sobel(interpSlope, height_map_gradient_y, CV_32FC1, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);

	// Square the x and y gradient matrices, this makes them positive, needed to obtain absolute slope value
	cv::pow(height_map_gradient_x, 2, height_map_gradient_x);
	cv::pow(height_map_gradient_y, 2, height_map_gradient_y);
	
	// Add squared slope of x and y together
	cv::add(height_map_gradient_x, height_map_gradient_y, interpSlope);
	
	// Take the square root to get the real slope
	cv::sqrt(interpSlope, interpSlope);

	// Apply the mask to extract only the valid data TODO also resize already?
	interpSlope.copyTo(slope_map, height_map_mask_scaled);
}

void LocalMap::thresholdSlopeMap(float slope_threshold)
{
	// Threshold the slope map to tag slopes that are not traversable
	// todo unterstand why it is slope_map_scale^2 (probably because scaled in two directions)
	float threshold = slope_threshold*slope_map_scale*slope_map_scale*histogram_resolution; // tan(st)=t/(sms*sms*hr)
	cv::resize(slope_map, slope_map, cv::Size(height_map.cols, height_map.rows), 0 , 0, cv::INTER_LINEAR);
    cv::threshold(slope_map, slope_map_thresholded, threshold, 1.0f, cv::THRESH_BINARY);
}

void LocalMap::detectObstacles(float height_threshold)
{
	cv::Mat interpLaplacian;
	
    // Find the obstacles with a Laplace filter (gradients)
    cv::Laplacian(height_map_interpolated, interpLaplacian, CV_32FC1, laplacian_kernel_size);

    // Mask the laplacian to remove invalid interpolated data
    height_map_laplacian.setTo(0);
    interpLaplacian.copyTo(height_map_laplacian, height_map_mask);

    // The obstacle map is based on the laplacien image and a threshold defines probably obstacles
    cv::threshold(-height_map_laplacian, height_map_laplacian_thresholded, laplacian_threshold, 1.0f, cv::THRESH_BINARY);	// TODO sure it is so? -laplacian or abs(laplacian)[

	// find contours, dilate obstacles a bit first so that all or
	// slightly more of it is included in the contour
	cv::Mat contour_mask; 
	cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                       cv::Size( obstacle_kernel_size,obstacle_kernel_size )); // square kernel
                                       
	height_map_laplacian_thresholded.convertTo(contour_mask, CV_8UC1);
	cv::dilate(contour_mask, contour_mask, element, cv::Point( -1, -1 ), 
				obstacle_iterations);
	std::vector < std::vector<cv::Point> > contours;
    findContours(contour_mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	// prepare variables used to iterate through the single obstacles 
	cv::Mat single_contour_mask; 
    cv::Mat single_surround_mask; 
	cv::Mat combined_mask; 
	cv::Mat obstacle_mask;
	contour_mask.copyTo(single_contour_mask); // todo unsure if 4 lines  useful
	contour_mask.copyTo(single_surround_mask);
	contour_mask.copyTo(combined_mask);
	contour_mask.copyTo(obstacle_mask);
	obstacle_mask.setTo(cv::Scalar(0));
	obstacle_map.setTo(cv::Scalar(0));
	
    int nContours = contours.size();
	double maxVal[nContours];
	double minVal[nContours]; // todo set up so that holes can be found out too
	double inside_mean[nContours];
	double outside_mean[nContours];
	int obstacle_check[nContours];
	element = cv::getStructuringElement( cv::MORPH_RECT,
                                       cv::Size( obstacle_vicinity_kernel_size,obstacle_vicinity_kernel_size )); // square kernel        
	
	// Iterate through the obstacles:
	// - dilate and mask to find the area around obstacles
	// - compare height of obstacles points to immediate surroundings
    for( int iContour = 0; iContour < nContours; iContour++)
    {
		single_contour_mask.setTo(cv::Scalar(0)); // initialize mask
		single_surround_mask.setTo(cv::Scalar(0));
		cv::drawContours(single_contour_mask, contours, iContour, cv::Scalar(1), CV_FILLED); // fill mask with only one contour
		cv::dilate(single_contour_mask, single_surround_mask, element, cv::Point( -1, -1 ) , obstacle_vicinity_iterations); 
		cv::bitwise_xor(single_contour_mask,single_surround_mask, single_surround_mask); // keep original mask
		cv::bitwise_and(height_map_mask,single_contour_mask, single_contour_mask); // remove regions where we do not have heightmap data
		cv::bitwise_and(height_map_mask,single_surround_mask, single_surround_mask); 
		inside_mean[iContour] = cv::mean(height_map,single_contour_mask)[0];
		outside_mean[iContour] = cv::mean(height_map,single_surround_mask)[0];
		cv::minMaxLoc(height_map, &minVal[iContour], &maxVal[iContour], NULL, NULL, single_contour_mask); // TODO check again because now peak relies on only one point (should be like top 3 or 5 points)
		if((maxVal[iContour] - outside_mean[iContour]) > height_threshold) // this is indeed an obstacle
		{
			obstacle_check[iContour] = 1;
			cv::bitwise_or(single_contour_mask, obstacle_mask, obstacle_mask); // add obstacle to total obstacle mask
		}	
		else
			obstacle_check[iContour] = 0;
	}		
	obstacle_mask.convertTo(obstacle_map, CV_32FC1);
}

void LocalMap::computeTraversability()
{
	// combine two traversability if feasible
	cv::Mat t1, t2;
	obstacle_map.convertTo(t1, CV_8UC1); // to do decide what to do with map types *masks should be all 8UC1?
	slope_map_thresholded.convertTo(t2, CV_8UC1);
	cv::multiply(t2, 255, t2); // multiplication because of previous comment
	cv::bitwise_or(t1, t2, traversability_map);
	traversability_map.convertTo(traversability_map, CV_32FC1);
}


cv::Mat LocalMap::getHeightMap()
{
	return height_map;
}

cv::Mat LocalMap::getHeightMapInterp()
{
	return height_map_interpolated;
}

cv::Mat LocalMap::getSlopeMap()
{
	return slope_map;
}

cv::Mat LocalMap::getSlopeMapThresholded()
{
	return slope_map_thresholded;
}

cv::Mat LocalMap::getLaplacian()
{
	return height_map_laplacian;
}

cv::Mat LocalMap::getLaplacianThresholded()
{
	return height_map_laplacian_thresholded;
}

cv::Mat LocalMap::getObstacles()
{
	return obstacle_map;
}

cv::Mat LocalMap::getTraversability()
{
	return traversability_map;
}

cv::Mat LocalMap::getMask()
{
	return height_map_mask;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LocalMap::getPointCloud()
{
	return cloud_input_p;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LocalMap::getPointCloudFiltered()
{
	return cloud_filtered_p;
}
