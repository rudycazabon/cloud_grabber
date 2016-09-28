#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <librealsense/rs.hpp>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#include <iostream>
#include <chrono>
#include <thread>
#include <memory>

#define NOISY 3.5			// Remove points past NOISY meters 
#define FPS_MILLI 500		// Update fps every 0.5 seconds

// time typdefs for fps
typedef std::chrono::milliseconds t_milli;
typedef std::chrono::duration<double, std::milli> t_diff;
typedef std::chrono::high_resolution_clock::time_point t_point;

void printUsage (const char* progName);

// ==== PCL f()'s ====
int parseFlow(int argc, char** argv, bool &realsense);

#define NONE
#ifdef BOOST
boost::shared_ptr<pcl::visualization::PCLVisualizer> getViewer(bool realsense, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr rs_cloud_ptr);
boost::shared_ptr<pcl::visualization::PCLVisualizer> rsVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

#else

std::shared_ptr<pcl::visualization::PCLVisualizer> rsVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
	// Open 3D viewer and add point cloud
	std::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("librealsense Viewer"));
	viewer->setBackgroundColor (0.251, 0.251, 0.251); // Floral white 1, 0.98, 0.94 | Misty Rose 1, 0.912, 0.9 | 
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	return (viewer);
}


std::shared_ptr<pcl::visualization::PCLVisualizer> getViewer(bool realsense, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr rs_cloud_ptr) {
	std::shared_ptr<pcl::visualization::PCLVisualizer> v;
	
	if(realsense) {
		v = rsVis(rs_cloud_ptr);
	}
	
	return v;	
}

#endif

// ==== RealSense f()'s ====
int ctxInfo(rs::context *c);
void logRS();
int configStreams(rs::device *dev);

// ==== Looping f()'s
t_point myClock();
int printTimeLoop(t_point &t0, t_point &t1, t_point &t2,int &frames, int totalframes, double &fps, double &totalfps, std::shared_ptr<pcl::visualization::PCLVisualizer> v);
int getFrame(rs::device *dev, pcl::PointCloud<pcl::PointXYZRGB>::Ptr);

// ==== Main ====
int 
main (int argc, char** argv)
{
	// ==== Parse Command Line Arguments ====
	// Check for help first
	if (pcl::console::find_argument (argc, argv, "-h") >= 0) 
	{
		printUsage (argv[0]);
		return -1;
	}
	
	bool realsense = false;
	int err = -1;
	err = parseFlow(argc, argv, realsense);
	if(err != EXIT_SUCCESS) 
	{
		std::cout << "Error in parseFlow\n" << std::endl;
		return 0;
	}
	
	// ==== Cloud Setup ====
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rs_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

	#ifdef BOOST
	// ==== Viewer Setup ====
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = getViewer(realsense, rs_cloud_ptr);

	#else
	std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = getViewer(realsense, rs_cloud_ptr);

	#endif

	
	// ==== RealSense Stream Setup ====
	logRS();
	rs::context ctx;
    ctxInfo (&ctx);
    // For single camera
    rs::device * dev = ctx.get_device(0);
    configStreams(dev);
	
	// ==== PROGRAM ANALYTICS SETUP ====
	// Frame Numbers
	int frames = 0, totalframes = 0;
	// FPS
	double fps = 0, totalfps = 0;
	// Time Points
	t_point t0, t1, t2;
	t0 = myClock();
	t1 = t2 = t0;
	
	// ==== Viewer Loop ====
	int waitSpin = 0, waitBoost = 1; // REMOVE - Set these from cmd line eventually
	while (!viewer->wasStopped ())
	{
		// ==== Timing ====
		++frames;
		++totalframes;
		err = printTimeLoop(t0, t1, t2, frames, totalframes, fps, totalfps, viewer);
		if(err != EXIT_SUCCESS) 
		{
			std::cout << "Error in printTimeLoop()\n" << std::endl;
			return err;
		}
		
		// ==== Data Grab ====
		err = getFrame(dev, rs_cloud_ptr);
		if(err != EXIT_SUCCESS) 
		{
			std::cout << "Error in getFrame()\n" << std::endl;
			return err;
		}
		
		// ==== Update Viewer Cloud State ====
		viewer->updatePointCloud(rs_cloud_ptr, "sample cloud");
		// ==== Viewer Display ==== 
		viewer->spinOnce ();
		boost::this_thread::sleep (boost::posix_time::microseconds (waitBoost));
	}
	viewer->close();
	return EXIT_SUCCESS;
}

// calculates time stats and prints them on the viewer
int printTimeLoop(t_point &t0, t_point &t1, t_point &t2, int &frames, int totalframes, double &fps, double &totalfps, std::shared_ptr<pcl::visualization::PCLVisualizer> v) {
	t_milli zero_ms{0};
	t_diff fp_ms, overall;
	fp_ms = overall = zero_ms;
	
	t2 = myClock();
	fp_ms = (t2 - t1);
	if(fp_ms.count() > FPS_MILLI) {
		fps  = frames / fp_ms.count();
		fps *= 1000.0;
		frames = 0;
		t1 = t2;
	}
	overall = (t2 - t0);
	totalfps  = overall.count();
	totalfps /= 1000.0;
	
	if((totalframes % 10) == 0) {
		char time_buffer[8], fps_buffer[8];
		
		std::stringstream ss;
		sprintf(time_buffer, "%4.2f", totalfps);
		sprintf(fps_buffer, "%4.2f", fps);
		
		ss << "FPS: " << fps_buffer << "  Frames: " << totalframes << "  Time: " << time_buffer;
		v->removeShape("text", 0);
		v->addText(ss.str(), 200, 10, "text");
	}
	
	return EXIT_SUCCESS;
}

///
/// Get the raw data and add it to the cloud
///
int getFrame(rs::device *dev, pcl::PointCloud<pcl::PointXYZRGB>::Ptr rs_cloud_ptr) {
	
	// Wait for new frame data
	if(dev->is_streaming()) dev->wait_for_frames();
	
	// Retrieve our images
	const uint16_t * depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);
	const uint8_t  * color_image = (const uint8_t  *)dev->get_frame_data(rs::stream::color); 
	
	// Retrieve camera parameters for mapping between depth and color
	rs::intrinsics depth_intrin   = dev->get_stream_intrinsics(rs::stream::depth);
	rs::extrinsics depth_to_color = dev->get_extrinsics(rs::stream::depth, rs::stream::color); 
	rs::intrinsics color_intrin   = dev->get_stream_intrinsics(rs::stream::color); 
	float scale 				  = dev->get_depth_scale();
	
	// Depth dimension helpers
	int dw = 0, dh = 0, dwh = 0;
	dw = depth_intrin.width;
	dh = depth_intrin.height;
	dwh = dw * dh;
	
	// Set the cloud up to be used 
	rs_cloud_ptr->clear();
	rs_cloud_ptr->is_dense = false;
	rs_cloud_ptr->resize(dwh);
	
	for(int dy=0; dy<dh; dy++) {
		for(int dx=0; dx<dw; dx++) {
			uint i = dy * dw + dx;
			uint16_t depth_value = depth_image[i];
			if(depth_value == 0) continue;
			
			rs::float2 depth_pixel = {(float)dx, (float)dy};
			float depth_in_meters = depth_value * scale;
			
			rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
			rs::float3 color_point = depth_to_color.transform(depth_point); 
			rs::float2 color_pixel = color_intrin.project(color_point); 
			
			const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y); 
			static const float nan = std::numeric_limits<float>::quiet_NaN();
			
			// Set up logic to remove bad points
			bool depth_fail = true, color_fail = true;
			depth_fail = (depth_point.z > NOISY);
			color_fail = (cx < 0 || cy < 0 || cx > color_intrin.width || cy > color_intrin.height);
			
			// ==== Cloud Input Pointers ====
			// XYZ input access to cloud
			float *dp_x, *dp_y, *dp_z;
			dp_x = &(rs_cloud_ptr->points[i].x);
			dp_y = &(rs_cloud_ptr->points[i].y);
			dp_z = &(rs_cloud_ptr->points[i].z);
			// RGB input access to cloud
			uint8_t *cp_r, *cp_g, *cp_b;
			cp_r = &(rs_cloud_ptr->points[i].r);
			cp_g = &(rs_cloud_ptr->points[i].g);
			cp_b = &(rs_cloud_ptr->points[i].b);
			
			// ==== Cloud Input Data ====
			// Set up depth point data
			float real_x=0, real_y=0, real_z=0, adjusted_x=0, adjusted_y=0, adjusted_z=0;
			real_x = depth_point.x;
			real_y = depth_point.y;
			real_z = depth_point.z;
			// Adjust point to coordinates
			adjusted_x = -1 * real_x;
			adjusted_y = -1 * real_y;
			adjusted_z = real_z;
			
			// Set up color point data
			const uint8_t *offset = (color_image + (cy * color_intrin.width + cx) * 3);
			uint8_t raw_r=0, raw_g=0, raw_b=0, adjusted_r=0, adjusted_g=0, adjusted_b=0;
			raw_r = *(offset);
			raw_g = *(offset + 1);
			raw_b = *(offset + 2);
			// Adjust color arbitrarily
			adjusted_r = raw_r; 
			adjusted_g = raw_g; 
			adjusted_b = raw_b; 
			
			// ==== Cloud Point Evaluation ====
			// If bad point, remove & skip
			if(depth_fail || color_fail) 
			{
				*dp_x = *dp_y = *dp_z = (float) nan;
				*cp_r = *cp_g = *cp_b = 0;
				continue;
			} 
			// If valid point, add data to cloud
			else 
			{
				// Fill in cloud depth
				*dp_x = adjusted_x;
				*dp_y = adjusted_y;
				*dp_z = adjusted_z;
				// Fill in cloud color
				*cp_r = adjusted_r;
				*cp_g = adjusted_g;
				*cp_b = adjusted_b;
			}
		}
	}
	
	return EXIT_SUCCESS;
}

#ifdef BOOST
// If we have different view scenarios
boost::shared_ptr<pcl::visualization::PCLVisualizer> getViewer(bool realsense, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr rs_cloud_ptr) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> v;
	
	if(realsense) {
		v = rsVis(rs_cloud_ptr);
	}
	
	return v;
}
#endif

// set flow vars based on command line options given
int parseFlow(int argc, char** argv, bool &realsense) {
	if (pcl::console::find_argument (argc, argv, "-r") >= 0)
	{
		realsense = true;
		std::cout << "RealSense cloud example\n";
	}
	else
	{
		printUsage (argv[0]);
		return -1;
	}
	
	return EXIT_SUCCESS;
}

void
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-r           RealSense cloud example\n"
            << "\n\n";
}

#ifdef BOOST
// RealSense Viewer settings
boost::shared_ptr<pcl::visualization::PCLVisualizer> rsVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // Open 3D viewer and add point cloud
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("librealsense Viewer"));
  viewer->setBackgroundColor (0.251, 0.251, 0.251); // Floral white 1, 0.98, 0.94 | Misty Rose 1, 0.912, 0.9 | 
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}
#endif 

// logging options, output to file, etc
void logRS() {
    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");
}

// provides context info and will exit if no devices
int ctxInfo(rs::context *c) {
	printf("There are %d connected RealSense devices.\n", c->get_device_count());
    if(c->get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");
    
    return EXIT_SUCCESS;
}

// stream config & enabling for device
int configStreams(rs::device *dev) {
	//printf("\nUsing device 0, an %s\n    Serial number: %s\n    Firmware version: %s\n", dev->get_name(), dev->get_serial(), dev->get_firmware_version());
	// REMOVE - should we use color to depth stream?
	std::cout << "Starting " << dev->get_name() << "... ";
    dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
    dev->enable_stream(rs::stream::color, rs::preset::best_quality);
    dev->start();
    std::cout << "done.\n";
    
    return EXIT_SUCCESS;
}

t_point myClock() {
	return std::chrono::high_resolution_clock::now();
}
