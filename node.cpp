
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <map>
#include <sys/stat.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <ros/package.h>

#include <opencv2/opencv.hpp>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <dynamic_reconfigure/server.h>
#include <realsense_camera/RealsenseCameraConfig.h>

#include <libusb-1.0/libusb.h>

int getNumRGBSubscribers()
{
    return realsense_reg_points_pub.getNumSubscribers() + realsense_rgb_image_pub.getNumSubscribers();
}
 
int getNumDepthSubscribers()
{
    int n = realsense_points_pub.getNumSubscribers() + realsense_reg_points_pub.getNumSubscribers() + realsense_depth_image_pub.getNumSubscribers();
#ifdef V4L2_PIX_FMT_INZI
    n += realsense_infrared_image_pub.getNumSubscribers();
#endif
    return n;
}


int main(int argc, char * argv[])
{
	ros::init(argc, argv, "alt_librealsense_camera");
    ros::NodeHandle n;
    ros::NodeHandle private_node_handle_("~");
    image_transport::ImageTransport image_transport(n);

//    private_node_handle_.param("realsense_camera_type", realsense_camera_type, std::string("Intel(R) RealSense(TM) 3D Camer"));
//    private_node_handle_.param("rgb_frame_id", rgb_frame_id, std::string("_rgb_optical_frame"));
//    private_node_handle_.param("rgb_frame_w", rgb_frame_w, 1280);
//    private_node_handle_.param("rgb_frame_h", rgb_frame_h, 720);

	ros::Rate loop_rate(60);
	ros::Rate idle_rate(1);

	while(ros::ok())
	{
		while ((getNumRGBSubscribers() + getNumDepthSubscribers()) == 0 && ros::ok())
		{
		    ros::spinOnce();
		    idle_rate.sleep();
		}
		ros::spinOnce();

        loop_rate.sleep();
    }
	return  0;

    return 0;
}