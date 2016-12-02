//
// TODO: generate pointcloud
// TODO: provide depth with correct scaling, currently it is Z16 that is the scaling of 
// rs::format::z16
// alternatively directly use: points
// 
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

#include <librealsense/rs.hpp>


ros::Publisher realsense_reg_points_pub;

image_transport::CameraPublisher realsense_rgb_image_pub;
image_transport::CameraPublisher realsense_ir_image_pub;
image_transport::CameraPublisher realsense_depth_image_pub;

int getNumRGBSubscribers()
{
    return realsense_reg_points_pub.getNumSubscribers() + realsense_rgb_image_pub.getNumSubscribers() + realsense_ir_image_pub.getNumSubscribers();
}
 
int getNumDepthSubscribers()
{
    int n = realsense_reg_points_pub.getNumSubscribers() + realsense_depth_image_pub.getNumSubscribers() + realsense_ir_image_pub.getNumSubscribers();
#ifdef V4L2_PIX_FMT_INZI
    n += realsense_infrared_image_pub.getNumSubscribers();
#endif
    return n;
}

/*
    ci.header.stamp = head_time_stamp;
    ci.header.seq = head_sequence_id;
*/
void rs_intrinsics2camerainfo(sensor_msgs::CameraInfo & ci, rs::intrinsics x, std::string frame_id)
{
	//memset(&ci,0,sizeof(ci));
	 ci.header.frame_id = frame_id;
	 // distortion_model D
    ci.width = x.width;
    ci.height = x.height;
    ci.K[0] = x.fx;
    ci.K[2] = x.ppx;
    ci.K[4] = x.fy;
    ci.K[5] = x.ppy;


}

void mySigintHandler(int sig)
{
  ros::shutdown();
}

int main(int argc, char * argv[])
{
	ros::init(argc, argv, "alt_librealsense_camera");
  std::string topic_points_id = "/points";
  std::string topic_depth_id = "/depth";
  std::string topic_ir_id = "/ir";
  std::string topic_image_rgb_id = "/rgb";
  std::string depth_frame_id = "/sr300_depth_optical_frame";
  std::string rgb_frame_id = "/sr300_rgb_optical_frame";

  signal(SIGINT, mySigintHandler);
    ros::NodeHandle n;
    ros::NodeHandle private_node_handle_("~");
    image_transport::ImageTransport image_transport(n);

//    private_node_handle_.param("realsense_camera_type", realsense_camera_type, std::string("Intel(R) RealSense(TM) 3D Camer"));
//    private_node_handle_.param("rgb_frame_id", rgb_frame_id, std::string("_rgb_optical_frame"));
//    private_node_handle_.param("rgb_frame_w", rgb_frame_w, 1280);
//    private_node_handle_.param("rgb_frame_h", rgb_frame_h, 720);
	rs::log_to_console(rs::log_severity::warn);
	rs::context ctx;
	if (ctx.get_device_count() == 0)
		throw std::runtime_error("No device detected. Is it plugged in?");
	printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
	std::cerr << "ctx\n";
  if (ctx.get_device_count() == 0) return EXIT_FAILURE;

  rs::device *dev = ctx.get_device(0);
  printf("\nUsing device 0, an %s\n", dev->get_name());
  printf("    Serial number: %s\n", dev->get_serial());
  printf("    Firmware version: %s\n", dev->get_firmware_version());

  rs::intrinsics depth_intrin;
  rs::extrinsics depth_to_color;
  rs::intrinsics color_intrin;
  rs::intrinsics ir_intrin;
  memset(&depth_intrin,0,sizeof(depth_intrin));
  memset(&depth_to_color,0,sizeof(depth_to_color));
  memset(&color_intrin,0,sizeof(color_intrin));
  memset(&ir_intrin,0,sizeof(ir_intrin));

  // rs::stream::rectified_color
  // rs::stream::depth_aligned_to_rectified_color
  dev->enable_stream(rs::stream::color,rs::preset::best_quality);
  dev->enable_stream(rs::stream::depth,rs::preset::best_quality);
  dev->enable_stream(rs::stream::infrared,rs::preset::best_quality);
  if(!dev->is_stream_enabled(rs::stream::color))
  {
      std::cerr << "cannot open\n";
      dev->stop();
      return 0;
  }
  else
  {
    auto &intrin  = color_intrin;
    auto stream = rs::stream::rectified_color;
    intrin = dev->get_stream_intrinsics(stream);
    auto fmt = dev->get_stream_format(stream);
    std::cout << "Capturing " << stream << " at " << intrin.width << " x " << intrin.height << " format " << fmt;
    std::cout << std::setprecision(1) << std::fixed << ", fov = " << intrin.hfov() << " x " << intrin.vfov() << ", distortion = " << intrin.model() << std::endl;
  }

  if(!dev->is_stream_enabled(rs::stream::depth))
  {
      std::cerr << "cannot open\n";
      dev->stop();
      return 0;
  }
  else
  {
    auto &intrin  = depth_intrin;
    auto stream = rs::stream::depth_aligned_to_rectified_color;
    intrin = dev->get_stream_intrinsics(stream);
    auto fmt = dev->get_stream_format(stream);
    const float scale = dev->get_depth_scale();
    std::cout << "Capturing " << stream << " at " << intrin.width << " x " << intrin.height << " format " << fmt << " scale " << scale;
    std::cout << std::setprecision(1) << std::fixed << ", fov = " << intrin.hfov() << " x " << intrin.vfov() << ", distortion = " << intrin.model() << std::endl;

  }


  if(!dev->is_stream_enabled(rs::stream::infrared))
  {
      std::cerr << "cannot open\n";
      dev->stop();
      return 0;
  }
  else
  {
    auto &intrin  = ir_intrin;
    auto stream = rs::stream::infrared;
    intrin = dev->get_stream_intrinsics(stream);
    auto fmt = dev->get_stream_format(stream);
    const float scale = dev->get_depth_scale();
    std::cout << "Capturing " << stream << " at " << intrin.width << " x " << intrin.height << " format " << fmt << " scale " << scale;
    std::cout << std::setprecision(1) << std::fixed << ", fov = " << intrin.hfov() << " x " << intrin.vfov() << ", distortion = " << intrin.model() << std::endl;

  }


  // camera_info_manager::CameraInfoManager ir_camera_info_manager(n, camera_name_ir, ir_camera_info_url);

  // fill in ... 
  //realsense_reg_points_pub = n.advertise<sensor_msgs::PointCloud2>(topic_depth_registered_points_id, 1);
  realsense_rgb_image_pub = image_transport.advertiseCamera(topic_image_rgb_id, 1);
  realsense_depth_image_pub = image_transport.advertiseCamera(topic_depth_id, 1);
  realsense_ir_image_pub = image_transport.advertiseCamera(topic_ir_id, 1);
  std::cout << "advertised\n";
  // convert
  //sensor_msgs::CameraInfo depth_ci = rs_intrinsics2camerainfo(depth_intrin, depth_frame_id);

  // TODO: unsupported INVERSE_BROWN_CONRADY
  std::cout << "allocating color_ci\n";
  sensor_msgs::CameraInfoPtr color_ci_ptr(boost::make_shared<sensor_msgs::CameraInfo>());
  rs_intrinsics2camerainfo(*color_ci_ptr, color_intrin,rgb_frame_id);
  sensor_msgs::CameraInfoPtr depth_ci_ptr(boost::make_shared<sensor_msgs::CameraInfo>());
  rs_intrinsics2camerainfo(*depth_ci_ptr, depth_intrin,depth_frame_id);
  sensor_msgs::CameraInfoPtr ir_ci_ptr(boost::make_shared<sensor_msgs::CameraInfo>());
  rs_intrinsics2camerainfo(*ir_ci_ptr, ir_intrin,depth_frame_id);

	ros::Rate loop_rate(60);
	ros::Rate idle_rate(1);

  dev->start();

  /*
  realsense_xyz_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
  realsense_xyz_cloud->width = depth_stream.width;
  realsense_xyz_cloud->height = depth_stream.height;
  realsense_xyz_cloud->is_dense = false;
  realsense_xyz_cloud->points.resize(depth_stream.width * depth_stream.height);  
  */
    std::cout << "started " << std::endl;

    unsigned int head_sequence_id = 0;
ros::Time head_time_stamp;
	while(true)
	{
    // any subscription
		while ((getNumRGBSubscribers() + getNumDepthSubscribers()) == 0 && ros::ok())
		{
		    ros::spinOnce();
		    idle_rate.sleep();
		}
    if(!ros::ok())
      break;
    dev->wait_for_frames();
    head_sequence_id++;
    head_time_stamp = ros::Time::now();

    std::cout << "got frame " << std::endl;
    std::cout << ". " << std::endl;
    const uint8_t *color_frame= 0;
    const uint16_t *depth_frame=0;
    const uint16_t *ir_frame=0;
    {
      // TODO publish depth (realsense_depth_image_pub, depth_ci, ...)
      color_frame = reinterpret_cast<const uint8_t *>(dev->get_frame_data(rs::stream::rectified_color));

      sensor_msgs::ImagePtr rgb_img(new sensor_msgs::Image);

      rgb_img->header.seq = head_sequence_id;
      rgb_img->header.stamp = head_time_stamp;
      rgb_img->header.frame_id = rgb_frame_id;

      rgb_img->width = color_intrin.width;
      rgb_img->height = color_intrin.height;

      rgb_img->encoding = sensor_msgs::image_encodings::RGB8;
      rgb_img->is_bigendian = 0;

      int step = sizeof(unsigned char) * 3 * rgb_img->width;
      int size = step * rgb_img->height;
      rgb_img->step = step;
      rgb_img->data.resize(size);
      memcpy(&(rgb_img->data[0]), color_frame, size);

      realsense_rgb_image_pub.publish(rgb_img,color_ci_ptr);
    }
    // Depth is stored as one unsigned 16-bit integer per pixel, mapped linearly to depth in camera-specific units
    {
      depth_frame = reinterpret_cast<const uint16_t *>(
          dev->get_frame_data(rs::stream::depth_aligned_to_rectified_color));
      // TODO realsense_depth_image_pub.publish(depth_img, ir_camera_info);
      //dh((uint16_t*)depth_frame, depth_intrin.width, depth_intrin.height, frames);

      // TODO publish depth (realsense_depth_image_pub, depth_ci, ...)
      sensor_msgs::ImagePtr depth_img(new sensor_msgs::Image);

      depth_img->header.seq = head_sequence_id;
      depth_img->header.stamp = head_time_stamp;
      depth_img->header.frame_id = depth_frame_id;

      depth_img->width = depth_intrin.width;
      depth_img->height = depth_intrin.height;

      depth_img->encoding = sensor_msgs::image_encodings::MONO16;
      depth_img->is_bigendian = 0;

      int step = sizeof(unsigned short) * depth_img->width;
      int size = step * depth_img->height;
      depth_img->step = step;
      depth_img->data.resize(size);
      memcpy(&(depth_img->data[0]), depth_frame, size);

      realsense_depth_image_pub.publish(depth_img,depth_ci_ptr);      
    }

      ir_frame = reinterpret_cast<const uint16_t *>(
          dev->get_frame_data(rs::stream::infrared));
      if(ir_frame)
    {

      // TODO realsense_depth_image_pub.publish(depth_img, ir_camera_info);
      //dh((uint16_t*)depth_frame, depth_intrin.width, depth_intrin.height, frames);

      // TODO publish depth (realsense_depth_image_pub, depth_ci, ...)
      sensor_msgs::ImagePtr ir_img(new sensor_msgs::Image);

      ir_img->header.seq = head_sequence_id;
      ir_img->header.stamp = head_time_stamp;
      ir_img->header.frame_id = depth_frame_id;

      ir_img->width = ir_intrin.width;
      ir_img->height = ir_intrin.height;

      ir_img->encoding = sensor_msgs::image_encodings::MONO16;
      ir_img->is_bigendian = 0;

      int step = sizeof(unsigned short) * ir_img->width;
      int size = step * ir_img->height;
      ir_img->step = step;
      ir_img->data.resize(size);
      memcpy(&(ir_img->data[0]), ir_frame, size);

      realsense_ir_image_pub.publish(ir_img,ir_ci_ptr);      
    }
    // uncolored pcl
    {
      auto  pts = reinterpret_cast<const float3 *>(
          dev->get_frame_data(rs::stream::points));
      std::cout << "generated points " << pts << std::endl;

      sensor_msgs::PointCloud2Ptr pc2(new sensor_msgs::PointCloud2());

      pc2.header.seq = head_sequence_id;
      pc2.header.stamp = head_time_stamp;
      pc2.header.frame_id = depth_frame_id;

      pc2.height =depth_intrin.height;
      pc2.width = depth_intrin.width;
      pc2.is_bigendian = 0;
      pc2.point_step = sizeof(float)*3;
      pc2.row_step = pc2.point_step*depth_intrin.width;
      pc2.is_dense = 1;
      int size =pc2.width*pc2.height*3;
      pc2.resize(size);
      memcpy(&(pc2->data[0]), pts, size);
      // PointField[] fields
      // uint8 FLOAT32 = 7
      // uint8 UINT8   = 2
      // name,offset,datatype,count
      // uses x y z rgb
      realsense_reg_points_pub.publish (pc2);
      // store in pointcloud
      // pointcloud
      //realsense_rgb_image_pub.publish(rgb_img, rgb_camera_info);
#if 0
    int dx, dy;
    for(dy=0; dy<depth_intrin.height; ++dy)
    {
        for(dx=0; dx<depth_intrin.width; ++dx)
        {
            /* Retrieve the 16-bit depth value and map it into a depth in meters */
            uint16_t depth_value = depth_image[dy * depth_intrin.width + dx];
            float depth_in_meters = depth_value * scale;

            /* Skip over pixels with a depth value of zero, which is used to indicate no data */
            if(depth_value == 0) continue;

            /* Map from pixel coordinates in the depth image to pixel coordinates in the color image */
            float depth_pixel[2] = {(float)dx, (float)dy};
            float depth_point[3];

            rs_deproject_pixel_to_point(depth_point, &depth_intrin, depth_pixel, depth_in_meters);

            /* store a vertex at the 3D location of this depth pixel */
            pointcloud[dy*depth_intrin.width*3 + dx*3 + 0] = depth_point[0];
            pointcloud[dy*depth_intrin.width*3 + dx*3 + 1] = depth_point[1];
            pointcloud[dy*depth_intrin.width*3 + dx*3 + 2] = depth_point[2];
        }
    }
    #endif
    }
    //loop_rate.sleep();
    ros::spinOnce();

  }
  dev->stop();
	return  0;

    return 0;
}