#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <message_filters/time_synchronizer.h>

ros::Publisher pub;
tf::TransformListener *listener;
sensor_msgs::PointCloud2 output, output1, output2;
pcl::PointCloud<pcl::PointXYZ> output_pcl, output1_pcl, output2_pcl;

void cloud_cb1(const sensor_msgs::PointCloud2ConstPtr& input1) {
    listener->waitForTransform("/cam_1_link", (*input1).header.frame_id, (*input1).header.stamp, ros::Duration(5.0));
    // sensor_msgs::PointCloud &pcout) const 
    pcl_ros::transformPointCloud("/cam_1_link", *input1, output1, *listener);
    pcl::fromROSMsg(output1, output1_pcl);
    output_pcl = output1_pcl;
    output_pcl += output2_pcl;
    pcl::toROSMsg(output_pcl, output);
    pub.publish(output);
}

void cloud_cb2(const sensor_msgs::PointCloud2ConstPtr& input2) {
    listener->waitForTransform("/cam_1_link", (*input2).header.frame_id, (*input2).header.stamp, ros::Duration(5.0));
    // sensor_msgs::PointCloud &pcout) const 
    pcl_ros::transformPointCloud("/cam_1_link", *input2, output2, *listener);
    pcl::fromROSMsg(output2, output2_pcl);
    output_pcl = output2_pcl;
    output_pcl += output1_pcl;
    pcl::toROSMsg(output_pcl, output);
    pub.publish(output);
}

int main (int argc, char **argv)
{
    ros::init (argc, argv, "concatenate_clouds");
    ros::NodeHandle n;
    // ARPublisher ar_kinect (n);

    listener = new tf::TransformListener();
    ros::Subscriber sub1 = n.subscribe("/cam_1/depth/color/points", 1, cloud_cb1);
    ros::Subscriber sub2 = n.subscribe("/cam_2/depth/color/points", 1, cloud_cb2);

    pub = n.advertise<sensor_msgs::PointCloud2>("/merged/depth_registered/points", 1);
    ros::spin ();

    return 0;
}