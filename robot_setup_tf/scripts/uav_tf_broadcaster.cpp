#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_tf_publisher");
    ros::NodeHandle n;

    ros::Rate r(100);

    tf::TransformBroadcaster lidar_to_base_tf;
    tf::TransformBroadcaster depth_cam_tf;

    while(n.ok()) {
        lidar_to_base_tf.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1125, 0.0, 0.5)),
                ros::Time::now(), "base_link", "velodyne"));

        depth_cam_tf.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.145, 0.0, 0.2855)),
                ros::Time::now(), "base_link", "camera_link"));
        
        r.sleep();
    }
}