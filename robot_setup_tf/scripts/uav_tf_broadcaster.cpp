#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <robot_setup_tf/gazebo_link_pose.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_tf_publisher");
    ros::NodeHandle n;

    ros::Rate r(70);

    tf::TransformBroadcaster lidar_to_base_tf;
    tf::TransformBroadcaster depth_cam_tf;
    tf::TransformBroadcaster base_link_to_map;

    robot_setup_tf::GazeboLinkPose pose_handle("if750a::velodyne_VLP16::base_footprint", n);
    pose_handle.setNegFlag(true);

    while(n.ok()) {
        lidar_to_base_tf.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1125, 0.0, 0.5)),
                ros::Time::now(), "base_link", "velodyne"));

        depth_cam_tf.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.145, 0.0, 0.2855)),
                ros::Time::now(), "base_link", "camera_link"));

        base_link_to_map.sendTransform(
            tf::StampedTransform(
                pose_handle.getTfFromPose(), ros::Time::now(), "velodyne", "map"
            )
        );
        
        ros::spinOnce();
        r.sleep();
    }
}
