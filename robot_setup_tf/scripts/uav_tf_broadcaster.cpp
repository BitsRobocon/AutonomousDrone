#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

class HandlePoseData {
    public:
        tf::Transform tf_from_pose;
        
        HandlePoseData(std::string _topic_name, ros::NodeHandle nh) {
            topic_name = _topic_name;

            gazebo_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_name, 70, &HandlePoseData::pose_data_callback, this);

        }

        ~HandlePoseData() {

        }

    private:
        ros::Subscriber gazebo_pose_sub;
        std::string topic_name;
        tf::Quaternion pose_quat;
        tf::Vector3 pose_trans;

        void pose_data_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
            // Taking the negative sign for reversing the direction.
            pose_quat = tf::Quaternion(-msg->pose.orientation.x,
                -msg->pose.orientation.y,
                -msg->pose.orientation.z,
                -msg->pose.orientation.w);

            pose_trans = tf::Vector3(-msg->pose.position.x,
                -msg->pose.position.y,
                -msg->pose.position.z);

            tf_from_pose = tf::Transform( pose_quat , pose_trans );
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_tf_publisher");
    ros::NodeHandle n;

    ros::Rate r(100);

    tf::TransformBroadcaster lidar_to_base_tf;
    tf::TransformBroadcaster depth_cam_tf;
    tf::TransformBroadcaster base_link_to_map;

    while(n.ok()) {
        lidar_to_base_tf.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1125, 0.0, 0.5)),
                ros::Time::now(), "base_link", "velodyne"));

        depth_cam_tf.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.145, 0.0, 0.2855)),
                ros::Time::now(), "base_link", "camera_link"));

        HandlePoseData pose_handle("if750a_base_link", n);

        base_link_to_map.sendTransform(
            tf::StampedTransform(
                pose_handle.tf_from_pose, ros::Time::now(), "base_link", "map"
            )
        );
        
        r.sleep();
    }
}