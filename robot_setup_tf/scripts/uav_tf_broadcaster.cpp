#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

class HandlePoseData {
    public:
        tf::Transform tf_from_pose;
        short neg_flag = 0;
        
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
        // for debugging
        geometry_msgs::PoseStamped debug_pose_var; 

        void pose_data_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

            // Taking the negative sign for reversing the direction.

            int sg = neg_flag ? -1 : 1;             

            pose_quat = tf::Quaternion(sg*(msg->pose.orientation.x),
                sg*(msg->pose.orientation.y),
                sg*(msg->pose.orientation.z),
                sg*(msg->pose.orientation.w)).normalize(); // normalize and return

            pose_trans = tf::Vector3(sg*(msg->pose.position.x),
                sg*(msg->pose.position.y),
                sg*(msg->pose.position.z));

            debug_pose_var = *msg;
            tf_from_pose = tf::Transform( pose_quat , pose_trans );
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_tf_publisher");
    ros::NodeHandle n;

    ros::Rate r(70);

    tf::TransformBroadcaster lidar_to_base_tf;
    tf::TransformBroadcaster depth_cam_tf;
    tf::TransformBroadcaster base_link_to_map;

    HandlePoseData pose_handle("/robot_setup_tf/if750a_base_link", n);
    pose_handle.neg_flag = 1;

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
                pose_handle.tf_from_pose, ros::Time::now(), "base_link", "map"
            )
        );
        
        ros::spinOnce();
        r.sleep();
    }
}