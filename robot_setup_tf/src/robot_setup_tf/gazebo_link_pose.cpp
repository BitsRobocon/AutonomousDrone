#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <robot_setup_tf/gazebo_link_pose.h>

typedef geometry_msgs::PoseStamped PoseStamped;

namespace robot_setup_tf{


    HandlePoseForTf::HandlePoseForTf(){ };

    HandlePoseForTf::HandlePoseForTf(PoseStamped _link_pose) { this->link_pose = _link_pose; };

    HandlePoseForTf::~HandlePoseForTf(){ };

    void HandlePoseForTf::setLinkPose(PoseStamped _link_pose) {
        this->link_pose = _link_pose;
    }

    tf::Transform HandlePoseForTf::getTfFromPose() {
        int sg = neg_flag ? -1 : 1;
        pose_trans = tf::Vector3(sg*(link_pose.pose.position.x), sg*(link_pose.pose.position.y), sg*(link_pose.pose.position.z));
        pose_quat = tf::Quaternion(sg*(link_pose.pose.orientation.x),
        sg*(link_pose.pose.orientation.y),
        sg*(link_pose.pose.orientation.z),
        sg*(link_pose.pose.orientation.w)).normalize();
        base_to_map_tf.setOrigin(pose_trans);
        base_to_map_tf.setRotation(pose_quat);

        return this->base_to_map_tf;
    }

    /**
     * \brief The Negative Flag is used to get the inverse transform -T 
     * \param i (bool)
    */
    void HandlePoseForTf::setNegFlag(bool i){
        this->neg_flag = i;
    }

    bool HandlePoseForTf::getNegFlag(){
        return this->neg_flag;
    }

    GazeboLinkPose::GazeboLinkPose(std::string _link_name, ros::NodeHandle _nh) : HandlePoseForTf() {
        link_name = _link_name;
        link_name_rectified = link_name;
        link_name_rectified = link_name_rectified.replace(link_name.find(toReplace), toReplace.length(), "_");
        nh = _nh;

        gazebo_sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 70, &GazeboLinkPose::link_state_callback, this);
    }

    GazeboLinkPose::~GazeboLinkPose(){ };

    void GazeboLinkPose::link_state_callback(const gazebo_msgs::LinkStates::ConstPtr& msg){
        try {
            int ind = getIndex(msg->name, link_name);
            if(ind) {
                link_pose_gazebo.pose = msg->pose[ind];
                link_pose_gazebo.header.stamp = ros::Time::now();
                link_pose_gazebo.header.frame_id = link_name_rectified;
                setLinkPose(link_pose_gazebo);
            }
            else {
                throw "Unable to find the link's name in Gazebo's link_states name index. Please check whether the name provided is correct";
            }
        }
        catch(const char* e) {
            ROS_INFO("WARNING: [%s]", e);
        }
    }
    
    int GazeboLinkPose::getIndex(std::vector<std::string> v, std::string search_element){
        for(int i = 0; i < v.size(); i++) {
            if(v[i].compare(search_element) == 0) {
                return i;
            }
        }
        return -1;
    }

    PoseStamped GazeboLinkPose::getLinkPose() {
        return this->link_pose_gazebo;
    }

    void GazeboLinkPose::initPosePublisher() {
        link_state_pub = nh.advertise<PoseStamped>("/robot_setup_tf/" + link_name_rectified, 70);
    }

}
