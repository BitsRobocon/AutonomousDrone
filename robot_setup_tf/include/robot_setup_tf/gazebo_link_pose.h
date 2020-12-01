#ifndef GAZEBO_LINK_POSE_H
#define GAZEBO_LINK_POSE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/LinkStates.h>
#include <tf/transform_datatypes.h>

namespace robot_setup_tf{

    /**
     * \brief This class inherits from the GazeboLinkPose class. Subscribes to gazebo/link_states and
     * converts the retrieved pose to tf::Transform object. By default the tf is from the link to the
     * base map, which is the global reference frame for gazebo.
    */
    class HandlePoseForTf {
        public:

            /**
             * \brief default constructor.
            */
            HandlePoseForTf();

            /**
             * \brief Constructor
             * \param link_pose The geometry_msgs PoseStamped object to be converted.
            */
            HandlePoseForTf(geometry_msgs::PoseStamped _link_pose);

            ~HandlePoseForTf();

            void setLinkPose(geometry_msgs::PoseStamped _link_pose);

            void setNegFlag(bool i);

            tf::Transform getTfFromPose();

            bool getNegFlag();

        protected: 
            geometry_msgs::PoseStamped link_pose;
            tf::Quaternion pose_quat;
            tf::Vector3 pose_trans;
            tf::Transform base_to_map_tf;
            bool neg_flag = 0;
    };

    class GazeboLinkPose : public HandlePoseForTf {
        public:
            ros::Subscriber gazebo_sub;
            ros::Publisher link_state_pub;

            /**
             * \brief Constructor
             * \param _link_name name of the link with complete gazebo heirarchy.
             * \param _nh nodehandle of the node for publishing the retrieved pose.
             */
            GazeboLinkPose(std::string _link_name, ros::NodeHandle _nh);

            virtual ~GazeboLinkPose();

            geometry_msgs::PoseStamped getLinkPose();

            void initPosePublisher();

        protected:
            std::string link_name;
            std::string link_name_rectified;
            std::string toReplace = "::";
            geometry_msgs::PoseStamped link_pose_gazebo;
            ros::NodeHandle nh;

            /**
             * \brief callback for the gazebo/link_state subscriber. Populates the link_pose object.
            */
            void link_state_callback(const gazebo_msgs::LinkStates::ConstPtr& msg);

            /**
             * \brief This finctions returns the index of the link in the link_states list of links and poses for pose search.
            */
            int getIndex(std::vector<std::string> v, std::string search_element);
    };


}
#endif //GAZEBO_LINK_POSE_H