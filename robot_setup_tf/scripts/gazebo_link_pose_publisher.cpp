#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/LinkStates.h>

#define pass (void)0

int getIndex(std::vector<std::string> v, std::string search_element) {
    for(int i = 0; i < v.size(); i++) {
        if(v[i].compare(search_element) == 0) {
            return i;
        }
    }
    return -1;
}

class GazeboLinkPose {
    public:
        ros::Subscriber gazebo_sub;
        ros::Publisher link_state_pub;
        geometry_msgs::PoseStamped link_pose;

        GazeboLinkPose(std::string _link_name, ros::NodeHandle nh) {
            link_name = _link_name;
            link_name_rectified = link_name;
            link_name_rectified = link_name_rectified.replace(link_name.find(toReplace), toReplace.length(), "_");

            gazebo_sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 70, &GazeboLinkPose::link_state_callback, this);
            link_state_pub = nh.advertise<geometry_msgs::PoseStamped>("/robot_setup_tf/" + link_name_rectified, 70);
        }
        
        ~GazeboLinkPose(){

        }

    private:
        std::string link_name;
        std::string link_name_rectified;
        std::string toReplace = "::";

        void link_state_callback(const gazebo_msgs::LinkStates::ConstPtr& msg) {
            try {
                int ind = getIndex(msg->name, link_name);
                if(ind) {
                    link_pose.pose = msg->pose[ind];
                    link_pose.header.stamp = ros::Time::now();
                    link_pose.header.frame_id = link_name_rectified;
                }
                else {
                    throw "Unable to find the link's name in Gazebo's link_states name index. Please check whether the name provided is correct";
                }
            }
            catch(const char* e) {
                ROS_INFO("WARNING: [%s]", e);
            }
        }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gazebo_link_pose", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    std::string link_name;
    int r;
    n.param<int>("publish_rate", r, 70);
    ros::Rate loop_rate(r);
    n.param<std::string>("link_name", link_name, "if750a::base_link");

    GazeboLinkPose gp(link_name, n);
    
    while(n.ok()) {
        gp.link_state_pub.publish(gp.link_pose);
        ros::spinOnce(); // https://answers.ros.org/question/282259/ros-class-with-callback-methods/
        loop_rate.sleep();
    }

    return 0;
}