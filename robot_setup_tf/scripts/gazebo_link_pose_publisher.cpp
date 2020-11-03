#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/LinkStates.h>

#define pass (void)0

class GazeboLinkPose {
    public:
        ros::Subscriber gazebo_sub;
        ros::Publisher link_state_pub;

        GazeboLinkPose(std::string _link_name, ros::NodeHandle nh) {
            link_name = _link_name;
            link_name_rectified = link_name.replace(link_name.find(toReplace), toReplace.length(), "_");

            gazebo_sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 500, link_state_callback());
            link_state_pub = nh.advertise<geometry_msgs::PoseStamped>("/gazebo/" + link_name, 500);
        }
        
        ~GazeboLinkPose(){

        }
        
        void link_state_callback(const gazebo_msgs::LinkStates msg) {
            try {
                int ind = msg.name.index(link_name);
                link_pose = msg.pose[ind];
            }
            catch {
                pass;
            }
        }

    private:
        std::string link_name;
        geometry_msgs::PoseStamped link_pose;
        std::string link_name_rectified;
        std::string toReplace = "::";

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gazebo_link_pose", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::spin();

    std::string link_name;
    ros::Rate r;

    n.param<ros::Rate>("publish_rate", r, 70)
    n.param<std::string>("link_name", link_name, "base_link")
    
    while(n.ok()) {
        GazeboLinkPose gp(link_name, n);
        gp.link_state_pub.publish(gp.link_pose);
        ros::spinOnce();
        r.sleep();
    }

    return 0
}