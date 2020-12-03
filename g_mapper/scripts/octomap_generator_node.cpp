#include <octomap_generator/octomap_generator_ros.h>
#include <ros/ros.h>
#include <cstring>

int main(int argc, char** argv) {
    ros::init(argc, argv, "octomap_generator");
    ros::NodeHandle nh;
    
    // Setting the params.
    // All the values should be fine tuned to get a good responsive map.
    // Especially tune the hit and miss probabilities acc to the sensor.
    // You may set these from the launch file and remove them from here.
    nh.setParam("/octomap/pointcloud_topic", "/velodyne_points");
    nh.setParam("/octomap/world_frame_id", "map");
    nh.setParam("/octomap/resolution", 0.05); // 5 cm
    nh.setParam("/octomap/max_range", 130.0); // see the SDF file of the sensor
    nh.setParam("/octomap/raycast_range", 130.0 - 0.9);
    nh.setParam("/octomap/clamping_thres_min", 0.12);
    nh.setParam("/octomap/clamping_thres_max", 0.97);
    nh.setParam("/octomap/occupancy_thres", 0.7);
    nh.setParam("/octomap/prob_hit", 0.8);
    nh.setParam("/octomap/prob_miss", 0.2);
    nh.setParam("/octomap/save_path", "save_dir/octomap_test.ot");

    // PASSED std::cout << "This is a test" << std::endl;

    g_mapper::OctomapGeneratorNode octomap_generator_node(nh);

    ros::spin();

    std::string save_path;
    nh.getParam("/octomap/save_path", save_path);

    octomap_generator_node.save(save_path.c_str());
    ROS_INFO(" Octomap Saved! ");
    
    return 1;
}
