#include <octomap_generator/octomap_generator_ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <octomap_msgs/conversions.h>
#include <pcl/conversions.h>
#include <boost/bind.hpp>
#include <cmath>
#include <sstream>
#include <cstring>

namespace g_mapper {
    OctomapGeneratorNode::OctomapGeneratorNode(ros::NodeHandle& _nh) : nh(_nh) {
        // Initiate octree
        octomap_generator = new OctomapGenerator();
        reset();
        // We will register the callback later after synchronizing. See the reference in th .h file.
        fullmap_pub = nh.advertise<octomap_msgs::Octomap>("octomap_full", 1, true);
        // refer to: http://docs.ros.org/en/jade/api/message_filters/html/c++/classmessage__filters_1_1Subscriber.html
        pointcloud_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, pointcloud_topic, 5);
        // refer to: http://docs.ros.org/en/diamondback/api/tf/html/c++/classtf_1_1MessageFilter.html
        tf_pointcloud_sub = new tf::MessageFilter<sensor_msgs::PointCloud2>(*pointcloud_sub, tf_listener, world_frame_id, 5);
        // refer to: http://www.boost.org/doc/libs/1_42_0/libs/bind/bind.html#with_functions
        // refer to: https://www.boost.org/doc/libs/1_66_0/libs/bind/doc/html/bind.html#bind.purpose.using_bind_with_pointers_to_memb
        // refer to: https://www.geeksforgeeks.org/bind-function-placeholders-c/
        // Note: The other option is to use std::bind but then you will have to use
        // std::placeholders::_1, which looks horrible compared to boost::bind's compact form.
        tf_pointcloud_sub->registerCallback(boost::bind(&OctomapGeneratorNode::insertCloudCallback, this, _1));
        // The above line is equivalent to (internal copy of this)->insertCloudCallback()
    }

    OctomapGeneratorNode::~OctomapGeneratorNode() {}

    void OctomapGeneratorNode::reset() { // Use nh.setParam to set their value in the node
        nh.getParam("/octomap/pointcloud_topic", pointcloud_topic);
        nh.getParam("/octomap/world_frame_id", world_frame_id);
        nh.getParam("/octomap/resolution", resolution);
        nh.getParam("/octomap/max_range", max_range);
        nh.getParam("/octomap/raycast_range", raycast_range);
        nh.getParam("/octomap/clamping_thres_min", clamping_thres_min);
        nh.getParam("/octomap/clamping_thres_max", clamping_thres_max);
        nh.getParam("/octomap/occupancy_thres", occupancy_thres);
        nh.getParam("/octomap/prob_hit", prob_hit);
        nh.getParam("/octomap/prob_miss", prob_miss);
        
        octomap_generator->setClampingThresMin(clamping_thres_min);
        octomap_generator->setClampingThresMax(clamping_thres_max);
        octomap_generator->setResolution(resolution);
        octomap_generator->setOccupancyThres(occupancy_thres);
        octomap_generator->setProbHit(prob_hit);
        octomap_generator->setProbMiss(prob_miss);
        octomap_generator->setRayCastRange(raycast_range);
        octomap_generator->setMaxRange(max_range);
    }

    void OctomapGeneratorNode::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(*cloud_msg, *cloud);

        tf::StampedTransform sensor_to_world_tf;
        try {
            tf_listener.lookupTransform(world_frame_id, cloud_msg->header.frame_id, cloud_msg->header.stamp, sensor_to_world_tf);
        } catch(tf::TransformException& ex) {
            ROS_ERROR_STREAM(" Error in sensor data transform: " << ex.what());
        }

        Eigen::Matrix4f sensor_to_world;
        pcl_ros::transformAsMatrix(sensor_to_world_tf, sensor_to_world);
        octomap_generator->insertPointCloud(cloud, sensor_to_world);
        // Publish the Octomap
        map_msg.header.frame_id = world_frame_id;
        map_msg.header.stamp = cloud_msg->header.stamp;
        if(octomap_msgs::fullMapToMsg(*octomap_generator->getOctree(), map_msg)) {
            fullmap_pub.publish(map_msg);
        } else {
            ROS_ERROR(" Unable to Publish Octomap: Check for Serialization Errors! ");
        }
    }

    bool OctomapGeneratorNode::save(const char* filename) const {
        return octomap_generator->save(filename);
    }
}