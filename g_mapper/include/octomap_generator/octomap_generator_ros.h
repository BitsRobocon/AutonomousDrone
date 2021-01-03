#ifndef OCTOMAP_GENERATOR_ROS_H
#define OCTOMAP_GENERATOR_ROS_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap/ColorOcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_generator/octomap_generator.h>

namespace g_mapper {
    class OctomapGeneratorNode {
        public:
            /**
             * @brief Constructor
             * @param _nh The ros node handle for the Octomap Generating ros node. 
             */
            OctomapGeneratorNode(ros::NodeHandle& _nh);

            /// Destructor.
            virtual ~OctomapGeneratorNode();

            /// Reset values to parameters from the parameter server.
            void reset();

            /**
             * @brief Callback to to the point cloud topic subscriber. Updates the octomap and publishes it.
             * @param cloud ROS PointCloud2 message in the sensor frame (unless transformed earlier). 
             */
            void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);

            /**
             * @brief Save the octomap to a file.
             * @param filename The output file's name.
             * @return Returns 1 if successful else 0.
             */
            bool save(const char* filename) const;

            // refer to: https://www.programmersought.com/article/20554576747/ for info on octomaps.

        protected:
            OctomapGeneratorBase* octomap_generator; // A pointer to the base class instance, acts as the interface b/w node and generator class.
            ros::NodeHandle nh; // The node handle
            ros::Publisher fullmap_pub; // ROS Publisher for octomap message
            // refer to: http://wiki.ros.org/message_filters for info on message filters (generally for Time Sync).
            message_filters::Subscriber<sensor_msgs::PointCloud2>* pointcloud_sub; // ROS subscriber for pointcloud message from the sensor node. 
            tf::MessageFilter<sensor_msgs::PointCloud2>* tf_pointcloud_sub; // ROS tf message filter for time synchronization of tf and PointCloud messages.
            tf::TransformListener tf_listener; // Listener for the transform between the sensor and the world. (velodyne base_foorprint -> map).
            std::string world_frame_id; // ID of the world frame.
            std::string pointcloud_topic; // PointCloud2 topic from the sensor plugin.
            float max_range; // Max range of the points to be inserted in the octomap.
            float raycast_range; // Max range for the points to be inserted into octomap.
            float clamping_thres_max; // Upper bound on Occupancy probability, usually 0.97.
            float clamping_thres_min; // Lower bound on Occupancy Probability, usually 0.12.
            float resolution; // Resolution of the Octomap, proportional to min voxel size.
            float occupancy_thres; // Minimum occupancy probability for a node to be considered as occupied, ussually 0.5 or 0.7.
            float prob_hit; // Hit probability of sensor.
            float prob_miss; // Miss probability of sensor.
            octomap_msgs::Octomap map_msg; // ROS Octomap message.
    };
}


#endif // OCTOMAP_GENERATOR_ROS_H