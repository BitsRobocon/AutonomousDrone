#include <octomap_generator/octomap_generator.h>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/conversions.h>
#include <boost/shared_ptr.hpp>
#include <cmath>
#include <sstream>
#include <cstring>


namespace g_mapper {
    // The custom constructor of OcTree classes takes resolution as the argument.
    OctomapGenerator::OctomapGenerator() : OctomapGeneratorBase(), octomap_(0.05), max_range(1.0), raycast_range(1.0) { };

    OctomapGenerator::~OctomapGenerator() { };

    void OctomapGenerator::insertPointCloud(const pcl::PCLPointCloud2::Ptr& cloud, const Eigen::Matrix4f& sensor_to_world) {
        // We will downsample the point cloud. Creating the filtering object.
        pcl::PCLPointCloud2::Ptr cloud_filtered( new pcl::PCLPointCloud2() );
        float voxel_filter_size = octomap_.getResolution();
        pcl::VoxelGrid<pcl::PCLPointCloud2> flt;
        flt.setInputCloud(cloud);
        flt.setLeafSize(voxel_filter_size, voxel_filter_size, voxel_filter_size);
        flt.filter(*cloud_filtered);

        // Convert to PCL Pointcloud
        PCLColor pcl_cloud;
        pcl::fromPCLPointCloud2(*cloud_filtered, pcl_cloud);
        
        // Transforming the coordinates sensor -> World.
        pcl::transformPointCloud(pcl_cloud, pcl_cloud, sensor_to_world);

        // self explanatory
        octomap::point3d origin(static_cast<float>(sensor_to_world(0,3)),
        static_cast<float>(sensor_to_world(1,3)),
        static_cast<float>(sensor_to_world(2,3)));

        octomap::Pointcloud raycast_cloud; // We will insert point cloud with ray casting
        int endpoint_count = 0; // total no of endpoints inserted

        // for( auto it:pcl_cloud.points() )
        for (PCLColor::const_iterator it = pcl_cloud.begin(); it != pcl_cloud.end(); ++it) {
            // Check point validity, if remove NaN doesn't work.
            if( !(std::isnan(it->x) || std::isnan(it->y) || std::isnan(it->z)) ) {
                float dist = sqrt( pow((it->x - origin.x()), 2.0) + pow((it->y - origin.y()), 2.0) + pow((it->z - origin.z()), 2.0) );
                // Check if point is in max_range.
                if( dist <= raycast_range ) {
                    // Add to point cloud and perform raycasting later all together (inbuilt in the insertPointCloud method of octomaps).
                    raycast_cloud.push_back(it->x, it->y, it->z);
                }
                else {
                    // Otherwise update the occupancy of the node and transfer the point to raycast range.
                    octomap::point3d direction = (octomap::point3d(it->x, it->y, it->z) - origin).normalize();
                    // Using ray tracing to transfer at max_range endpoint.
                    // Recall the factors alpha and beta.
                    octomap::point3d new_end = origin + direction * (raycast_range + octomap_.getResolution()*2);
                    raycast_cloud.push_back(new_end);
                    // refer to: https://octomap.github.io/octomap/doc/classoctomap_1_1OccupancyOcTreeBase.html#a395884de10ad8de0696afb09636bc2e4
                    octomap_.updateNode(it->x, it->y, it->z, true, false); // use lazy_eval, run updateInnerOccupancy() when done
                }
            }
            endpoint_count++;
        }

        if(raycast_cloud.size() > 0) {
            // refer to: https://octomap.github.io/octomap/doc/classoctomap_1_1OccupancyOcTreeBase.html#a3c6d38e8a7028416cd23449f14e215e8
            octomap_.insertPointCloud(raycast_cloud, origin, raycast_range, false, true); // discretization is true.
        }
        // Update colors.
        updateColor(&pcl_cloud);
        // update inner node occupancy and colors.
        if(endpoint_count > 0) {
            octomap_.updateInnerOccupancy();
        }
    }

    void OctomapGenerator::updateColor(PCLColor* pcl_cloud) {
        for(PCLColor::const_iterator it = pcl_cloud->begin(); it != pcl_cloud->end(); it++) {
            if( !(std::isnan(it->x) || std::isnan(it->y) || (std::isnan(it->z))) ) {
                // refer to: https://octomap.github.io/octomap/doc/classoctomap_1_1ColorOcTree.html#aad2f351f0d703f397b51c4fe2cdd1719
                octomap_.averageNodeColor(it->x, it->y, it->z, it->r, it->g, it->b);
            }
        }
    }

    // Some of the code has been adapted from RGBD Slam V2. See ColorOctomapServer.h and .cpp files.
    // Refer to: [RGBDSlam_V2] https://github.com/felixendres/rgbdslam_v2/tree/kinetic/src

    bool OctomapGenerator::save(const char* filename) const {
        std::ofstream outfile(filename, std::ios_base::out | std::ios_base::binary);
        if( outfile.is_open() ) {
            std::cout << "Saving Octomap to file: " << filename << std::endl;
            octomap_.write(outfile);
            outfile.close();
            std::cout << "Successfully saved tree to: " << filename << std::endl;
            return true;
        }
        else {
            ROS_ERROR_STREAM(" Could not open " << filename << " for writing " << std::endl);
            return false;
        }
    }

}
