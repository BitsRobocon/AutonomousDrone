#ifndef OCTOMAP_GENERATOR_H
#define OCTOMAP_GENERATOR_H

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PCLColor;
typedef octomap::ColorOcTree ColorOcTree;



namespace g_mapper {

    /**
     * @brief Provides interface for the octomap generator class for polymorphism. No custom constructor.
     * Add all the custom parameter interfacing methods in this class. 
     *  
     * @details Adapted from: [Semantic Slam] https://awesomeopensource.com/project/floatlazer/semantic_slam.
     */
    class OctomapGeneratorBase {
        public:
            // https://stackoverflow.com/questions/39288839/why-in-c-virtual-and-0-is-both-needed-to-describe-a-method-is-abstract

            /// Destructor
            virtual ~OctomapGeneratorBase() { };
            // IMPORTANT: https://gcc.gnu.org/faq.html#vtables see the last FAQ.

            /// Set max range for point cloud insertion.
            virtual void setMaxRange(float _max_range) = 0;

            /// Set max range to perform raycasting on the inserted points.
            virtual void setRayCastRange(float _raycast_range) = 0;

            /// Set max clamping thershold for octomap. Recommended default is 0.97
            virtual void setClampingThresMax(float _clamping_thres_max) = 0;

            /// Set min clamping threshold for octomap. Recommended default is 0.12
            virtual void setClampingThresMin(float _clamping_thres_min) = 0;

            /// Set the resolution for the octomap
            virtual void setResolution(float _resolution) = 0;

            /// Set the occupancy threshold for octomap. Recommended default is 0.5 or 0.7
            virtual void setOccupancyThres(float _occupancy_thres) = 0;

            /// Set the hitting probability of the sensor model.
            virtual void setProbHit(float _prob_hit) = 0;

            /// Set the missing probability of the sensor model.
            virtual void setProbMiss(float _prob_miss) = 0;

            /**
             * @brief Insert the point cloud into the OcTree and update.
             * @param cloud Converted ROS PointCloud2 object
             * @param sensor_to_world Tf from sensor frame to world frame (map) 
             */
            virtual void insertPointCloud(const pcl::PCLPointCloud2::Ptr& cloud, const Eigen::Matrix4f& sensor_to_world) = 0;

            /// get octree
            virtual octomap::AbstractOccupancyOcTree* getOctree() = 0;

            /// Save octomap to a file.
            virtual bool save(const char* filename) const = 0;

        protected:
    };

    /**
     * @brief The main octomap generator class. Generates a ColorOcTree for now. A semantic octree can be added later.
     * refer to: https://github.com/floatlazer/semantic_slam. \n
     * IMPORTANT: Needs a Tf from Sensor to World frame to work. Hence ensure that said tf is broadcasted.
     *
     */
    class OctomapGenerator : public OctomapGeneratorBase {
        public:
            
            OctomapGenerator();

            virtual ~OctomapGenerator(); // https://stackoverflow.com/questions/461203/when-to-use-virtual-destructors

            virtual void setMaxRange(float _max_range) { 
                max_range = _max_range; 
            }

            virtual void setRayCastRange(float _raycast_range) {
                raycast_range = _raycast_range;
            }

            virtual void setClampingThresMax(float _clamping_thres_max) {
                octomap_.setClampingThresMax( _clamping_thres_max );
            }

            virtual void setClampingThresMin(float _clamping_thres_min) {
                octomap_.setClampingThresMin( _clamping_thres_min );
            }

            virtual void setProbHit(float _prob_hit) {
                octomap_.setProbHit( _prob_hit );
            }

            virtual void setProbMiss(float _prob_miss) {
                octomap_.setProbMiss( _prob_miss );
            }

            /**
             * @brief [IMPORTANT] Callback to the point cloud topic from the LiDAR. Update the octomap and publish it.
             * @param cloud ROS PointCloud2 message in the sensor frame. 
             */
            virtual void insertPointCloud(const pcl::PCLPointCloud2::Ptr& cloud, const Eigen::Matrix4f& sensor_to_world);

            /// Get the octree 
            virtual octomap::AbstractOccupancyOcTree* getOctree() {
                return &octomap_;
            }

            /**
             * @brief Save the octomap to a file.
             * @param filename The output file's name.
             */
            virtual bool save(const char* filename) const;

        protected:
            octomap::ColorOcTree octomap_; // Will be templated in the future.
            float max_range;
            float raycast_range;

            void updateColor(PCLColor* pcl_cloud);
    };
}

#endif//OCTOMAP_GENERATOR_H