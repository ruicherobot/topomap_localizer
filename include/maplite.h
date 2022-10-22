/*******************************************************************************************
 * File:        maplite.h
 * 
 * Author:      Rui Chen (ruiche[at]umich.edu)
 * Last Modified:     05/19/2022
 * 
 * Description: Adopt particle filter and g2o to estimate cassie's pose in topological map
*******************************************************************************************/
#ifndef MAPLITE_H
#define MAPLITE_H
// #pragma once 

#include <stdio.h>
#include <numeric>
#include <random>
#include <vector>
#include <deque>
#include <stack>
#include <tuple>
#include <cmath>

//reconfigure
#include <dynamic_reconfigure/server.h>
#include <topomap_localizer/topomap_localizerConfig.h>

//ros and tf
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h> //tftree
#include <tf/tf.h>

//Eigen
#include <Eigen/Dense>
#include <Eigen/Eigen>

//messages
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <nav_msgs/Odometry.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <std_msgs/String.h>

#include "coordinate_t.h"
#include "particle_t.h"
#include "kdtree.h"
#include "inekf_msgs/State.h"
#include "inekf_msgs/LocalizationState.h"

namespace maplite {
    
    class MapliteClass {

    public:

        MapliteClass(ros::NodeHandle& n);

        ros::NodeHandle& n;

        ros::Subscriber landmark_sub_;

        ros::Subscriber odo_sub_;

        ros::Subscriber cloud_sub_;

        ros::Publisher pose_estimate_visualization_pub_;
        
        ros::Publisher pose_pureodom_visualization_pub_;
        
        ros::Publisher pose_accuodom_visualization_pub_;
        
        ros::Publisher pose_estimate_inekf_pub_;
        
        ros::Publisher pose_estimate_geometry_pub_;

        ros::Publisher cloud_pub_;

        ros::Publisher border_pcl_pub_;

        ros::Publisher landmark_pub_;

        ros::Publisher particles_pub_;

        tf::TransformBroadcaster base_broadcaster_;//tftree

        tf::TransformBroadcaster velodyne_broadcaster_;//tftree

        tf::TransformListener base_to_velodyne_listener_;//tftree

        double tx_;

        double ty_;

        double theta_;
        
        int kdtreeCount_;
        
        int odomcount_;

        int nearest_node_number_;

        int threshold_odom_init_;

        std::vector<particle_t> particles_;

        geometry_msgs::Point odom_pf_pre_;

        geometry_msgs::Point odom_pf_;

        geometry_msgs::Point odom_temp_;

        geometry_msgs::Point odom_temp_pre_;

        geometry_msgs::Point pos_temp_;

        double center_rec_x_;

        double center_rec_y_;

        double estimated_x_;

        double estimated_y_;

        double estimated_t_;

        double noise_coefficient_action_x_;

        double noise_coefficient_action_y_;

        double noise_coefficient_action_t_;

        double noise_stdev_action_;

        double noise_coefficient_init_x_;

        double noise_coefficient_init_y_;

        double noise_coefficient_init_t_;

        double noise_stdev_init_;

        double ddistance_;

        double odom_dx_;

        double odom_dy_;

        double odom_dt_;
        
        bool moved_;
        
        std::vector<coordinate_t> point_cloud_;

        KdTree landmarks_;

        std::vector<coordinate_t> knpoints_;
        
        geometry_msgs::Quaternion quaternion_geo_;

        geometry_msgs::Quaternion quaternion_geo_pre_
        ;
        geometry_msgs::Quaternion quaternion_geo_world_;

        Eigen::Matrix<double, 3, 3> UT_;

        int particleNum_;

        sensor_msgs::PointCloud cloud;
       
        double roll_;

        double pitch_;

        double yaw_;

        double pre_roll_;

        double pre_pitch_;

        double pre_yaw_;

        double pre_yaw_pf_;

        double yaw_pf_;

        ros::ServiceClient osm_node_client_;

        static bool compareBydist(const coordinate_t& point1, const coordinate_t& point2);

        static bool compareByWeight(const particle_t &a, const particle_t &b);

        double wrap_to_pi(double angle);

        double wrap_to_pi_2(double angle);

        void storeLandmarks(visualization_msgs::Marker points);

        void updateOdometry(inekf_msgs::State input);

        void updateLocalization(const sensor_msgs::PointCloud2 &msg);

        bool determineMovement();
        
        void initializeParticles();

        void resampleParticles();

        void applyActionmodel();

        void applySensormodel(const sensor_msgs::PointCloud2 &msg);

        void calculatePosterior();

        void publishPointcloud(const sensor_msgs::PointCloud2 &msg);

        void publishTransformedOdometry(inekf_msgs::State input);

        void publishLandmarks();

        void publishParticles();

        void updateVelodyne(const sensor_msgs::PointCloud2 &msg); //tftree
    };
}

#endif /* MAPLITE_H */