/*******************************************************************************************
 * File:        maplite.cpp
 * 
 * Author:      Rui Chen (ruiche[at]umich.edu)
 * Last Modified:     05/05/2022
 * 
 * Description: Adopt particle filter and g2o to estimate cassie's pose in topological map
*******************************************************************************************/

#include <maplite.h>

namespace maplite{

    bool MapliteClass::compareByWeight(const particle_t &a, const particle_t &b)
    {
        return a.weight > b.weight;
    }

    bool MapliteClass::compareBydist(const coordinate_t& point1, const coordinate_t& point2)
    {
        double dist1 = sqrt(pow(point1.val_[0] - point1.val_[2], 2) + pow(point1.val_[1] - point1.val_[3], 2));
        double dist2 = sqrt(pow(point2.val_[0] - point2.val_[2], 2) + pow(point2.val_[1] - point2.val_[3], 2));
        return dist1 < dist2;
    }

    double MapliteClass::wrap_to_pi(double angle)
    {
        if(angle < -M_PI)
        {
            for(; angle < -M_PI; angle += 2.0*M_PI);
        }
        else if(angle > M_PI)
        {
            for(; angle > M_PI; angle -= 2.0*M_PI);
        }

        return angle;
    }

    double MapliteClass::wrap_to_pi_2(double angle)
    {
        double wrapped = wrap_to_pi(angle);
        
        if(wrapped < -M_PI_2)
        {
            wrapped += M_PI;
        }
        else if(wrapped > M_PI_2)
        {
            wrapped -= M_PI;
        }
        
        return wrapped;
    }

    MapliteClass::MapliteClass(ros::NodeHandle& n): n(n){
        // parameters parser
        n.getParam("maplite/tx", tx_);
	    n.getParam("maplite/ty", ty_);
	    n.getParam("maplite/theta", theta_);
        n.getParam("maplite/particleNum", particleNum_);
        n.getParam("maplite/nearest_node_number", nearest_node_number_);
        n.getParam("maplite/noise_coefficient_x", noise_coefficient_x_);
        n.getParam("maplite/noise_coefficient_y", noise_coefficient_y_);
        n.getParam("maplite/noise_coefficient_t", noise_coefficient_t_);
        n.getParam("maplite/noise_stdev", noise_stdev_);

        // initialize updateOdom
        odom_temp_pre_.x = 0;
        odom_temp_pre_.y = 0;
        odom_temp_pre_.z = 0;
        pos_temp_.x = tx_;
        pos_temp_.y = ty_;
        pos_temp_.z = 0;
        quaternion_geo_world_ = tf::createQuaternionMsgFromRollPitchYaw(0, 0, theta_);
        
        // initialize updateLocalization
        odom_pf_pre_ = odom_temp_pre_;
        estimated_x_ = tx_;
        estimated_y_ = ty_;
        estimated_t_ = theta_;
        odomcount_ = 0;
        kdtreeCount_ = 0;
        // Publisher
        pose_estimate_visualization_pub_ = n.advertise<visualization_msgs::Marker>("estimate_visualization", 1);
        pose_pureodom_visualization_pub_ = n.advertise<visualization_msgs::Marker>("pureodom_visualization", 1);
        pose_accuodom_visualization_pub_ = n.advertise<visualization_msgs::Marker>("accuodom_visualization", 1);
        pose_estimate_geometry_pub_ = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("estimate_geometry", 1);
        pose_estimate_inekf_pub_ = n.advertise<inekf_msgs::LocalizationState>("estimated_inekf", 1);
        landmark_pub_ = n.advertise<visualization_msgs::MarkerArray>("references", 10);
        particles_pub_ = n.advertise<visualization_msgs::MarkerArray>("particles", 10);
        cloud_pub_ = n.advertise<sensor_msgs::PointCloud>("point_cloud", 10);
        border_pcl_pub_ = n.advertise<sensor_msgs::PointCloud2>("border_pointcloud", 10);
        // Subscriber
        landmark_sub_ = n.subscribe("/IMOMD/global_map_landmark", 1, &MapliteClass::storeLandmarks, this);
        odo_sub_ = n.subscribe("/cassie/inekf_state", 10, &MapliteClass::updateOdometry, this);
        cloud_sub_ = n.subscribe("/feature_extraction/landmark", 10, &MapliteClass::updateLocalization, this);
    }

    void MapliteClass::storeLandmarks(visualization_msgs::Marker points)
    {
        coordinate_t temp;
        for(int i = 0; i < points.points.size(); i++){
            temp.val_[0] = points.points[i].x;
            temp.val_[1] = points.points[i].y;
            point_cloud_.push_back(temp);
        }
        std::vector<double> bias;
        bias.push_back(8.0);
        bias.push_back(-28.0);
        landmarks_.Create(bias, point_cloud_);
        std::cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<std::endl;
        std::cout<<points.points.size()<<"   points<<<<<<<<<<----size---->>>>>>>>landmarks  "<<landmarks_.size_<<std::endl;
        std::cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<std::endl;
        
    }

    void MapliteClass::updateOdometry(inekf_msgs::State input)
    {   
        odomcount_++;
        odom_temp_pre_ = odom_temp_;
        odom_temp_ = input.position;

        quaternion_geo_pre_ = quaternion_geo_;
        quaternion_geo_ = input.orientation;
        tf::Quaternion quaternion_tf;
        tf::quaternionMsgToTF(quaternion_geo_pre_, quaternion_tf);
        tf::Matrix3x3(quaternion_tf).getRPY(pre_roll_, pre_pitch_, pre_yaw_);
        tf::quaternionMsgToTF(quaternion_geo_, quaternion_tf);
        tf::Matrix3x3(quaternion_tf).getRPY(roll_, pitch_, yaw_);
        
        // compensate the bias in original odom orientation
        if(odomcount_ == 20){
            theta_ = theta_ - yaw_;
            estimated_t_ = theta_;
            std::cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<std::endl;
            std::cout<<"      Bias in original odom orientation is switched back      "<<std::endl;
            std::cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<std::endl;
        }
        
        quaternion_geo_world_ = tf::createQuaternionMsgFromRollPitchYaw(roll_, pitch_, yaw_ + theta_);

        double dx = odom_temp_.x - odom_temp_pre_.x;
        double dy = odom_temp_.y - odom_temp_pre_.y;
        pos_temp_.x = pos_temp_.x + cos(theta_) * dx - sin(theta_) * dy;
        pos_temp_.y = pos_temp_.y + sin(theta_) * dx + cos(theta_) * dy;
        this->publishTransformedOdometry(input);
    }


    void MapliteClass::updateLocalization(const sensor_msgs::PointCloud2 &msg)
    {
        kdtreeCount_++;
        odom_pf_ = odom_temp_;
        yaw_pf_ = yaw_;

        if(kdtreeCount_ == 1){
            
            this->initializeParticles();
            
            this->applySensormodel(msg);

            this->calculatePosterior();

            this->publishLandmarks();

            this->publishPointcloud(msg);

        }

        else{
            this->publishParticles();
            
            moved_ = this->determineMovement();
            
            if(moved_ || kdtreeCount_ <= 10){
                // 1
                this->resampleParticles(); 
                // 2
                this->applyActionmodel();
                // 3
                this->applySensormodel(msg);
                // 4
                this->calculatePosterior();
            }
            
            this->publishPointcloud(msg);
            
            this->publishLandmarks();
            
            odom_pf_pre_ = odom_pf_;
            pre_yaw_pf_ = yaw_pf_;
        }        
    }
    
    bool MapliteClass::determineMovement()
    {
        ddistance_ = sqrt(pow((odom_pf_.x - odom_pf_pre_.x), 2) + pow((odom_pf_.y - odom_pf_pre_.y), 2));
        odom_dx_= (odom_pf_.x - odom_pf_pre_.x) * cos(theta_) - (odom_pf_.y - odom_pf_pre_.y) * sin(theta_);
        odom_dy_ = (odom_pf_.x - odom_pf_pre_.x) * sin(theta_) + (odom_pf_.y - odom_pf_pre_.y) * cos(theta_);
        odom_dt_ = yaw_pf_ - pre_yaw_pf_;
        return ddistance_ > 0.0001? true: false;
    }

    void MapliteClass::initializeParticles()
    {
        std::random_device rd;
        std::mt19937 generator(rd());
        std::uniform_real_distribution<> dist_uniform(0.0, 1.0 / particleNum_);
        std::normal_distribution<> dist_normal(0.0, 4.0);
        
        for(int i = 0; i < particleNum_; i++){
            particle_t p;
            p.x = tx_ + dist_normal(generator);
            p.y = ty_ + dist_normal(generator);
            p.theta = theta_ + 0.1 * dist_normal(generator);
            p.weight = 1.0 / particleNum_;
            particles_.push_back(p);
        }
        
        std::cout<<"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"<<std::endl;
        std::cout<<"++++++++++++++++   Particles are initialized  ++++++++++++++++"<<std::endl;
        std::cout<<"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"<<std::endl;

    }

    void MapliteClass::resampleParticles(){
        std::random_device rd;
        std::mt19937 generator(rd());
        std::uniform_real_distribution<> dist_uniform(0.0, 1.0 / particleNum_);
        std::normal_distribution<> dist_normal(0.0, 1.0);
        // std::sort(particles_.begin(), particles_.end(), compareByWeight);
        std::vector<particle_t> prior;
        double r = dist_uniform(generator);
        double c = particles_[0].weight;
        int i = 0;
        double U = r - 1.0 / particleNum_;
        double ratio = 1.0;
        // std::sort (particles_.begin(), particles_.end(), compareByWeight);
        for(int j = 0; j < particleNum_; j++){
            U = U + 1.0 / particleNum_;
            while(U > c){
                i += 1;
                c += particles_[i].weight;
            }
            prior.push_back(particles_[i]);
        }
        for(int j = particleNum_ * ratio; j < particleNum_; j++){
            particle_t newParticle;
            newParticle.x = estimated_x_ + 0.5 * dist_normal(generator);
            newParticle.y = estimated_y_ + 0.5 * dist_normal(generator);
            newParticle.weight = 1.0 / particleNum_;
            prior.push_back(newParticle);
        }
        particles_.clear();
        for (auto p: prior){
            particles_.push_back(p);
        }
        /**
        std::random_device rd;
        std::mt19937 generator(rd());
        std::vector<particle_t> prior;
        std::uniform_int_distribution<int> particle_index(0, kNumParticles_ - 1);
        int current_index = particle_index(generator);
        float beta = 0.0;
        float max_weight = 0;
        for(int j = 0; j < kNumParticles_; j++){
            if(max_weight < posterior_[j].weight){
                max_weight = posterior_[j].weight;
            }
        }
        
        float max_weight_2 = 2 * max_weight;

        for (int i = 0; i < kNumParticles_; i++) {
            
            std::uniform_real_distribution<float> random_weight(0.0, max_weight_2);
            beta += random_weight(generator);

            while (beta > posterior_[current_index].weight) {
                beta -= posterior_[current_index].weight;
                current_index = (current_index + 1) % kNumParticles_;
            }..
            prior.push_back(posterior_[current_index]);
        }    
        */
    }

    // The noise magnitude should be adoped according to the difference of odometry 
    void MapliteClass::applyActionmodel(){
        std::random_device rd;
        std::mt19937 generator(rd());
        std::uniform_real_distribution<> dist_uniform(0.0, 1.0 / particleNum_);
        std::normal_distribution<> dist_normal(0.0, noise_stdev_);
        // when cassie doesn't move, there are no noises when propagating the particles.
        // otherwise if cassie stands still for a long time, the particles will diverge.
        // under this logic, the greater the delta_odom, the greater the noise.
        // this makes more sense than simply set maximum thereshold.
        double noise_magnitude_x = noise_coefficient_x_ * ddistance_;
        double noise_magnitude_y = noise_coefficient_y_ * ddistance_;
        double noise_magnitude_t = noise_coefficient_t_ * abs(odom_dt_);
        for(int i = 0; i < particleNum_; i++){
            particles_[i].x = particles_[i].x + odom_dx_ + noise_magnitude_x * dist_normal(generator);
            particles_[i].y = particles_[i].y + odom_dy_ + noise_magnitude_y * dist_normal(generator);
            particles_[i].theta = particles_[i].theta + odom_dt_ + noise_magnitude_t * dist_normal(generator);
        }
    }

    void MapliteClass::applySensormodel(const sensor_msgs::PointCloud2 &msg){
        coordinate_t temp;
        temp.val_[0] = estimated_x_;
        temp.val_[1] = estimated_y_;
        std::vector<coordinate_t> tempNpointes;
        landmarks_.FindKNearestByR(temp, tempNpointes, 35, 35, 600); // radius, num set to -1 if not needed
        if(tempNpointes.size() != 0){
            knpoints_.clear();
            knpoints_.resize(tempNpointes.size());
            for(auto p:tempNpointes){
                knpoints_.push_back(p);
            }
            tempNpointes.clear();
        }    

        sensor_msgs::PointCloud cloudData;
        sensor_msgs::convertPointCloud2ToPointCloud(msg, cloudData);
        Eigen::Matrix<double, 3, 3> tempT;
        Eigen::Vector3d tempP;
        std::vector<double> tempWeight(particleNum_);
        for (int i = 0; i < particleNum_; i++){ // for every particle
            tempT << cos(particles_[i].theta), -sin(particles_[i].theta), particles_[i].x,
                     sin(particles_[i].theta), cos(particles_[i].theta), particles_[i].y,
                     0, 0, 1;
            tempWeight[i] = 0;
            int step_size = 2.0;
            for (int j = 0; j < cloudData.points.size() / step_size; j++){ // for every feature
                tempP << cloudData.points[j * step_size].x, cloudData.points[j * step_size].y, 1.0;
                for (int k = 0; k < (int)knpoints_.size(); k++){ //for every mappoint
                    tempWeight[i] += exp(-(pow((tempT*tempP)[0] - knpoints_[k].val_[0], 2) + pow((tempT*tempP)[1] - knpoints_[k].val_[1], 2)) / 2.0);
                }
            }
        }
        double weightSum = 0.0;
        for(auto tp: tempWeight){
            weightSum = weightSum + tp;
        }

        if(weightSum > 0.0){
            for(int i = 0; i < particleNum_; i++){
                particles_[i].weight = tempWeight[i];
            }
        }
        std::cout<<" reference_size: "<<knpoints_.size()<<" || feature_size: "<<cloudData.points.size()<<std::endl;
    }

    void MapliteClass::calculatePosterior(){
        
        double x_mean = 0.0;
        double y_mean = 0.0;
        double cos_mean = 0.0;
        double sin_mean = 0.0;
        double weightSum = 0.0;
        double covarianceSum = 0.0;
        double ratio = 1.0;
        //std::sort (particles_.begin(), particles_.end(), compareByWeight);
        for(int idx = 0; idx < ratio * particleNum_; idx++){
            
            x_mean += particles_[idx].weight * particles_[idx].x;
            y_mean += particles_[idx].weight * particles_[idx].y;
            cos_mean += particles_[idx].weight * cos(particles_[idx].theta);
            sin_mean += particles_[idx].weight * sin(particles_[idx].theta);
            weightSum += particles_[idx].weight;
        }
        
        if(weightSum == 0.0){
            std::cout<<"!!!!!!!!!! weightSum == 0 !!!!!!!!!!!"<<std::endl;
        }

        else{
            std::cout<<"                      weightSum = "<<weightSum<<std::endl;
            estimated_x_ = x_mean / weightSum;
            estimated_y_ = y_mean / weightSum;
            estimated_t_ = std::atan2(sin_mean, cos_mean);

            for (int i = 0; i < particleNum_; i++){
                particles_[i].weight = particles_[i].weight / weightSum;
            }
        }

        // for(int i = 0; i < particleNum_; i++){
        //     covarianceSum += (particles_[i].x - x_mean) * (particles_[i].y - y_mean);
        // }
        // covarianceSum /= (particleNum_ - 1);
        // std::cout<<"       covariance = "<<covarianceSum<<std::endl;
    }
    

    void MapliteClass::publishPointcloud(const sensor_msgs::PointCloud2 &msg){
        sensor_msgs::PointCloud cloudData;
        sensor_msgs::convertPointCloud2ToPointCloud(msg, cloudData);
        cloud.header.stamp = ros::Time::now();
        cloud.header.frame_id = "map";
        cloud.points.resize(cloudData.points.size());

        // yaw_ + theta_ to estimated_t_
        UT_ << cos(estimated_t_), -sin(estimated_t_), estimated_x_,
                sin(estimated_t_), cos(estimated_t_), estimated_y_,
                0, 0, 1;

        Eigen::Vector3d p2;
        for (int i = 0; i < cloudData.points.size(); i++)
        {
            p2 << cloudData.points[i].x, cloudData.points[i].y, 1.0;
            cloud.points[i].x = (UT_*p2)[0];
            cloud.points[i].y = (UT_*p2)[1];
            cloud.points[i].z = 0;
        }

        cloud_pub_.publish(cloud);

        sensor_msgs::PointCloud2 border_msg;
        for (int i = 0; i < cloudData.points.size(); i++)
        {
            cloud.points[i].x = cloudData.points[i].x;
            cloud.points[i].y = cloudData.points[i].y;
            cloud.points[i].z = cloudData.points[i].z;
        }
        sensor_msgs::convertPointCloudToPointCloud2(cloud, border_msg);
        border_msg.header.stamp = msg.header.stamp;
        border_msg.header.frame_id = "new_velodyne";
        // border_pcl_pub_.publish(border_msg);
    }

    void MapliteClass::publishTransformedOdometry(inekf_msgs::State input){
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();

        marker.ns = "pureodom";
        marker.id = 1;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = pos_temp_.x + 3.0;
        marker.pose.position.y = pos_temp_.y + 3.6;
        marker.pose.position.z = -2;
        marker.pose.orientation = quaternion_geo_world_;
        marker.scale.x = 7;
        marker.scale.y = 4;
        marker.scale.z = 2;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;
        pose_pureodom_visualization_pub_.publish(marker);
        
        marker.ns = "accuodom";
        marker.id = 2;
        marker.pose.position.x = odom_temp_.x; 
        marker.pose.position.y = odom_temp_.y;
        marker.pose.orientation = quaternion_geo_;
        marker.scale.x = 8;
        marker.scale.y = 5;
        marker.scale.z = 2.5;
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;
        pose_accuodom_visualization_pub_.publish(marker);       

        
        marker.ns = "estimated";
        marker.id = 3;
        marker.pose.position.x = estimated_x_;
        marker.pose.position.y = estimated_y_;
        marker.pose.position.z = -2;
        geometry_msgs::Quaternion quaternion_geo_est = tf::createQuaternionMsgFromRollPitchYaw(roll_, pitch_, estimated_t_);
        marker.pose.orientation = quaternion_geo_est; //quaternion_geo_world_;
        marker.scale.x = 7;
        marker.scale.y = 4;
        marker.scale.z = 2;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;
        pose_estimate_visualization_pub_.publish(marker);

        geometry_msgs::PoseWithCovarianceStamped pcs;
        pcs.header = input.header;
        pcs.pose.pose.position.x = estimated_x_;
        pcs.pose.pose.position.y = estimated_y_;
        pcs.pose.pose.position.z = input.position.z;
        pcs.pose.pose.orientation = quaternion_geo_est;
        // pose_estimate_geometry_pub_.publish(pcs);

        inekf_msgs::LocalizationState state;
        state.header = input.header;
        state.cassie_position = input.position;
        state.cassie_orientation = input.orientation;
        state.velocity = input.velocity;
        state.position.x = estimated_x_;
        state.position.y = estimated_y_;
        state.position.z = input.position.z;        
        state.orientation = quaternion_geo_est;
        // pose_estimate_inekf_pub_.publish(state);

        //tftree
        tf::Transform odom_to_base;
        odom_to_base.setOrigin(tf::Vector3(estimated_x_, estimated_y_, input.position.z));

        tf::Quaternion q_base;
        tf::quaternionMsgToTF(quaternion_geo_est, q_base);
        odom_to_base.setRotation(q_base);
        
        tf::StampedTransform stamped_base_to_velodyne;
        try{
            base_to_velodyne_listener_.lookupTransform("cassie/pelvis", "velodyne",
                                        input.header.stamp, stamped_base_to_velodyne);
            tf::Transform base_to_velodyne(stamped_base_to_velodyne);

            base_broadcaster_.sendTransform(tf::StampedTransform(odom_to_base,
                                    input.header.stamp, "odom", "cassie/base"));
            velodyne_broadcaster_.sendTransform(tf::StampedTransform(base_to_velodyne,
                                    input.header.stamp, "cassie/base", "new_velodyne"));
        }
        catch (tf::TransformException ex)
        {
        //     ROS_ERROR("%s", ex.what());
        }

    }

    void MapliteClass::publishLandmarks()
    {
        visualization_msgs::MarkerArray landMark_array_msg;
        landMark_array_msg.markers.resize(knpoints_.size());
        for(int i = 0; i < knpoints_.size(); i++)
        {
            landMark_array_msg.markers[i].header.frame_id = "map";
            //landMark_array_msg.markers[i].header.stamp = ros::Time(0);
            landMark_array_msg.markers[i].header.stamp = ros::Time::now();
            landMark_array_msg.markers[i].ns = "landmarks";
            // landMark_array_msg.markers[i].id = i + particleNum_;
            landMark_array_msg.markers[i].id = i;
            landMark_array_msg.markers[i].type = visualization_msgs::Marker::CYLINDER;
            landMark_array_msg.markers[i].action = visualization_msgs::Marker::ADD;
            landMark_array_msg.markers[i].lifetime = ros::Duration();
            landMark_array_msg.markers[i].pose.position.x = knpoints_[i].val_[0];
            landMark_array_msg.markers[i].pose.position.y = knpoints_[i].val_[1];
            landMark_array_msg.markers[i].pose.position.z = 0;
            //landMark_array_msg.markers[i].pose.orientation = quaternion_geo_world_;
            landMark_array_msg.markers[i].scale.x = 1.0;
            landMark_array_msg.markers[i].scale.y = 1.0;
            landMark_array_msg.markers[i].scale.z = 1.0;
            landMark_array_msg.markers[i].color.a = 1.0;
            landMark_array_msg.markers[i].color.r = 0.0f;
            landMark_array_msg.markers[i].color.g = 0.0f;
            landMark_array_msg.markers[i].color.b = 1.0f;
        }
        landmark_pub_.publish(landMark_array_msg);
    }

    void MapliteClass::publishParticles()
    {
        visualization_msgs::MarkerArray particle_array_msg;
        particle_array_msg.markers.resize(particleNum_);
        for(int i = 0; i < particleNum_; i++)
        {
            particle_array_msg.markers[i].header.frame_id = "map";
            //landMark_array_msg.markers[i].header.stamp = ros::Time(0);
            particle_array_msg.markers[i].header.stamp = ros::Time::now();
            particle_array_msg.markers[i].ns = "particles";
            // particle_array_msg.markers[i].id = i + knpoints_.size();
            particle_array_msg.markers[i].id = i;
            particle_array_msg.markers[i].type = visualization_msgs::Marker::ARROW;
            particle_array_msg.markers[i].action = visualization_msgs::Marker::ADD;
            particle_array_msg.markers[i].lifetime = ros::Duration();
            particle_array_msg.markers[i].pose.position.x = particles_[i].x;
            particle_array_msg.markers[i].pose.position.y = particles_[i].y;
            particle_array_msg.markers[i].pose.position.z = 0;
            geometry_msgs::Quaternion quaternion_geo_pfi = tf::createQuaternionMsgFromRollPitchYaw(0, 0, particles_[i].theta);
            particle_array_msg.markers[i].pose.orientation = quaternion_geo_pfi;
            particle_array_msg.markers[i].scale.x = 1.0;
            particle_array_msg.markers[i].scale.y = 1.0;
            particle_array_msg.markers[i].scale.z = 1.0;
            particle_array_msg.markers[i].color.a = 0.9; // particles_[i].weight
            particle_array_msg.markers[i].color.r = 1.0f;
            particle_array_msg.markers[i].color.g = 0.0f;
            particle_array_msg.markers[i].color.b = 0.0f;
        }
        particles_pub_.publish(particle_array_msg);
    }
}
