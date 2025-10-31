#include "utility.h"

#include "dataManager/dataSaver.hpp"

class TransformFusion : public ParamServer {
public:
    std::mutex mtx;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subImuOdometry;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuOdometry;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubImuPath;

    Eigen::Affine3f lidarOdomAffine;
    Eigen::Affine3f imuOdomAffineFront;
    Eigen::Affine3f imuOdomAffineBack;

    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfMap2Odom;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfOdom2BaseLink;
    tf2::Stamped<tf2::Transform> lidar2Baselink;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    double lidarOdomTime = -1;
    deque<nav_msgs::msg::Odometry> imuOdomQueue;

    TransformFusion(const rclcpp::NodeOptions & options) : ParamServer("liloc_transformFusion", options) 
    {

        tfBuffer = std::make_shared<tf2_ros::Buffer>(get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

        tfMap2Odom = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        tfOdom2BaseLink = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        geometry_msgs::msg::TransformStamped static_tf;
        static_tf.header.stamp = get_clock()->now();
        static_tf.header.frame_id = mapFrame;
        static_tf.child_frame_id = odometryFrame;
        static_tf.transform.translation.x = 0.0;
        static_tf.transform.translation.y = 0.0;
        static_tf.transform.translation.z = 0.0;
        static_tf.transform.rotation.x = 0.0;
        static_tf.transform.rotation.y = 0.0;
        static_tf.transform.rotation.z = 0.0;
        static_tf.transform.rotation.w = 1.0;
        tf_static_broadcaster_->sendTransform(static_tf);

        if(lidarFrame != baselinkFrame)
        {
            try
            {
                tf2::fromMsg(tfBuffer->lookupTransform(
                    lidarFrame, baselinkFrame, tf2::TimePointZero, tf2::durationFromSec(3.0)), lidar2Baselink);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_ERROR(get_logger(), "%s", ex.what());
            }
        }

        subLaserOdometry = create_subscription<nav_msgs::msg::Odometry>("liloc/mapping/odometry", 5, 
                    std::bind(&TransformFusion::lidarOdometryHandler, this, std::placeholders::_1));

        subImuOdometry = create_subscription<nav_msgs::msg::Odometry>(odomTopic+"_incremental", 2000,
                    std::bind(&TransformFusion::imuOdometryHandler, this, std::placeholders::_1));

        pubImuOdometry = create_publisher<nav_msgs::msg::Odometry>(odomTopic, 2000);
        pubImuPath = create_publisher<nav_msgs::msg::Path>("liloc/imu/path", 1);
    }

    Eigen::Affine3f odom2affine(nav_msgs::msg::Odometry odom) {
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        tf2::Quaternion orientation;
        tf2::fromMsg(odom.pose.pose.orientation, orientation);
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }

    void lidarOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
        std::lock_guard<std::mutex> lock(mtx);

        lidarOdomAffine = odom2affine(*odomMsg);

        lidarOdomTime = ROS_TIME(odomMsg->header.stamp);
    }

    void imuOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {

        std::lock_guard<std::mutex> lock(mtx);

        imuOdomQueue.push_back(*odomMsg);

        // get latest odometry (at current IMU stamp)
        if (lidarOdomTime == -1)
            return;
        while (!imuOdomQueue.empty()) {
            if (ROS_TIME(imuOdomQueue.front().header.stamp) <= lidarOdomTime)
                imuOdomQueue.pop_front();
            else
                break;
        }
        Eigen::Affine3f imuOdomAffineFront = odom2affine(imuOdomQueue.front());
        Eigen::Affine3f imuOdomAffineBack = odom2affine(imuOdomQueue.back());
        Eigen::Affine3f imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
        Eigen::Affine3f imuOdomAffineLast = lidarOdomAffine * imuOdomAffineIncre;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch, yaw);
        
        nav_msgs::msg::Odometry laserOdometry = imuOdomQueue.back();
        laserOdometry.pose.pose.position.x = x;
        laserOdometry.pose.pose.position.y = y;
        laserOdometry.pose.pose.position.z = z;

        tf2::Quaternion quat_tf;
        quat_tf.setRPY(roll, pitch, yaw);
        geometry_msgs::msg::Quaternion quat_msg;
        tf2::convert(quat_tf, quat_msg);
        laserOdometry.pose.pose.orientation = quat_msg;
        pubImuOdometry->publish(laserOdometry);

        tf2::Transform tCur(tf2::Quaternion(laserOdometry.pose.pose.orientation.x, laserOdometry.pose.pose.orientation.y, laserOdometry.pose.pose.orientation.z, laserOdometry.pose.pose.orientation.w), 
                                tf2::Vector3(laserOdometry.pose.pose.position.x, laserOdometry.pose.pose.position.y, laserOdometry.pose.pose.position.z));
        if(lidarFrame != baselinkFrame)
            tCur = tCur * lidar2Baselink;

        rclcpp::Time t(static_cast<uint32_t>(lidarOdomTime * 1e9));
        tf2::TimePoint time_point = tf2_ros::fromRclcpp(t);
        tf2::Stamped<tf2::Transform> tmp_odom_to_base(tCur, time_point, odometryFrame);
        geometry_msgs::msg::TransformStamped odom_2_baselink;
        tf2::convert(tmp_odom_to_base, odom_2_baselink);
        odom_2_baselink.child_frame_id = baselinkFrame;
        tfOdom2BaseLink->sendTransform(odom_2_baselink);

        static nav_msgs::msg::Path imuPath;
        static double last_path_time = -1;
        double imuTime = ROS_TIME(imuOdomQueue.back().header.stamp);
        if (imuTime - last_path_time > 0.1) {
            last_path_time = imuTime;
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
            pose_stamped.header.frame_id = odometryFrame;
            pose_stamped.pose = laserOdometry.pose.pose;
            imuPath.poses.push_back(pose_stamped);
            while(!imuPath.poses.empty() && ROS_TIME(imuPath.poses.front().header.stamp) < lidarOdomTime - 1.0)
                imuPath.poses.erase(imuPath.poses.begin());

            if (pubImuPath->get_subscription_count() != 0) {
                imuPath.header.stamp = imuOdomQueue.back().header.stamp;
                imuPath.header.frame_id = odometryFrame;
                pubImuPath->publish(imuPath);
            }
        }
    }
};

class IMUPreintegration : public ParamServer {
public:
    std::mutex mtx;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuOdometry;

    bool systemInitialized = false;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;


    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

    std::deque<sensor_msgs::msg::Imu> imuQueOpt;
    std::deque<sensor_msgs::msg::Imu> imuQueImu;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    bool doneFirstOpt = false;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;

    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    const double delta_t = 0;

    int key = 1;
    
    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

    IMUPreintegration(const rclcpp::NodeOptions & options) :
            ParamServer("liloc_imu_preintegration", options) 
    {
        subImu = create_subscription<sensor_msgs::msg::Imu>(imuTopic, 2000, 
                    std::bind(&IMUPreintegration::imuHandler, this, std::placeholders::_1));

        subOdometry = create_subscription<nav_msgs::msg::Odometry>("liorf/mapping/odometry_incremental", 5,
                    std::bind(&IMUPreintegration::odometryHandler, this, std::placeholders::_1));

        pubImuOdometry = create_publisher<nav_msgs::msg::Odometry>(odomTopic+"_incremental", 2000);

        std::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2); // acc white noise in continuous
        p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
        p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

        priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
        priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
        priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();
        
        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);  
    }

    void resetOptimization() {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

    void resetParams() {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }

    void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
        std::lock_guard<std::mutex> lock(mtx);

        double currentCorrectionTime = ROS_TIME(odomMsg->header.stamp);

        if (imuQueOpt.empty())
            return;

        float p_x = odomMsg->pose.pose.position.x;
        float p_y = odomMsg->pose.pose.position.y;
        float p_z = odomMsg->pose.pose.position.z;
        float r_x = odomMsg->pose.pose.orientation.x;
        float r_y = odomMsg->pose.pose.orientation.y;
        float r_z = odomMsg->pose.pose.orientation.z;
        float r_w = odomMsg->pose.pose.orientation.w;
        bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
        gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));

        if (systemInitialized == false) {
            resetOptimization();

            while (!imuQueOpt.empty()) {
                if (ROS_TIME((&imuQueOpt.front())->header.stamp) < currentCorrectionTime - delta_t) {
                    lastImuT_opt = ROS_TIME((&imuQueOpt.front())->header.stamp);
                    imuQueOpt.pop_front();
                }
                else
                    break;
            }

            prevPose_ = lidarPose.compose(lidar2Imu);
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
            graphFactors.add(priorPose);

            prevVel_ = gtsam::Vector3(0, 0, 0);
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
            graphFactors.add(priorVel);

            prevBias_ = gtsam::imuBias::ConstantBias();
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
            graphFactors.add(priorBias);

            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);

            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
            
            key = 1;
            systemInitialized = true;
            return;
        }

        if (key == 100) {
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key-1)));

            resetOptimization();

            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors.add(priorPose);

            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            graphFactors.add(priorVel);

            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            graphFactors.add(priorBias);

            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);

            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            key = 1;
        }


        while (!imuQueOpt.empty()) {
            sensor_msgs::msg::Imu *thisImu = &imuQueOpt.front();
            double imuTime = ROS_TIME(thisImu->header.stamp);
            if (imuTime < currentCorrectionTime - delta_t) {
                double dt = (lastImuT_opt < 0) ? (1.0 / imuRate) : (imuTime - lastImuT_opt);
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                
                lastImuT_opt = imuTime;
                imuQueOpt.pop_front();
            }
            else
                break;
        }

        const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);

        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                         gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));

        gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
        graphFactors.add(pose_factor);

        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);

        optimizer.update(graphFactors, graphValues);
        optimizer.update();
        graphFactors.resize(0);
        graphValues.clear();

        gtsam::Values result = optimizer.calculateEstimate();
        prevPose_  = result.at<gtsam::Pose3>(X(key));
        prevVel_   = result.at<gtsam::Vector3>(V(key));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));

        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

        if (failureDetection(prevVel_, prevBias_)) {
            resetParams();
            return;
        }

        prevStateOdom = prevState_;
        prevBiasOdom  = prevBias_;

        double lastImuQT = -1;
        while (!imuQueImu.empty() && ROS_TIME((imuQueImu.front()).header.stamp) < currentCorrectionTime - delta_t) {
            lastImuQT = ROS_TIME((imuQueImu.front()).header.stamp);
            imuQueImu.pop_front();
        }

        if (!imuQueImu.empty()) {

            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            for (int i = 0; i < (int)imuQueImu.size(); ++i) {
                sensor_msgs::msg::Imu *thisImu = &imuQueImu[i];
                double imuTime = ROS_TIME(thisImu->header.stamp);
                double dt = (lastImuQT < 0) ? (1.0 / imuRate) :(imuTime - lastImuQT);

                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                lastImuQT = imuTime;
            }
        }

        ++key;
        doneFirstOpt = true;
    }

    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur) {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30) {
            RCLCPP_WARN(this->get_logger(), "Large velocity, reset IMU-preintegration!");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0) {
            RCLCPP_WARN(this->get_logger(), "Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }

    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imu_raw) {

        RCLCPP_INFO_ONCE(this->get_logger(), "Got first IMU message from topic: %s", imuTopic.c_str());

        std::lock_guard<std::mutex> lock(mtx);

        sensor_msgs::msg::Imu thisImu = imuConverter(*imu_raw);

        if (correct) {
            thisImu.linear_acceleration.x = thisImu.linear_acceleration.x * kAccScale;
            thisImu.linear_acceleration.y = thisImu.linear_acceleration.y * kAccScale;
            thisImu.linear_acceleration.z = thisImu.linear_acceleration.z * kAccScale;
        }

        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);

        if (doneFirstOpt == false)
            return;

        double imuTime = ROS_TIME((&thisImu)->header.stamp);
        double dt = (lastImuT_imu < 0) ? (1.0 / imuRate) : (imuTime - lastImuT_imu);
        lastImuT_imu = imuTime;

        // integrate this single imu message
        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
                                                gtsam::Vector3(thisImu.angular_velocity.x,    thisImu.angular_velocity.y,    thisImu.angular_velocity.z), dt);

        // predict odometry
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

        // publish odometry
        nav_msgs::msg::Odometry odometry;
        odometry.header.stamp = thisImu.header.stamp;
        odometry.header.frame_id = odometryFrame;
        odometry.child_frame_id = "odom_imu";

        // transform imu pose to ldiar
        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

        odometry.pose.pose.position.x = lidarPose.translation().x();
        odometry.pose.pose.position.y = lidarPose.translation().y();
        odometry.pose.pose.position.z = lidarPose.translation().z();
        odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
        
        odometry.twist.twist.linear.x = currentState.velocity().x();
        odometry.twist.twist.linear.y = currentState.velocity().y();
        odometry.twist.twist.linear.z = currentState.velocity().z();
        odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        pubImuOdometry->publish(odometry);
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor e;
    
    auto ImuP = std::make_shared<IMUPreintegration>(options);
    auto TF = std::make_shared<TransformFusion>(options);
    e.add_node(ImuP);
    e.add_node(TF);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> IMU Preintegration Started.\033[0m");
    
    e.spin();
    
    rclcpp::shutdown();
    return 0;
}
