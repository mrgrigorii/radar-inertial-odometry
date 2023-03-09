#include "utility.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include <geometry_msgs/TwistWithCovarianceStamped.h>

using gtsam::symbol_shorthand::B; 
using gtsam::symbol_shorthand::V; 
using gtsam::symbol_shorthand::X; 

class IMUPreintegration : public ParamServer
{
public:
    std::mutex mtx;
    ros::Subscriber subImu;
    ros::Subscriber subOdometry;
    ros::Publisher pubImuOdometry;
    ros::Publisher pubImuPath; 
    nav_msgs::Path ImupathMsg; 
    bool systemInitialized = false;
    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise2;
    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;
    std::deque<sensor_msgs::Imu> imuQueOpt;
    std::deque<sensor_msgs::Imu> imuQueImu;

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
    ros::Subscriber sub_twist_;
    std::deque<geometry_msgs::TwistWithCovarianceStamped> velQue;
    geometry_msgs::TwistWithCovarianceStamped curVel_msgs;
    std::mutex velLock;

    double StartTime;
    bool StartFlag;

    IMUPreintegration()
    {
        subImu = nh.subscribe<sensor_msgs::Imu>("/imu/data", 2000, &IMUPreintegration::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subOdometry = nh.subscribe<nav_msgs::Odometry>("/current_pose", 5, &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        pubImuOdometry = nh.advertise<nav_msgs::Odometry>("/current_pose_incremental", 2000);
        pubImuPath = nh.advertise<nav_msgs::Path>("/current_path_incremental", 2000);
        sub_twist_ = nh.subscribe<geometry_msgs::TwistWithCovarianceStamped>("/reve/twist", 5, &IMUPreintegration::reveCallBack, this, ros::TransportHints().tcpNoDelay());

        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuAccNoise, 2); 
        p->gyroscopeCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuGyrNoise, 2);     
        p->integrationCovariance = gtsam::Matrix33::Identity(3, 3) * pow(1e-4, 2); 
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());
        priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
        priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
        priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);                                                           // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished());               // rad,rad,rad,m, m, m
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();
        priorVelNoise2 = gtsam::noiseModel::Isotropic::Sigma(6, 1e0); // m/s

        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); 
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); 

        StartFlag = true;
    }

    bool reveIsValid(const geometry_msgs::TwistWithCovarianceStamped &velmsgs)
    {
        if (abs(velmsgs.twist.twist.linear.x) < 30 && abs(velmsgs.twist.twist.linear.y) < 30 && abs(velmsgs.twist.twist.linear.z) < 30)
        {
            return true;
        }
        return false;
    }

    void reveCallBack(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &velmsgs)
    {

        std::lock_guard<std::mutex> lock1(velLock);
        if (reveIsValid(*velmsgs))
        {
            velQue.push_back(*velmsgs);
            auto vxx = velmsgs->twist.twist.linear.y;
            auto vyy = velmsgs->twist.twist.linear.x;
            auto vzz = velmsgs->twist.twist.linear.z;
        }
    }

    void resetOptimization()
    {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1; // 0.1
        optParameters.relinearizeSkip = 1;
        //optParameters.factorization = gtsam::ISAM2Params::QR; 
        optimizer = gtsam::ISAM2(optParameters);
        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;
        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

    void resetParams()
    {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr &odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        std::lock_guard<std::mutex> lock1(velLock);

        double currentCorrectionTime = ROS_TIME(odomMsg);
        if (imuQueOpt.empty())
            return;
        float p_x = odomMsg->pose.pose.position.x;
        float p_y = odomMsg->pose.pose.position.y;
        float p_z = odomMsg->pose.pose.position.z;
        float r_x = odomMsg->pose.pose.orientation.x;
        float r_y = odomMsg->pose.pose.orientation.y;
        float r_z = odomMsg->pose.pose.orientation.z;
        float r_w = odomMsg->pose.pose.orientation.w;
        gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));

        if (systemInitialized == false)
        {
            resetOptimization();
            while (!imuQueOpt.empty())
            {
                if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t)
                {
                    lastImuT_opt = ROS_TIME(&imuQueOpt.front());
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

        if (key == 100)
        {
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key - 1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key - 1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key - 1)));

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

        while (!imuQueOpt.empty())
        {
            sensor_msgs::Imu *thisImu = &imuQueOpt.front();
            double imuTime = ROS_TIME(thisImu);

            if (imuTime < currentCorrectionTime - delta_t)
            {

                double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);

                imuIntegratorOpt_->integrateMeasurement(
                    gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                    gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y, thisImu->angular_velocity.z), dt);
                lastImuT_opt = imuTime;
                imuQueOpt.pop_front(); 
            }
            else
                break;
        }


        while (!velQue.empty())
        {
            curVel_msgs = velQue.front();
            double VelTime = ROS_TIME(&curVel_msgs);
            if (VelTime < currentCorrectionTime)
            {
                velQue.pop_front();
            }
            else
                break;
        }

        auto vx = curVel_msgs.twist.twist.linear.y;
        auto vy = curVel_msgs.twist.twist.linear.x;
        auto vz = curVel_msgs.twist.twist.linear.z;
        gtsam::Velocity3 cur_Vel = gtsam::Vector3(vx, vy, vz);
        ROS_WARN("after velocity xyz is %f %f %f", vx, vy, vz);

        gtsam::PriorFactor<gtsam::Vector3> velocity_factor(V(key), cur_Vel, priorVelNoise2);
        graphFactors.add(velocity_factor);

        const gtsam::PreintegratedImuMeasurements &preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                                                                            gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));

        gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, correctionNoise);
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
        prevPose_ = result.at<gtsam::Pose3>(X(key));
        prevVel_ = result.at<gtsam::Vector3>(V(key));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));

        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

        if (failureDetection(prevVel_, prevBias_))
        {
            resetParams();
            return;
        }
        prevStateOdom = prevState_; 
        prevBiasOdom = prevBias_;
        double lastImuQT = -1;
        while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t)
        {
            lastImuQT = ROS_TIME(&imuQueImu.front());
            imuQueImu.pop_front();
        }
        if (!imuQueImu.empty())
        {

            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            for (int i = 0; i < (int)imuQueImu.size(); ++i)
            {
                sensor_msgs::Imu *thisImu = &imuQueImu[i];
                double imuTime = ROS_TIME(thisImu);
                double dt = (lastImuQT < 0) ? (1.0 / 500.0) : (imuTime - lastImuQT);
                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                        gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y, thisImu->angular_velocity.z), dt);
                lastImuQT = imuTime;
            }
        }

        ++key;
        doneFirstOpt = true;
    }

    bool failureDetection(const gtsam::Vector3 &velCur, const gtsam::imuBias::ConstantBias &biasCur)
    {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30)
        {
            ROS_WARN("Large velocity, reset IMU-preintegration!");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0)
        {
            ROS_WARN("Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr &imu_raw)
    {
        std::lock_guard<std::mutex> lock(mtx);
        sensor_msgs::Imu thisImu = imuConverter(*imu_raw);
        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);
        if (doneFirstOpt == false)
            return;

        double imuTime = ROS_TIME(&thisImu);

        if (StartFlag == true)
        {
            StartTime = imuTime;
            StartFlag = false;
        }

        double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
        lastImuT_imu = imuTime;

        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
                                                gtsam::Vector3(thisImu.angular_velocity.x, thisImu.angular_velocity.y, thisImu.angular_velocity.z), dt);

        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

        nav_msgs::Odometry odometry;
        odometry.header.stamp = thisImu.header.stamp;
        odometry.header.frame_id = "map";
        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

        if (imuTime - StartTime > 3.0)
        {
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
            pubImuOdometry.publish(odometry);

            geometry_msgs::PoseStamped path_odom;
            path_odom.pose.position.x = odometry.pose.pose.position.x;
            path_odom.pose.position.y = odometry.pose.pose.position.y;
            path_odom.pose.position.z = odometry.pose.pose.position.z;
            path_odom.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
            path_odom.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
            path_odom.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
            path_odom.pose.orientation.w = lidarPose.rotation().toQuaternion().w();

            path_odom.header.stamp = thisImu.header.stamp;
            path_odom.header.frame_id = "map";
            ImupathMsg.poses.push_back(path_odom);

            ImupathMsg.header.frame_id = "map";
            ImupathMsg.header.stamp = thisImu.header.stamp;
            pubImuPath.publish(ImupathMsg);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roboat_loam");
    IMUPreintegration ImuP;
    ROS_INFO("\033[1;32m----> Optimization Started.\033[0m");
    // 使用多线程触发 给此进程开辟四个线程进行回调 利用并发的方式提高速度
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}
