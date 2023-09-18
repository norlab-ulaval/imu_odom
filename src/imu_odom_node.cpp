#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Core>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <list>

class ImuOdomNode : public rclcpp::Node
{
public:
    ImuOdomNode() : Node("imu_odom_node")
    {
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("robot_frame", "base_link");
        this->declare_parameter<std::string>("imu_frame", "imu_link");
        this->declare_parameter<bool>("rotation_only", false);
        this->declare_parameter<bool>("real_time", true);

        this->get_parameter("odom_frame", odomFrame);
        this->get_parameter("robot_frame", robotFrame);
        this->get_parameter("imu_frame", imuFrame);
        this->get_parameter("rotation_only", rotationOnly);
        bool realTime;
        this->get_parameter("real_time", realTime);

        int messageQueueSize;
        if(realTime)
        {
            tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(this->get_clock()));
            messageQueueSize = 1;
        }
        else
        {
            tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(this->get_clock(), std::chrono::seconds(1000000)));
            messageQueueSize = 0;
        }
        tfListener = std::unique_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(*tfBuffer));
        tfBroadcaster = std::unique_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster(*this));

        odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("imu_odom", 50);

        imuSubscription = this->create_subscription<sensor_msgs::msg::Imu>("imu_topic", messageQueueSize, std::bind(&ImuOdomNode::imuCallback, this,
                                                                                             std::placeholders::_1));
        icpOdomSubscription = this->create_subscription<nav_msgs::msg::Odometry>("icp_odom", messageQueueSize, std::bind(&ImuOdomNode::icpOdomCallback, this,
                                                                                                std::placeholders::_1));
    }

private:
    const rclcpp::Duration MAX_TIME_WITHOUT_ICP_ODOM = std::chrono::milliseconds(2000);

    std::string odomFrame;
    std::string robotFrame;
    std::string imuFrame;
    bool rotationOnly;
    double gravity;
    bool gravitySet = false;
    Eigen::Vector3d velocity;
    std::mutex velocityMutex;
    Eigen::Vector3d position;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    sensor_msgs::msg::Imu lastImuMeasurement;
    nav_msgs::msg::Odometry lastIcpOdom;
    std::mutex lastIcpOdomMutex;
    std::list<std::pair<rclcpp::Time, Eigen::Vector3d>> deltaVelocities;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr icpOdomSubscription;

    void imuCallback(const sensor_msgs::msg::Imu& msg)
    {
        try
        {
            lastIcpOdomMutex.lock();
            bool underMaxTimeWithoutIcpOdom = (rclcpp::Time(msg.header.stamp) - rclcpp::Time(lastIcpOdom.header.stamp)) <= MAX_TIME_WITHOUT_ICP_ODOM;
            lastIcpOdomMutex.unlock();

            if(lastImuMeasurement.header.stamp.sec == 0 && lastImuMeasurement.header.stamp.nanosec == 0)
            {
                geometry_msgs::msg::TransformStamped robotToImuTf = tfBuffer->lookupTransform(msg.header.frame_id, robotFrame, msg.header.stamp, std::chrono::milliseconds(100));
                geometry_msgs::msg::Point robotPositionInImuFrame;
                robotPositionInImuFrame.x = robotToImuTf.transform.translation.x;
                robotPositionInImuFrame.y = robotToImuTf.transform.translation.y;
                robotPositionInImuFrame.z = robotToImuTf.transform.translation.z;
                geometry_msgs::msg::Point robotPositionInOdomFrame;
                geometry_msgs::msg::TransformStamped imuToOdomOrientation;
                imuToOdomOrientation.transform.rotation = msg.orientation;
                tf2::doTransform(robotPositionInImuFrame, robotPositionInOdomFrame, imuToOdomOrientation);
                position = Eigen::Vector3d(-robotPositionInOdomFrame.x, -robotPositionInOdomFrame.y, -robotPositionInOdomFrame.z);
            }
            else if(underMaxTimeWithoutIcpOdom && !rotationOnly)
            {
                geometry_msgs::msg::Vector3 lastAccelerationInOdomFrame;
                geometry_msgs::msg::TransformStamped lastImuToOdomOrientation;
                lastImuToOdomOrientation.transform.rotation = lastImuMeasurement.orientation;
                tf2::doTransform(lastImuMeasurement.linear_acceleration, lastAccelerationInOdomFrame, lastImuToOdomOrientation);

                geometry_msgs::msg::Vector3 currentAccelerationInOdomFrame;
                geometry_msgs::msg::TransformStamped currentImuToOdomOrientation;
                currentImuToOdomOrientation.transform.rotation = msg.orientation;
                tf2::doTransform(msg.linear_acceleration, currentAccelerationInOdomFrame, currentImuToOdomOrientation);

                if(!gravitySet)
                {
                    gravity = lastAccelerationInOdomFrame.z;
                    gravitySet = true;
                }
                lastAccelerationInOdomFrame.z -= gravity;
                currentAccelerationInOdomFrame.z -= gravity;

                double deltaTime = (rclcpp::Time(msg.header.stamp) - rclcpp::Time(lastImuMeasurement.header.stamp)).seconds();
                Eigen::Vector3d lastAcceleration(lastAccelerationInOdomFrame.x, lastAccelerationInOdomFrame.y, lastAccelerationInOdomFrame.z);
                Eigen::Vector3d currentAcceleration(currentAccelerationInOdomFrame.x, currentAccelerationInOdomFrame.y, currentAccelerationInOdomFrame.z);
                Eigen::Vector3d deltaVelocity = 0.5 * (lastAcceleration + currentAcceleration) * deltaTime;

                Eigen::Vector3d lastVelocity;
                velocityMutex.lock();
                deltaVelocities.emplace_back(std::make_pair(msg.header.stamp, deltaVelocity));
                lastVelocity = velocity;
                velocity += deltaVelocity;
                velocityMutex.unlock();

                position += (lastVelocity + (0.5 * deltaVelocity)) * deltaTime;
            }

            geometry_msgs::msg::TransformStamped robotToImuTf = tfBuffer->lookupTransform(msg.header.frame_id, robotFrame, msg.header.stamp, std::chrono::milliseconds(100));
            geometry_msgs::msg::Pose robotPoseInImuFrame;
            robotPoseInImuFrame.position.x = robotToImuTf.transform.translation.x;
            robotPoseInImuFrame.position.y = robotToImuTf.transform.translation.y;
            robotPoseInImuFrame.position.z = robotToImuTf.transform.translation.z;
            robotPoseInImuFrame.orientation = robotToImuTf.transform.rotation;
            geometry_msgs::msg::Pose robotPoseInOdomFrame;
            geometry_msgs::msg::TransformStamped imuToOdomTf;
            imuToOdomTf.transform.translation.x = position[0];
            imuToOdomTf.transform.translation.y = position[1];
            imuToOdomTf.transform.translation.z = position[2];
            imuToOdomTf.transform.rotation = msg.orientation;
            tf2::doTransform(robotPoseInImuFrame, robotPoseInOdomFrame, imuToOdomTf);

            geometry_msgs::msg::TransformStamped robotToOdomTf;
            robotToOdomTf.header.frame_id = odomFrame;
            robotToOdomTf.header.stamp = msg.header.stamp;
            robotToOdomTf.child_frame_id = robotFrame;
            robotToOdomTf.transform.translation.x = robotPoseInOdomFrame.position.x;
            robotToOdomTf.transform.translation.y = robotPoseInOdomFrame.position.y;
            robotToOdomTf.transform.translation.z = robotPoseInOdomFrame.position.z;
            robotToOdomTf.transform.rotation = robotPoseInOdomFrame.orientation;
            tfBroadcaster->sendTransform(robotToOdomTf);

            nav_msgs::msg::Odometry odomMsg;
            odomMsg.header.stamp = msg.header.stamp;
            odomMsg.header.frame_id = odomFrame;
            odomMsg.child_frame_id = robotFrame;
            odomMsg.pose.pose = robotPoseInOdomFrame;
            odomPublisher->publish(odomMsg);
        }
        catch(const tf2::TransformException& ex)
        {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        }
        lastImuMeasurement = msg;
    }

    void icpOdomCallback(const nav_msgs::msg::Odometry& msg)
    {
        try
        {
            if(lastIcpOdom.header.stamp.sec != 0 || lastIcpOdom.header.stamp.nanosec != 0)
            {
                geometry_msgs::msg::TransformStamped currentImuToRobotTf = tfBuffer->lookupTransform(msg.child_frame_id, imuFrame, msg.header.stamp,
                                                                                                std::chrono::milliseconds(100));
                geometry_msgs::msg::Point currentPositionInRobotFrame;
                currentPositionInRobotFrame.x = currentImuToRobotTf.transform.translation.x;
                currentPositionInRobotFrame.y = currentImuToRobotTf.transform.translation.y;
                currentPositionInRobotFrame.z = currentImuToRobotTf.transform.translation.z;
                geometry_msgs::msg::Point currentPositionInMapFrame;
                geometry_msgs::msg::TransformStamped currentRobotToMapTf;
                currentRobotToMapTf.transform.translation.x = msg.pose.pose.position.x;
                currentRobotToMapTf.transform.translation.y = msg.pose.pose.position.y;
                currentRobotToMapTf.transform.translation.z = msg.pose.pose.position.z;
                currentRobotToMapTf.transform.rotation = msg.pose.pose.orientation;
                tf2::doTransform(currentPositionInRobotFrame, currentPositionInMapFrame, currentRobotToMapTf);

                geometry_msgs::msg::TransformStamped lastImuToRobotTf = tfBuffer->lookupTransform(lastIcpOdom.child_frame_id, imuFrame, lastIcpOdom.header.stamp,
                                                                                             std::chrono::milliseconds(100));
                geometry_msgs::msg::Point lastPositionInRobotFrame;
                lastPositionInRobotFrame.x = lastImuToRobotTf.transform.translation.x;
                lastPositionInRobotFrame.y = lastImuToRobotTf.transform.translation.y;
                lastPositionInRobotFrame.z = lastImuToRobotTf.transform.translation.z;
                geometry_msgs::msg::Point lastPositionInMapFrame;
                geometry_msgs::msg::TransformStamped lastRobotToMapTf;
                lastRobotToMapTf.transform.translation.x = lastIcpOdom.pose.pose.position.x;
                lastRobotToMapTf.transform.translation.y = lastIcpOdom.pose.pose.position.y;
                lastRobotToMapTf.transform.translation.z = lastIcpOdom.pose.pose.position.z;
                lastRobotToMapTf.transform.rotation = lastIcpOdom.pose.pose.orientation;
                tf2::doTransform(lastPositionInRobotFrame, lastPositionInMapFrame, lastRobotToMapTf);

                double deltaTime = (rclcpp::Time(msg.header.stamp) - rclcpp::Time(lastIcpOdom.header.stamp)).seconds();
                geometry_msgs::msg::Vector3 currentVelocityInMapFrame;
                currentVelocityInMapFrame.x = (currentPositionInMapFrame.x - lastPositionInMapFrame.x) / deltaTime;
                currentVelocityInMapFrame.y = (currentPositionInMapFrame.y - lastPositionInMapFrame.y) / deltaTime;
                currentVelocityInMapFrame.z = (currentPositionInMapFrame.z - lastPositionInMapFrame.z) / deltaTime;
                geometry_msgs::msg::Vector3 currentVelocityInOdomFrame;
                geometry_msgs::msg::TransformStamped currentMapToOdomTf = tfBuffer->lookupTransform(odomFrame, msg.header.frame_id, msg.header.stamp,
                                                                                               std::chrono::milliseconds(100));
                tf2::doTransform(currentVelocityInMapFrame, currentVelocityInOdomFrame, currentMapToOdomTf);
                Eigen::Vector3d currentVelocity(currentVelocityInOdomFrame.x, currentVelocityInOdomFrame.y, currentVelocityInOdomFrame.z);

                velocityMutex.lock();
                while(!deltaVelocities.empty() && deltaVelocities.front().first <= msg.header.stamp)
                {
                    deltaVelocities.pop_front();
                }
                for(auto it = deltaVelocities.begin(); it != deltaVelocities.end(); it++)
                {
                    currentVelocity += it->second;
                }
                velocity = currentVelocity;
                velocityMutex.unlock();
            }

        }
        catch(const tf2::TransformException& ex)
        {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        }
        lastIcpOdomMutex.lock();
        lastIcpOdom = msg;
        lastIcpOdomMutex.unlock();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ImuOdomNode>());
    rclcpp::shutdown();
	return 0;
}

