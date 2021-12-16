#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <list>
#include "PositionFilter.h"

std::string odomFrame;
std::string robotFrame;
std::string imuFrame;
float gravity;
bool gravitySet = false;
std::atomic_bool filterDoneInitializing(false);
std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
std::mutex positionFilterMutex;
std::unique_ptr<tf2_ros::Buffer> tfBuffer;
ros::Publisher odomPublisher;
PositionFilter positionFilter;

Eigen::Matrix3f quaternionToRotationMatrix(const Eigen::Vector4f& quaternion)
{
	Eigen::Vector3f epsilon(quaternion[0], quaternion[1], quaternion[2]);
	float eta = quaternion[3];
	Eigen::Matrix3f skewSymmetricEpsilon = Eigen::Matrix3f::Zero(3, 3);
	skewSymmetricEpsilon << 0, -epsilon[2], epsilon[1],
			epsilon[2], 0, -epsilon[0],
			-epsilon[1], epsilon[0], 0;
	return (((eta * eta) - epsilon.dot(epsilon)) * Eigen::Matrix3f::Identity(3, 3)) + (2 * eta * skewSymmetricEpsilon) + (2 * epsilon * epsilon.transpose());
}

void imuCallback(const sensor_msgs::Imu& msg)
{
	try
	{
		Eigen::Vector3f position;
		if(filterDoneInitializing.load())
		{
			Eigen::Vector4f imuToOdomQuaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
			Eigen::Matrix3f imuToOdomRotationMatrix = quaternionToRotationMatrix(imuToOdomQuaternion);
			Eigen::Vector3f accelerationInImuFrame(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
			Eigen::Matrix3f covarianceInImuFrame = (Eigen::Matrix3f()
					<< msg.linear_acceleration_covariance[0], msg.linear_acceleration_covariance[1], msg.linear_acceleration_covariance[2],
					msg.linear_acceleration_covariance[3], msg.linear_acceleration_covariance[4], msg.linear_acceleration_covariance[5],
					msg.linear_acceleration_covariance[6], msg.linear_acceleration_covariance[7], msg.linear_acceleration_covariance[8]).finished();
			Eigen::Vector3f acceleration = imuToOdomRotationMatrix * accelerationInImuFrame;
			Eigen::Matrix3f covariance = imuToOdomRotationMatrix * covarianceInImuFrame * imuToOdomRotationMatrix.transpose();

			if(!gravitySet)
			{
				gravity = acceleration[2];
				gravitySet = true;
			}
			acceleration[2] -= gravity;

			positionFilterMutex.lock();
			positionFilter.processImuMeasurement(acceleration, covariance,
												 std::chrono::time_point<std::chrono::steady_clock>(std::chrono::nanoseconds(msg.header.stamp.toNSec())));
			position = positionFilter.getPosition();
			positionFilterMutex.unlock();
		}
		else
		{
			geometry_msgs::TransformStamped robotToImuTf = tfBuffer->lookupTransform(msg.header.frame_id, robotFrame, msg.header.stamp, ros::Duration(0.1));
			geometry_msgs::Point robotPositionInImuFrame;
			robotPositionInImuFrame.x = robotToImuTf.transform.translation.x;
			robotPositionInImuFrame.y = robotToImuTf.transform.translation.y;
			robotPositionInImuFrame.z = robotToImuTf.transform.translation.z;
			geometry_msgs::Point robotPositionInOdomFrame;
			geometry_msgs::TransformStamped imuToOdomOrientation;
			imuToOdomOrientation.transform.rotation = msg.orientation;
			tf2::doTransform(robotPositionInImuFrame, robotPositionInOdomFrame, imuToOdomOrientation);
			position = Eigen::Vector3f(-robotPositionInOdomFrame.x, -robotPositionInOdomFrame.y, -robotPositionInOdomFrame.z);
		}

		geometry_msgs::TransformStamped robotToImuTf = tfBuffer->lookupTransform(msg.header.frame_id, robotFrame, msg.header.stamp, ros::Duration(0.1));
		geometry_msgs::Pose robotPoseInImuFrame;
		robotPoseInImuFrame.position.x = robotToImuTf.transform.translation.x;
		robotPoseInImuFrame.position.y = robotToImuTf.transform.translation.y;
		robotPoseInImuFrame.position.z = robotToImuTf.transform.translation.z;
		robotPoseInImuFrame.orientation = robotToImuTf.transform.rotation;
		geometry_msgs::Pose robotPoseInOdomFrame;
		geometry_msgs::TransformStamped imuToOdomTf;
		imuToOdomTf.transform.translation.x = position[0];
		imuToOdomTf.transform.translation.y = position[1];
		imuToOdomTf.transform.translation.z = position[2];
		imuToOdomTf.transform.rotation = msg.orientation;
		tf2::doTransform(robotPoseInImuFrame, robotPoseInOdomFrame, imuToOdomTf);

		geometry_msgs::TransformStamped robotToOdomTf;
		robotToOdomTf.header.frame_id = odomFrame;
		robotToOdomTf.header.stamp = msg.header.stamp;
		robotToOdomTf.child_frame_id = robotFrame;
		robotToOdomTf.transform.translation.x = robotPoseInOdomFrame.position.x;
		robotToOdomTf.transform.translation.y = robotPoseInOdomFrame.position.y;
		robotToOdomTf.transform.translation.z = robotPoseInOdomFrame.position.z;
		robotToOdomTf.transform.rotation = robotPoseInOdomFrame.orientation;
		tfBroadcaster->sendTransform(robotToOdomTf);

		nav_msgs::Odometry odomMsg;
		odomMsg.header.stamp = msg.header.stamp;
		odomMsg.header.frame_id = odomFrame;
		odomMsg.child_frame_id = robotFrame;
		odomMsg.pose.pose = robotPoseInOdomFrame;
		odomPublisher.publish(odomMsg);
	}
	catch(const tf2::TransformException& ex)
	{
		ROS_WARN("%s", ex.what());
	}
}

void icpOdomCallback(const nav_msgs::Odometry& msg)
{
	try
	{
		geometry_msgs::TransformStamped imuToRobotTf = tfBuffer->lookupTransform(msg.child_frame_id, imuFrame, msg.header.stamp, ros::Duration(0.1));
		geometry_msgs::Point positionInRobotFrame;
		positionInRobotFrame.x = imuToRobotTf.transform.translation.x;
		positionInRobotFrame.y = imuToRobotTf.transform.translation.y;
		positionInRobotFrame.z = imuToRobotTf.transform.translation.z;
		geometry_msgs::Point positionInMapFrame;
		geometry_msgs::TransformStamped robotToMapTf;
		robotToMapTf.transform.translation.x = msg.pose.pose.position.x;
		robotToMapTf.transform.translation.y = msg.pose.pose.position.y;
		robotToMapTf.transform.translation.z = msg.pose.pose.position.z;
		robotToMapTf.transform.rotation = msg.pose.pose.orientation;
		tf2::doTransform(positionInRobotFrame, positionInMapFrame, robotToMapTf);

		geometry_msgs::Point positionInOdomFrame;
		geometry_msgs::TransformStamped mapToOdomTf = tfBuffer->lookupTransform(odomFrame, msg.header.frame_id, msg.header.stamp, ros::Duration(0.1));
		tf2::doTransform(positionInMapFrame, positionInOdomFrame, mapToOdomTf);

		Eigen::Vector3f position(positionInOdomFrame.x, positionInOdomFrame.y, positionInOdomFrame.z);

		positionFilterMutex.lock();
		positionFilter.processICPMeasurement(position, std::chrono::time_point<std::chrono::steady_clock>(std::chrono::nanoseconds(msg.header.stamp.toNSec())));
		filterDoneInitializing.store(positionFilter.isDoneInitializing());
		positionFilterMutex.unlock();
	}
	catch(const tf2::TransformException& ex)
	{
		ROS_WARN("%s", ex.what());
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "imu_odom_node");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	pnh.param<std::string>("odom_frame", odomFrame, "odom");
	pnh.param<std::string>("robot_frame", robotFrame, "base_link");
	pnh.param<std::string>("imu_frame", imuFrame, "imu_link");

	bool realTime;
	pnh.param<bool>("real_time", realTime, true);

	int messageQueueSize;
	if(realTime)
	{
		tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer);
		messageQueueSize = 1;
	}
	else
	{
		tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(ros::DURATION_MAX));
		messageQueueSize = 0;
	}
	tf2_ros::TransformListener tfListener(*tfBuffer);
	tfBroadcaster = std::unique_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster);

	odomPublisher = nh.advertise<nav_msgs::Odometry>("imu_odom", 50, true);

	ros::Subscriber imuSubscriber = nh.subscribe("imu_topic", messageQueueSize, imuCallback);
	ros::Subscriber icpOdomSubscriber = nh.subscribe("icp_odom", messageQueueSize, icpOdomCallback);

	ros::MultiThreadedSpinner multiThreadedSpinner;
	multiThreadedSpinner.spin();

	return 0;
}

