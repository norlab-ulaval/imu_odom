#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <list>
#include <tf2/LinearMath/Quaternion.h>
#include <imu_odom/Inertia.h>

std::string odomFrame;
std::string robotFrame;
std::string imuFrame;
std::string lidarFrame;
bool rotationOnly;
double gravity;
bool gravitySet = false;
std::mutex velocityMutex;
Eigen::Vector3d velocity;
Eigen::Vector3d position;
std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
sensor_msgs::Imu lastImuMeasurement;
nav_msgs::Odometry lastIcpOdom;
std::list<std::pair<ros::Time, Eigen::Vector3d>> deltaVelocities;
std::unique_ptr<tf2_ros::Buffer> tfBuffer;
ros::Publisher inertiaPublisher;
bool mapperStarted = false;

void imuCallback(const sensor_msgs::Imu& msg)
{
	try
	{
		if(lastImuMeasurement.header.stamp.isZero() && !rotationOnly)
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
			position = Eigen::Vector3d(-robotPositionInOdomFrame.x, -robotPositionInOdomFrame.y, -robotPositionInOdomFrame.z);
		}
		else
		{
			geometry_msgs::Vector3 lastAngularVelocityInOdomFrame;
			geometry_msgs::TransformStamped lastImuToOdomOrientation;
			lastImuToOdomOrientation.transform.rotation = lastImuMeasurement.orientation;
			tf2::doTransform(lastImuMeasurement.angular_velocity, lastAngularVelocityInOdomFrame, lastImuToOdomOrientation);
			
			geometry_msgs::Vector3 currentAngularVelocityInOdomFrame;
			geometry_msgs::TransformStamped currentImuToOdomOrientation;
			currentImuToOdomOrientation.transform.rotation = msg.orientation;
			tf2::doTransform(msg.angular_velocity, currentAngularVelocityInOdomFrame, currentImuToOdomOrientation);
			
			Eigen::Vector3d lastAngularVelocity(lastAngularVelocityInOdomFrame.x, lastAngularVelocityInOdomFrame.y, lastAngularVelocityInOdomFrame.z);
			Eigen::Vector3d currentAngularVelocity(currentAngularVelocityInOdomFrame.x, currentAngularVelocityInOdomFrame.y, currentAngularVelocityInOdomFrame.z);
			
			double deltaTime = (msg.header.stamp - lastImuMeasurement.header.stamp).toSec();
			Eigen::Vector3d angularAcceleration = (currentAngularVelocity - lastAngularVelocity) / deltaTime;
			geometry_msgs::Vector3 angularAccelerationInOdomFrame;
			angularAccelerationInOdomFrame.x = angularAcceleration[0];
			angularAccelerationInOdomFrame.y = angularAcceleration[1];
			angularAccelerationInOdomFrame.z = angularAcceleration[2];
			
			geometry_msgs::Vector3 angularVelocityInLidarFrame;
			tf2::Quaternion imuOrientationInOdomFrame;
			tf2::fromMsg(msg.orientation, imuOrientationInOdomFrame);
			geometry_msgs::Quaternion odomOrientationInImuFrame = tf2::toMsg(imuOrientationInOdomFrame.inverse());
			geometry_msgs::Quaternion odomOrientationInLidarFrame;
			geometry_msgs::TransformStamped imuToLidarTf = tfBuffer->lookupTransform(lidarFrame, imuFrame, msg.header.stamp, ros::Duration(0.1));
			tf2::doTransform(odomOrientationInImuFrame, odomOrientationInLidarFrame, imuToLidarTf);
			geometry_msgs::TransformStamped odomToLidarOrientationTf;
			odomToLidarOrientationTf.transform.rotation = odomOrientationInLidarFrame;
			tf2::doTransform(currentAngularVelocityInOdomFrame, angularVelocityInLidarFrame, odomToLidarOrientationTf);
			
			geometry_msgs::Vector3 angularAccelerationInLidarFrame;
			tf2::doTransform(angularAccelerationInOdomFrame, angularAccelerationInLidarFrame, odomToLidarOrientationTf);
			
			geometry_msgs::Vector3 lidarLinearVelocityInLidarFrame, lidarLinearAccelerationInLidarFrame;
			if(rotationOnly)
			{
				geometry_msgs::TransformStamped robotToImuTf = tfBuffer->lookupTransform(msg.header.frame_id, robotFrame, msg.header.stamp, ros::Duration(0.1));
				geometry_msgs::Quaternion robotOrientationInOdomFrame;
				geometry_msgs::TransformStamped imuToOdomOrientation;
				imuToOdomOrientation.transform.rotation = msg.orientation;
				tf2::doTransform(robotToImuTf.transform.rotation, robotOrientationInOdomFrame, imuToOdomOrientation);
				
				geometry_msgs::TransformStamped robotToOdomTf;
				robotToOdomTf.header.frame_id = odomFrame;
				robotToOdomTf.header.stamp = msg.header.stamp;
				robotToOdomTf.child_frame_id = robotFrame;
				robotToOdomTf.transform.rotation = robotOrientationInOdomFrame;
				tfBroadcaster->sendTransform(robotToOdomTf);
			}
			else
			{
				geometry_msgs::Vector3 lastAccelerationInOdomFrame;
				tf2::doTransform(lastImuMeasurement.linear_acceleration, lastAccelerationInOdomFrame, lastImuToOdomOrientation);
				
				geometry_msgs::Vector3 currentAccelerationInOdomFrame;
				tf2::doTransform(msg.linear_acceleration, currentAccelerationInOdomFrame, currentImuToOdomOrientation);
				
				if(!gravitySet)
				{
					gravity = lastAccelerationInOdomFrame.z;
					gravitySet = true;
				}
				lastAccelerationInOdomFrame.z -= gravity;
				currentAccelerationInOdomFrame.z -= gravity;
				
				Eigen::Vector3d lastAcceleration(lastAccelerationInOdomFrame.x, lastAccelerationInOdomFrame.y, lastAccelerationInOdomFrame.z);
				Eigen::Vector3d currentAcceleration(currentAccelerationInOdomFrame.x, currentAccelerationInOdomFrame.y, currentAccelerationInOdomFrame.z);
				Eigen::Vector3d deltaVelocity = 0.5 * (lastAcceleration + currentAcceleration) * deltaTime;
				
				Eigen::Vector3d lastVelocity;
				velocityMutex.lock();
				deltaVelocities.emplace_back(std::make_pair(msg.header.stamp, deltaVelocity));
				if(!mapperStarted)
				{
					deltaVelocity = Eigen::Vector3d::Zero();
				}
				lastVelocity = velocity;
				velocity += deltaVelocity;
				velocityMutex.unlock();
				
				position += (lastVelocity + (0.5 * deltaVelocity)) * deltaTime;
				
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
				
				geometry_msgs::TransformStamped lidarToImuTf = tfBuffer->lookupTransform(imuFrame, lidarFrame, msg.header.stamp, ros::Duration(0.1));
				geometry_msgs::Point lidarPositionInImuFrame;
				lidarPositionInImuFrame.x = lidarToImuTf.transform.translation.x;
				lidarPositionInImuFrame.y = lidarToImuTf.transform.translation.y;
				lidarPositionInImuFrame.z = lidarToImuTf.transform.translation.z;
				geometry_msgs::Point lidarPositionInOdomFrame;
				tf2::doTransform(lidarPositionInImuFrame, lidarPositionInOdomFrame, imuToOdomTf);
				Eigen::Vector3d lidarPosition(lidarPositionInOdomFrame.x, lidarPositionInOdomFrame.y, lidarPositionInOdomFrame.z);
				
				Eigen::Vector3d linearVelocity = lastVelocity + deltaVelocity;
				Eigen::Vector3d lidarLinearVelocity = linearVelocity + currentAngularVelocity.cross(lidarPosition - position);
				geometry_msgs::Vector3 lidarLinearVelocityInOdomFrame;
				lidarLinearVelocityInOdomFrame.x = lidarLinearVelocity[0];
				lidarLinearVelocityInOdomFrame.y = lidarLinearVelocity[1];
				lidarLinearVelocityInOdomFrame.z = lidarLinearVelocity[2];
				tf2::doTransform(lidarLinearVelocityInOdomFrame, lidarLinearVelocityInLidarFrame, odomToLidarOrientationTf);
				
				Eigen::Vector3d lidarLinearAcceleration = currentAcceleration +
						currentAngularVelocity.cross(currentAngularVelocity.cross(lidarPosition - position)) +
														  angularAcceleration.cross(lidarPosition - position);
				geometry_msgs::Vector3 lidarLinearAccelerationInOdomFrame;
				lidarLinearAccelerationInOdomFrame.x = lidarLinearAcceleration[0];
				lidarLinearAccelerationInOdomFrame.y = lidarLinearAcceleration[1];
				lidarLinearAccelerationInOdomFrame.z = lidarLinearAcceleration[2];
				tf2::doTransform(lidarLinearAccelerationInOdomFrame, lidarLinearAccelerationInLidarFrame, odomToLidarOrientationTf);
			}
			
			imu_odom::Inertia inertiaMsg;
			inertiaMsg.header = msg.header;
			inertiaMsg.linear_velocity = lidarLinearVelocityInLidarFrame;
			inertiaMsg.linear_acceleration = lidarLinearAccelerationInLidarFrame;
			inertiaMsg.angular_velocity = angularVelocityInLidarFrame;
			inertiaMsg.angular_acceleration = angularAccelerationInLidarFrame;
			inertiaPublisher.publish(inertiaMsg);
		}
		
		lastImuMeasurement = msg;
	}
	catch(const tf2::TransformException& ex)
	{
		ROS_WARN("%s", ex.what());
		return;
	}
}

void icpOdomCallback(const nav_msgs::Odometry& msg)
{
	try
	{
		if(!lastIcpOdom.header.stamp.isZero())
		{
			geometry_msgs::TransformStamped currentImuToRobotTf = tfBuffer->lookupTransform(msg.child_frame_id, imuFrame, msg.header.stamp,
																							ros::Duration(0.1));
			geometry_msgs::Point currentPositionInRobotFrame;
			currentPositionInRobotFrame.x = currentImuToRobotTf.transform.translation.x;
			currentPositionInRobotFrame.y = currentImuToRobotTf.transform.translation.y;
			currentPositionInRobotFrame.z = currentImuToRobotTf.transform.translation.z;
			geometry_msgs::Point currentPositionInMapFrame;
			geometry_msgs::TransformStamped currentRobotToMapTf;
			currentRobotToMapTf.transform.translation.x = msg.pose.pose.position.x;
			currentRobotToMapTf.transform.translation.y = msg.pose.pose.position.y;
			currentRobotToMapTf.transform.translation.z = msg.pose.pose.position.z;
			currentRobotToMapTf.transform.rotation = msg.pose.pose.orientation;
			tf2::doTransform(currentPositionInRobotFrame, currentPositionInMapFrame, currentRobotToMapTf);
			
			geometry_msgs::TransformStamped lastImuToRobotTf = tfBuffer->lookupTransform(lastIcpOdom.child_frame_id, imuFrame, lastIcpOdom.header.stamp,
																						 ros::Duration(0.1));
			geometry_msgs::Point lastPositionInRobotFrame;
			lastPositionInRobotFrame.x = lastImuToRobotTf.transform.translation.x;
			lastPositionInRobotFrame.y = lastImuToRobotTf.transform.translation.y;
			lastPositionInRobotFrame.z = lastImuToRobotTf.transform.translation.z;
			geometry_msgs::Point lastPositionInMapFrame;
			geometry_msgs::TransformStamped lastRobotToMapTf;
			lastRobotToMapTf.transform.translation.x = lastIcpOdom.pose.pose.position.x;
			lastRobotToMapTf.transform.translation.y = lastIcpOdom.pose.pose.position.y;
			lastRobotToMapTf.transform.translation.z = lastIcpOdom.pose.pose.position.z;
			lastRobotToMapTf.transform.rotation = lastIcpOdom.pose.pose.orientation;
			tf2::doTransform(lastPositionInRobotFrame, lastPositionInMapFrame, lastRobotToMapTf);
			
			double deltaTime = (msg.header.stamp - lastIcpOdom.header.stamp).toSec();
			geometry_msgs::Vector3 currentVelocityInMapFrame;
			currentVelocityInMapFrame.x = (currentPositionInMapFrame.x - lastPositionInMapFrame.x) / deltaTime;
			currentVelocityInMapFrame.y = (currentPositionInMapFrame.y - lastPositionInMapFrame.y) / deltaTime;
			currentVelocityInMapFrame.z = (currentPositionInMapFrame.z - lastPositionInMapFrame.z) / deltaTime;
			geometry_msgs::Vector3 currentVelocityInOdomFrame;
			geometry_msgs::TransformStamped currentMapToOdomTf = tfBuffer->lookupTransform(odomFrame, msg.header.frame_id, msg.header.stamp,
																						   ros::Duration(0.1));
			tf2::doTransform(currentVelocityInMapFrame, currentVelocityInOdomFrame, currentMapToOdomTf);
			Eigen::Vector3d currentVelocity(currentVelocityInOdomFrame.x, currentVelocityInOdomFrame.y, currentVelocityInOdomFrame.z);
			
			velocityMutex.lock();
			mapperStarted = true;
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
		
		lastIcpOdom = msg;
	}
	catch(const tf2::TransformException& ex)
	{
		ROS_WARN("%s", ex.what());
		return;
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
	pnh.param<std::string>("lidar_frame", lidarFrame, "rslidar16");
	pnh.param<bool>("rotation_only", rotationOnly, false);
	
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
	
	ros::Subscriber imuSubscriber = nh.subscribe("imu_topic", messageQueueSize, imuCallback);
	ros::Subscriber icpOdomSubscriber = nh.subscribe("icp_odom", messageQueueSize, icpOdomCallback);
	
	inertiaPublisher = nh.advertise<imu_odom::Inertia>("inertia_topic", messageQueueSize);
	
	ros::spin();
	
	return 0;
}
