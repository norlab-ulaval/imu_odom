#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <list>

std::string odomFrame;
std::string mapFrame;
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

void imuCallback(const sensor_msgs::Imu& msg)
{
	if(!lastImuMeasurement.header.stamp.isZero())
	{
		geometry_msgs::Vector3 lastAccelerationInOdomFrame;
		geometry_msgs::TransformStamped imuToOdom;
		imuToOdom.transform.rotation = lastImuMeasurement.orientation;
		tf2::doTransform(lastImuMeasurement.linear_acceleration, lastAccelerationInOdomFrame, imuToOdom);
		
		if(!gravitySet)
		{
			gravity = lastAccelerationInOdomFrame.z;
			gravitySet = true;
		}
		lastAccelerationInOdomFrame.z -= gravity;
		
		double deltaTime = (msg.header.stamp - lastImuMeasurement.header.stamp).toSec();
		Eigen::Vector3d lastAcceleration(lastAccelerationInOdomFrame.x, lastAccelerationInOdomFrame.y, lastAccelerationInOdomFrame.z);
		Eigen::Vector3d deltaVelocity = lastAcceleration * deltaTime;
		
		Eigen::Vector3d lastVelocity;
		velocityMutex.lock();
		deltaVelocities.emplace_back(std::make_pair(msg.header.stamp, deltaVelocity));
		lastVelocity = velocity;
		velocity += deltaVelocity;
		velocityMutex.unlock();
		
		position += (lastVelocity + (0.5 * deltaVelocity)) * deltaTime;
		
		geometry_msgs::TransformStamped imuToOdomTf;
		imuToOdomTf.header.frame_id = odomFrame;
		imuToOdomTf.header.stamp = msg.header.stamp;
		imuToOdomTf.child_frame_id = msg.header.frame_id;
		imuToOdomTf.transform.translation.x = position[0];
		imuToOdomTf.transform.translation.y = position[1];
		imuToOdomTf.transform.translation.z = position[2];
		imuToOdomTf.transform.rotation = msg.orientation;
		tfBroadcaster->sendTransform(imuToOdomTf);
	}
	
	lastImuMeasurement = msg;
}

void icpOdomCallback(const nav_msgs::Odometry& msg)
{
	if(!lastIcpOdom.header.stamp.isZero())
	{
		double deltaTime = (msg.header.stamp - lastIcpOdom.header.stamp).toSec();
		geometry_msgs::Vector3 currentVelocityInMapFrame;
		currentVelocityInMapFrame.x = (msg.pose.pose.position.x - lastIcpOdom.pose.pose.position.x) / deltaTime;
		currentVelocityInMapFrame.y = (msg.pose.pose.position.y - lastIcpOdom.pose.pose.position.y) / deltaTime;
		currentVelocityInMapFrame.z = (msg.pose.pose.position.z - lastIcpOdom.pose.pose.position.z) / deltaTime;
		geometry_msgs::Vector3 currentVelocityInOdomFrame;
		geometry_msgs::TransformStamped currentMapToOdom = tfBuffer->lookupTransform(odomFrame, mapFrame, msg.header.stamp, ros::Duration(0.1));
		tf2::doTransform(currentVelocityInMapFrame, currentVelocityInOdomFrame, currentMapToOdom);
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
	
	lastIcpOdom = msg;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "imu_odom_node");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	
	pnh.param<std::string>("odom_frame", odomFrame, "odom");
	pnh.param<std::string>("map_frame", mapFrame, "map");
	
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
	
	ros::spin();
	
	return 0;
}
