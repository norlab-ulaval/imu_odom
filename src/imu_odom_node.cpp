#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mutex>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <marvelmind_nav/hedge_pos.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <list>

std::string odomFrame;
std::string robotFrame;
std::string mobileHedgeFrame;
bool rotationOnly;
std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
std::unique_ptr<tf2_ros::Buffer> tfBuffer;
std::mutex mutex;
geometry_msgs::Quaternion odomToStaticHedgeQuaternion;
geometry_msgs::Quaternion staticHedgeToOdomQuaternion;
std::list<sensor_msgs::Imu> imuMeasurements;

void imuCallback(const sensor_msgs::Imu& msg)
{
	mutex.lock();
	imuMeasurements.emplace_back(msg);
	mutex.unlock();
}

void hedgeCallback(const marvelmind_nav::hedge_pos& msg)
{
	try
	{
		mutex.lock();
		while(imuMeasurements.size() >= 2 && (++imuMeasurements.begin())->header.stamp <= msg.stamp)
		{
			imuMeasurements.pop_front();
		}
		sensor_msgs::Imu imuMsg;
		if(!imuMeasurements.empty())
		{
			imuMsg = imuMeasurements.front();
		}
		mutex.unlock();

		if(!imuMsg.header.stamp.isZero())
		{
			// compute mobileHedge orientation in odom frame
			geometry_msgs::TransformStamped mobileHedgeToImuTf = tfBuffer->lookupTransform(imuMsg.header.frame_id, mobileHedgeFrame, msg.stamp, ros::Duration(0.1));
                        geometry_msgs::Quaternion mobileHedgeOrientationInImuFrame = mobileHedgeToImuTf.transform.rotation;
			geometry_msgs::Quaternion mobileHedgeOrientationInOdomFrame;
			geometry_msgs::TransformStamped imuToOdomOrientationTf;
			imuToOdomOrientationTf.transform.rotation = imuMsg.orientation;
			tf2::doTransform(mobileHedgeOrientationInImuFrame, mobileHedgeOrientationInOdomFrame, imuToOdomOrientationTf);
			
			// compute mobileHedge orientation in static hedge frame
			geometry_msgs::Quaternion mobileHedgeOrientationInStaticHedgeFrame;
			geometry_msgs::TransformStamped odomToStaticHedgeOrientationTf;
                        odomToStaticHedgeOrientationTf.transform.rotation = odomToStaticHedgeQuaternion;
			tf2::doTransform(mobileHedgeOrientationInOdomFrame, mobileHedgeOrientationInStaticHedgeFrame, odomToStaticHedgeOrientationTf);

			// compute mobileHedge pose in odom frame
			geometry_msgs::Pose mobileHedgeInStaticHedgeFrame;
			mobileHedgeInStaticHedgeFrame.position.x = msg.x_m;
			mobileHedgeInStaticHedgeFrame.position.y = msg.y_m;
			mobileHedgeInStaticHedgeFrame.position.z = msg.z_m;
			mobileHedgeInStaticHedgeFrame.orientation = mobileHedgeOrientationInStaticHedgeFrame;
			geometry_msgs::Pose mobileHedgeInOdomFrame;
			geometry_msgs::TransformStamped staticHedgeToOdomTf;
			staticHedgeToOdomTf.transform.rotation = staticHedgeToOdomQuaternion;
			tf2::doTransform(mobileHedgeInStaticHedgeFrame, mobileHedgeInOdomFrame, staticHedgeToOdomTf);
			
			// compute robot pose in odom frame
			geometry_msgs::TransformStamped robotToMobileHedgeTf = tfBuffer->lookupTransform(mobileHedgeFrame, robotFrame, msg.stamp, ros::Duration(0.1));
			geometry_msgs::Pose robotPoseInMobileHedgeFrame;
			robotPoseInMobileHedgeFrame.position.x = robotToMobileHedgeTf.transform.translation.x;
			robotPoseInMobileHedgeFrame.position.y = robotToMobileHedgeTf.transform.translation.y;
			robotPoseInMobileHedgeFrame.position.z = robotToMobileHedgeTf.transform.translation.z;
			robotPoseInMobileHedgeFrame.orientation = robotToMobileHedgeTf.transform.rotation;
			geometry_msgs::Pose robotPoseInOdomFrame;
			geometry_msgs::TransformStamped mobileHedgeToOdomTf;
			mobileHedgeToOdomTf.transform.translation.x = mobileHedgeInOdomFrame.position.x;
			mobileHedgeToOdomTf.transform.translation.y = mobileHedgeInOdomFrame.position.y;
			mobileHedgeToOdomTf.transform.translation.z = mobileHedgeInOdomFrame.position.z;
			mobileHedgeToOdomTf.transform.rotation = mobileHedgeInOdomFrame.orientation;
			tf2::doTransform(robotPoseInMobileHedgeFrame, robotPoseInOdomFrame, mobileHedgeToOdomTf);

			// publish tf
			geometry_msgs::TransformStamped robotToOdomTf;
			robotToOdomTf.header.frame_id = odomFrame;
			robotToOdomTf.child_frame_id = robotFrame;
			robotToOdomTf.header.stamp = msg.stamp;
			if(!rotationOnly)
			{
				robotToOdomTf.transform.translation.x = robotPoseInOdomFrame.position.x;
				robotToOdomTf.transform.translation.y = robotPoseInOdomFrame.position.y;
				robotToOdomTf.transform.translation.z = robotPoseInOdomFrame.position.z;
			}
			robotToOdomTf.transform.rotation = robotPoseInOdomFrame.orientation;
			tfBroadcaster->sendTransform(robotToOdomTf);
		}
	}
	catch(const tf2::TransformException& ex)
	{
		ROS_WARN("%s", ex.what());
		return;
	}
}

tf2::Matrix3x3 castToRotationMatrix(std::string matrixStr)
{
	matrixStr.erase(std::remove(matrixStr.begin(), matrixStr.end(), '['), matrixStr.end());
	matrixStr.erase(std::remove(matrixStr.begin(), matrixStr.end(), ']'), matrixStr.end());
	
	double elements[9];
	int counter = 0;	
	size_t previousPos = 0;
	size_t pos = 0;
	while(pos != std::string::npos)
	{
		pos = matrixStr.find(",", previousPos);
		elements[counter++] = std::stod(matrixStr.substr(previousPos, pos-previousPos));
		previousPos = pos+1;
	}
	return tf2::Matrix3x3(elements[0], elements[1], elements[2], elements[3], elements[4], elements[5], elements[6], elements[7], elements[8]);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "imu_odom_node");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	
	pnh.param<std::string>("odom_frame", odomFrame, "odom");
	pnh.param<std::string>("robot_frame", robotFrame, "base_link");
	pnh.param<std::string>("mobile_hedge_frame", mobileHedgeFrame, "hedge");

	pnh.param<bool>("rotation_only", rotationOnly, false);
	
	std::string matrixStr;
	pnh.param<std::string>("odom_to_static_hedge_rotation_matrix", matrixStr, "[[1,0,0],[0,1,0],[0,0,1]]");
	
	tf2::Quaternion quaternion;
	castToRotationMatrix(matrixStr).getRotation(quaternion);
	odomToStaticHedgeQuaternion = tf2::toMsg(quaternion);
	staticHedgeToOdomQuaternion = tf2::toMsg(quaternion.inverse());
	
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
	ros::Subscriber hedgeSubscriber = nh.subscribe("hedge_topic", messageQueueSize, hedgeCallback);
	
	ros::spin();
	
	return 0;
}
