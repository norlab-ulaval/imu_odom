# imu_odom
A node that estimates IMU poses based on ICP poses and accelerometer measurements.

IMPORTANT: The orientation quaternion of the IMU messages pusblished in imu_topic must be a good attitude estimate (i.e. not always the identity quaternion). Also, when running this node, do not forget to set the robot_frame parameter to the name of the IMU frame in the norlab_icp_mapper_ros package. Finally, set the map_tf_publish_rate parameter to the rate of the IMU.
