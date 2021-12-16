#ifndef IMU_ODOM_POSITIONFILTER_H
#define IMU_ODOM_POSITIONFILTER_H

#include <Eigen/Core>
#include <vector>
#include <chrono>

class PositionFilter
{
private:
	const float ICP_COVARIANCE = 1;
	const float PREDICTION_PROCESS_COVARIANCE = 1e-6;
	const float EPSILON = 1e-6;

	Eigen::Matrix<float, 9, 1> x;
	Eigen::Matrix<float, 9, 9> F;
	Eigen::Matrix<float, 9, 9> P;
	Eigen::Matrix<float, 9, 9> Q;
	Eigen::Matrix<float, 3, 9> H_ICP;
	Eigen::Matrix<float, 3, 9> H_IMU;
	Eigen::Matrix<float, 3, 3> R_ICP;
	std::vector<std::chrono::time_point<std::chrono::steady_clock>> times;

public:
	PositionFilter();

	void processICPMeasurement(const Eigen::Vector3f& measurement, const std::chrono::time_point<std::chrono::steady_clock>& time);
	void processImuMeasurement(const Eigen::Vector3f& measurement, const Eigen::Matrix3f& covariance, const std::chrono::time_point<std::chrono::steady_clock>& time);
	bool isDoneInitializing() const;
	Eigen::Vector3f getPosition() const;
};

#endif
