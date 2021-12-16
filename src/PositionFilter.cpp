#include "PositionFilter.h"
#include <Eigen/LU>

// TODO: save initial orientation to cancel it

PositionFilter::PositionFilter() :
		x(Eigen::Matrix<float, 9, 1>::Zero()),
		P(Eigen::Matrix<float, 9, 9>::Identity() * EPSILON),
		H_IMU(Eigen::Matrix<float, 3, 9>::Zero()),
		R_ICP(Eigen::Matrix<float, 3, 3>::Identity() * ICP_COVARIANCE)
{
	F = Eigen::Matrix<float, 9, 9>::Zero();
	F(3, 0) = 1;
	F(4, 1) = 1;
	F(5, 2) = 1;
	F(6, 3) = 1;
	F(7, 4) = 1;
	F(8, 5) = 1;

	Q = Eigen::Matrix<float, 9, 9>::Zero();
	Q(0, 0) = PREDICTION_PROCESS_COVARIANCE;
	Q(1, 1) = PREDICTION_PROCESS_COVARIANCE;
	Q(2, 2) = PREDICTION_PROCESS_COVARIANCE;

	H_ICP = Eigen::Matrix<float, 3, 9>::Zero();
	H_ICP(0, 0) = 1;
	H_ICP(1, 1) = 1;
	H_ICP(2, 2) = 1;
}

void PositionFilter::processICPMeasurement(const Eigen::Vector3f& measurement, const std::chrono::time_point<std::chrono::steady_clock>& time)
{
	if(times.size() == 3)
	{
		float deltaT0 = std::chrono::duration<float>(time - times[2]).count();
		float deltaT1 = std::chrono::duration<float>(times[2] - times[1]).count();
		float deltaT2 = std::chrono::duration<float>(times[1] - times[0]).count();

		float coefficient = 1 + (deltaT0 / deltaT1) + ((deltaT0 * deltaT0) / (2 * deltaT1 * deltaT1));
		F(0, 0) = coefficient;
		F(1, 1) = coefficient;
		F(2, 2) = coefficient;

		coefficient = -((deltaT0 / deltaT1) + ((deltaT0 * deltaT0) / (2 * deltaT1 * deltaT1)) + ((deltaT0 * deltaT0) / (2 * deltaT1 * deltaT2)));
		F(0, 3) = coefficient;
		F(1, 4) = coefficient;
		F(2, 5) = coefficient;

		coefficient = ((deltaT0 * deltaT0) / (2 * deltaT1 * deltaT2));
		F(0, 6) = coefficient;
		F(1, 7) = coefficient;
		F(2, 8) = coefficient;

		x = F * x;
		P = (F * P * F.transpose()) + Q;
		Eigen::Vector3f y = measurement - (H_ICP * x);
		Eigen::Matrix3f S = (H_ICP * P * H_ICP.transpose()) + R_ICP;
		Eigen::Matrix<float, 9, 3> K = P * H_ICP.transpose() * S.inverse();
		x = x + (K * y);
		P = (Eigen::Matrix<float, 9, 9>::Identity() - (K * H_ICP)) * P;

		times[0] = times[1];
		times[1] = times[2];
		times[2] = time;
	}
	else
	{
		x(6, 0) = x(3, 0);
		x(7, 0) = x(4, 0);
		x(8, 0) = x(5, 0);

		x(3, 0) = x(0, 0);
		x(4, 0) = x(1, 0);
		x(5, 0) = x(2, 0);

		x(0, 0) = measurement(0, 0);
		x(1, 0) = measurement(1, 0);
		x(2, 0) = measurement(2, 0);

		times.push_back(time);
	}
}

void PositionFilter::processImuMeasurement(const Eigen::Vector3f& measurement, const Eigen::Matrix3f& covariance, const std::chrono::time_point<std::chrono::steady_clock>& time)
{
	if(times.size() == 3)
	{
		float deltaT0 = std::chrono::duration<float>(time - times[2]).count();
		float deltaT1 = std::chrono::duration<float>(times[2] - times[1]).count();
		float deltaT2 = std::chrono::duration<float>(times[1] - times[0]).count();

		float coefficient = 1 + (deltaT0 / deltaT1) + ((deltaT0 * deltaT0) / (2 * deltaT1 * deltaT1));
		F(0, 0) = coefficient;
		F(1, 1) = coefficient;
		F(2, 2) = coefficient;

		coefficient = -((deltaT0 / deltaT1) + ((deltaT0 * deltaT0) / (2 * deltaT1 * deltaT1)) + ((deltaT0 * deltaT0) / (2 * deltaT1 * deltaT2)));
		F(0, 3) = coefficient;
		F(1, 4) = coefficient;
		F(2, 5) = coefficient;

		coefficient = ((deltaT0 * deltaT0) / (2 * deltaT1 * deltaT2));
		F(0, 6) = coefficient;
		F(1, 7) = coefficient;
		F(2, 8) = coefficient;

		coefficient = 2 / (deltaT1 * (deltaT1 + deltaT2));
		H_IMU(0, 0) = coefficient;
		H_IMU(1, 1) = coefficient;
		H_IMU(2, 2) = coefficient;

		coefficient = -((2 / (deltaT1 * (deltaT1 + deltaT2))) + (2 / (deltaT2 * (deltaT1 + deltaT2))));
		H_IMU(0, 3) = coefficient;
		H_IMU(1, 4) = coefficient;
		H_IMU(2, 5) = coefficient;

		coefficient = 2 / (deltaT2 * (deltaT1 + deltaT2));
		H_IMU(0, 6) = coefficient;
		H_IMU(1, 7) = coefficient;
		H_IMU(2, 8) = coefficient;

		x = F * x;
		P = (F * P * F.transpose()) + Q;
		Eigen::Vector3f y = measurement - (H_IMU * x);
		Eigen::Matrix3f S = (H_IMU * P * H_IMU.transpose()) + covariance;
		Eigen::Matrix<float, 9, 3> K = P * H_IMU.transpose() * S.inverse();
		x = x + (K * y);
		P = (Eigen::Matrix<float, 9, 9>::Identity() - (K * H_IMU)) * P;

		times[0] = times[1];
		times[1] = times[2];
		times[2] = time;
	}
}

bool PositionFilter::isDoneInitializing() const
{
	return times.size() == 3;
}

Eigen::Vector3f PositionFilter::getPosition() const
{
	return x.block<3, 1>(0, 0);
}
