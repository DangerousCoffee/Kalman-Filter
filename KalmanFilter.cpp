#include "KalmanFilter.h"

const float std_weight_pos = 1. / 20;
const float std_weight_vel = 1. / 160;
const std::map<int, float> chi2magicnumber =
		{
			{1, 3.8415},
			{2, 5.9915},
			{3, 7.8147},
			{4, 9.4877},
			{5, 11.070},
			{6, 12.592},
			{7, 14.067},
			{8, 15.507},
			{9, 16.919}
		};



KalmanFilter::KalmanFilter() {
	printf("creating kalman filter object");
	std::cout << std::endl;

	this->motion_mat = Eigen::Matrix<float, double_dim, double_dim>::Zero();
	this->motion_mat.diagonal().array() += 1.0;

	for (int i = 0; i < num_dim; i++) {
		this->motion_mat(i, i + num_dim) = dt;
	}

	this->update_mat = Eigen::Matrix<float, num_dim, double_dim>::Zero();
	this->update_mat.diagonal().array() += 1.0;

	printf("motion mat: ");
	std::cout << std::endl << this->motion_mat << std::endl;
	printf("update mat: ");
	std::cout << std::endl << this->update_mat << std::endl;
};

std::pair<Eigen::Vector<float, double_dim>, Eigen::Matrix<float, double_dim, double_dim>> KalmanFilter::initiate(Eigen::Vector<float, num_dim> measurement) {
	printf("initializing kalman filter");
	std::cout << std::endl;

	Eigen::Vector<float, num_dim> mean_pos = measurement;
	Eigen::Vector<float, num_dim> mean_vel = Eigen::Vector<float, num_dim>::Zero();
	Eigen::Vector<float, double_dim> mean;
	mean << mean_pos, mean_vel;

	printf("initiate mean value:");

	std::cout << std::endl << mean << std::endl;

	float double_std_pos_measurement = 2 * this->std_weight_pos * measurement(3);
	float tten_std_vel_measurement = 10 * this->std_weight_vel * measurement(3);

	Eigen::Vector<float, double_dim> stdeviation = {
		double_std_pos_measurement,
		double_std_pos_measurement,
		1e-2,
		double_std_pos_measurement,
		tten_std_vel_measurement,
		tten_std_vel_measurement,
		1e-5,
		tten_std_vel_measurement
	};
	Eigen::Vector<float, double_dim> std_squared = stdeviation.array().square();

	Eigen::Matrix<float, double_dim, double_dim> covariance = std_squared.asDiagonal();
	
	printf("initiate covariance:");

	std::cout << std::endl << covariance << std::endl;

	std::pair<Eigen::Vector<float, double_dim>, Eigen::Matrix<float, double_dim, double_dim>> result_pair = std::make_pair(mean, covariance);

	return result_pair;
}

KalmanFilter::~KalmanFilter() {
	printf("deleting kalman filter object");
}

