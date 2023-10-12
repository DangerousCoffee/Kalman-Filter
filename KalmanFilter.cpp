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

KalmanFilter::~KalmanFilter() {
	printf("deleting kalman filter object");
}

