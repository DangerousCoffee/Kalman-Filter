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

std::pair<Eigen::Vector<float, double_dim>, Eigen::Matrix<float, double_dim, double_dim>> KalmanFilter::predict(Eigen::Vector<float, double_dim> mean, Eigen::Matrix<float, double_dim, double_dim> covariance) {
	printf("kalman filter prediction step");
	std::cout << std::endl;

	float std_weight_pos_by_mean = this->std_weight_pos * mean(3);
	float std_weight_vel_by_mean = this->std_weight_vel * mean(3);

	Eigen::Vector<float, num_dim> std_pos = {
		std_weight_pos_by_mean,
		std_weight_pos_by_mean,
		1e-2,
		std_weight_pos_by_mean
	};
	Eigen::Vector<float, num_dim> std_vel = {
		std_weight_vel_by_mean,
		std_weight_vel_by_mean,
		1e-5,
		std_weight_vel_by_mean
	};

	Eigen::Vector<float, double_dim> apriori_est;
	apriori_est << std_pos, std_vel;

	Eigen::Vector<float, double_dim> apriori_squared = apriori_est.array().square();
	Eigen::Matrix<float, double_dim, double_dim> motion_cov = apriori_squared.asDiagonal();

	Eigen::Vector<float, double_dim> predicted_mean = this->motion_mat * mean;

	printf("prediction new mean:");

	std::cout << std::endl << predicted_mean << std::endl;

	Eigen::Matrix<float, double_dim, double_dim> predicted_covariance = this->motion_mat * covariance * this->motion_mat.transpose() + motion_cov;

	printf("prediction new covariance:");

	std::cout << std::endl << predicted_covariance << std::endl;

	std::pair<Eigen::Vector<float, double_dim>, Eigen::Matrix<float, double_dim, double_dim>> result_pair = std::make_pair(predicted_mean, predicted_covariance);

	return result_pair;
}

KalmanFilter::~KalmanFilter() {
	printf("deleting kalman filter object");
}

