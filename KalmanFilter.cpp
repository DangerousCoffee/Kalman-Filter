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

std::pair<Eigen::Vector<float, num_dim>, Eigen::Matrix<float, num_dim, num_dim>> KalmanFilter::project(Eigen::Vector<float, double_dim> mean, Eigen::Matrix<float, double_dim, double_dim> covariance) {
	printf("kalman filter project step");
	std::cout << std::endl;
	float std_weight_pos_by_mean = std_weight_pos * mean(3);

	Eigen::Vector<float, num_dim> std = {
		std_weight_pos_by_mean,
		std_weight_pos_by_mean,
		1e-1,
		std_weight_pos_by_mean
	};

	Eigen::Vector<float, num_dim> std_squared = std.array().square();
	Eigen::Matrix<float, num_dim, num_dim> innovation_cov = std_squared.asDiagonal();

	Eigen::Vector<float, num_dim> projected_mean = this->update_mat * mean;

	printf("projected mean:");
	std::cout << std::endl << projected_mean << std::endl;

	Eigen::Matrix<float, num_dim, num_dim> projected_covariance = this->update_mat * covariance * this->update_mat.transpose() + innovation_cov;
	
	printf("projected cov:");
	std::cout << std::endl << projected_covariance << std::endl;

	std::pair<Eigen::Vector<float, num_dim>, Eigen::Matrix<float, num_dim, num_dim>> result_pair = std::make_pair(projected_mean, projected_covariance);

	return result_pair;
}

std::pair<Eigen::Vector<float, double_dim>, Eigen::Matrix<float, double_dim, double_dim>> KalmanFilter::update(Eigen::Vector<float, double_dim> mean, Eigen::Matrix<float, double_dim, double_dim> covariance, Eigen::Vector<float, num_dim> measurement) {
	printf("kalman filter update step");
	std::cout << std::endl;
	std::pair<Eigen::Vector<float, num_dim>, Eigen::Matrix<float, num_dim, num_dim>> projected_pair = this->project(mean, covariance);

	Eigen::Vector<float, num_dim> projected_mean = projected_pair.first;
	Eigen::Matrix<float, num_dim, num_dim> projected_cov = projected_pair.second;

	// LDLT decomposition works here but need more testing, maybe use LLT instead
	Eigen::LDLT<Eigen::Matrix<float, num_dim, num_dim>> chol_factor = projected_cov.ldlt();

	Eigen::Matrix<float, double_dim, num_dim> kalman_gain = chol_factor.solve((covariance * update_mat.transpose()).transpose()).transpose();

	Eigen::Vector<float, num_dim> innovation = measurement.array() - projected_mean.array();

	Eigen::Vector<float, double_dim> intermediate_result = innovation.transpose() * kalman_gain.transpose();
	Eigen::Vector<float, double_dim> new_mean = mean.array() + (intermediate_result).array();

	printf("updated new mean:");
	std::cout << std::endl << new_mean << std::endl;

	Eigen::Matrix<float, double_dim, double_dim> new_cov = covariance - kalman_gain * projected_cov * kalman_gain.transpose();

	printf("updated new covariance:");
	std::cout << std::endl << new_cov << std::endl;

	std::pair<Eigen::Vector<float, double_dim>, Eigen::Matrix<float, double_dim, double_dim>> result_pair = std::make_pair(new_mean, new_cov);

	return result_pair;
}

Eigen::VectorXf KalmanFilter::gating_distance(Eigen::Vector<float, double_dim> mean, Eigen::Matrix<float, double_dim, double_dim> covariance, Eigen::VectorX<Eigen::Vector<float, 4>> measurements, bool only_position) {
	printf("Calculating gating distance");
	std::cout << std::endl;
	std::pair<Eigen::Vector<float, num_dim>, Eigen::Matrix<float, num_dim, num_dim>> projected_pair = this->project(mean, covariance);

	Eigen::Vector<float, num_dim> projected_mean = projected_pair.first;
	Eigen::Matrix<float, num_dim, num_dim> projected_covariance = projected_pair.second;


	if (only_position) {
		Eigen::Vector2f only_pos_mean = projected_mean.head(2);
		Eigen::Matrix2f only_pos_cov = projected_covariance(Eigen::seq(0, 1), Eigen::seq(0, 1));
		Eigen::VectorX<Eigen::Vector2f> only_pos_measurements(measurements.size());
		for (int i = 0; i < measurements.size(); i++) {
			only_pos_measurements[i] = measurements(i)(Eigen::seq(0, 1));
		}
		return this->gating_solve(only_pos_mean, only_pos_cov, only_pos_measurements);
	}


	return this->gating_solve(projected_mean, projected_covariance, measurements);
}

Eigen::VectorXf KalmanFilter::gating_solve(Eigen::Vector<float, num_dim> mean, Eigen::Matrix<float, num_dim, num_dim> covariance, Eigen::VectorX<Eigen::Vector<float, num_dim>> measurements) {
	Eigen::LLT<Eigen::Matrix<float, num_dim, num_dim>> chol_factor = covariance.llt();

	Eigen::VectorX<Eigen::Vector<float, num_dim>> d(measurements.size());
	for (int i = 0; i < measurements.size(); i++) {
		d[i] = measurements[i] - mean;
	}

	Eigen::MatrixX<float> d_matrix = this->nested_vector_to_matrix(d);

	Eigen::MatrixXf z = chol_factor.matrixL().solve(d_matrix.transpose());

	z.transposeInPlace();
	Eigen::MatrixXf squared_z(z.rows(), z.cols());
	for (int i = 0; i < z.size(); i++) {
		squared_z(i) = z(i) * z(i);
	}

	Eigen::VectorXf squared_maha = squared_z.rowwise().sum();

	printf("squared maha: ");
	std::cout << std::endl << squared_maha << std::endl;

	return squared_maha;
}

Eigen::VectorXf KalmanFilter::gating_solve(Eigen::Vector2f mean, Eigen::Matrix2f covariance, Eigen::VectorX<Eigen::Vector2f> measurements) {
	Eigen::LLT<Eigen::Matrix2f> chol_factor = covariance.llt();

	Eigen::VectorX<Eigen::Vector2f> d(measurements.size());
	for (int i = 0; i < measurements.size(); i++) {
		d[i] = measurements[i] - mean;
	}

	Eigen::MatrixX<float> d_matrix = this->nested_vector_to_matrix(d);

	Eigen::MatrixXf z = chol_factor.matrixL().solve(d_matrix.transpose());

	z.transposeInPlace();
	Eigen::MatrixXf squared_z(z.rows(), z.cols());
	for (int i = 0; i < z.size(); i++) {
		squared_z(i) = z(i) * z(i);
	}

	Eigen::VectorXf squared_maha = squared_z.rowwise().sum();

	printf("squared maha: ");
	std::cout << std::endl << squared_maha << std::endl;

	return squared_maha;
}

Eigen::MatrixX<float> KalmanFilter::nested_vector_to_matrix(Eigen::VectorX<Eigen::Vector<float, num_dim>> vector) {
	Eigen::MatrixX<float> matrix(vector.size(), vector[0].size());

	for (int i = 0; i < vector.size(); i++) {
		matrix.row(i) = vector[i];
	}

	return matrix;
}

Eigen::MatrixX<float> KalmanFilter::nested_vector_to_matrix(Eigen::VectorX<Eigen::Vector2f> vector) {
	Eigen::MatrixX<float> matrix(vector.size(), vector[0].size());

	for (int i = 0; i < vector.size(); i++) {
		matrix.row(i) = vector[i];
	}

	return matrix;
}

KalmanFilter::~KalmanFilter() {
	printf("deleting kalman filter object");
}

