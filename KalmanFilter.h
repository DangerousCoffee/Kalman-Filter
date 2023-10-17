#pragma once
#include "Eigen/Cholesky"
#include "iostream"
#include "map"

/// size of input measurements vector
#define num_dim 4
#define double_dim num_dim*2
/// motion matrix step coefficient
#define dt 1.0

/**
* Container for mean and covariance pair of double num_dim length, it is used as a return value for functions in the class
*/
struct mean_cov_pair {
	Eigen::Vector<float, double_dim> mean;
	Eigen::Matrix<float, double_dim, double_dim> covariance;
};

/**
* Container for mean and covariance pair of num_dim length, it is used as a return value for project function in the class
*/
struct mean_cov_pair_small {
	Eigen::Vector<float, num_dim> mean;
	Eigen::Matrix<float, num_dim, num_dim> covariance;
};

/**
* A simple implementation of a kalman filter
* 
* The class implements a kalman filter with 2 coordinates, speed and aspect ratio
* The filter class is used to calculate the state of an object in image space with (x, y) coordinates, (a, h) aspect ration and height, (vx, vy, va, vh) respective velocities
* Implementation assumes a direct observation and linear observation model
*/
class KalmanFilter {
public:
	/**
	* Class constructor, initialises motion and update matrices for calculating object state
	*/
	KalmanFilter();
	/**
	* Class destructor, curently does nothing
	*/
	virtual ~KalmanFilter();
	/**
	* A method to set initial object state, computes mean and covariance of the initial state
	* 
	* @param measurement - the initial measurement of the object
	* @returns mean_cov_pair(see above) - the mean and covariance for the initial state 
	*/
	virtual mean_cov_pair initiate(Eigen::Vector<float, num_dim> measurement);
	/**
	* A method to predict next object state, computes mean and covariance of the predicted state
	*
	* @param mean - the mean of the object in previous state
	* @param covariance - the covariance of the object in previous state
	* @returns mean_cov_pair(see above) - the mean and covariance for the predicted state
	*/
	virtual mean_cov_pair predict(Eigen::Vector<float, double_dim> mean, Eigen::Matrix<float, double_dim, double_dim> covariance);
	/**
	* A method to project next object state, computes mean and covariance of the projected state
	* Essentially measurement update
	*
	* @param mean - the mean of the object in previous state
	* @param covariance - the covariance of the object in previous state
	* @returns mean_cov_pair_small(see above) - the mean and covariance for the projected state, these have a size of num_dim
	*/
	virtual mean_cov_pair_small project(Eigen::Vector<float, double_dim> mean, Eigen::Matrix<float, double_dim, double_dim> covariance);
	/**
	* Update step of the kalman filter, used to correct mean and covariance for current state and update kalman gain
	*
	* @param mean - the mean of the object in previous state
	* @param covariance - the covariance of the object in previous state
	* @param measurement - measurement for the current state
	* @returns mean_cov_pair(see above) - the corrected mean and covariance for the current state
	*/
	virtual mean_cov_pair update(Eigen::Vector<float, double_dim> mean, Eigen::Matrix<float, double_dim, double_dim> covariance, Eigen::Vector<float, num_dim> measurement);
	/**
	* Used to compute gating distance between filter values and measurements
	*
	* @param mean - the mean of the object in current state
	* @param covariance - the covariance of the object in current state
	* @param measurements - array of measurements for object states
	* @param only_position - if true only calculate distance based on x and y of the object, otherwise use full measurement vector
	* @returns vector<float> - the gating distance for each measurement
	*/
	virtual Eigen::VectorXf gating_distance(Eigen::Vector<float, double_dim> mean, Eigen::Matrix<float, double_dim, double_dim> covariance, Eigen::VectorX<Eigen::Vector<float, num_dim>> measurements, bool only_position=false);

private:
	/**
	* Motion matrix for calculating kalman filter prediction
	*/
	Eigen::Matrix<float, double_dim, double_dim> motion_mat;
	/**
	* Update matrix for calculating kalman filter projection
	*/
	Eigen::Matrix<float, num_dim, double_dim> update_mat;

	/**
	* Uncertainty coefficients for position and velocity
	*/
	const float std_weight_pos = 1. / 20;
	const float std_weight_vel = 1. / 160;
	/**
	* Gating distance acceptable thresholds
	*/
	const std::map<int, float> chi2magicnumbers = 
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

	/**
	* Method to help with gating distance calculation, calculates the distance itself for only_pos = false
	* 
	* @param mean - num_dim sized vector of current state mean
	* @param covariance - num_dim sized vector of current state covariance
	* @param measurements - vector of num_dim sized measurements
	* @returns vector<float> - vector of gating distance values
	*/
	virtual Eigen::VectorXf gating_solve(Eigen::Vector<float, num_dim> mean, Eigen::Matrix<float, num_dim, num_dim> covariance, Eigen::VectorX<Eigen::Vector<float, num_dim>> measurements);
	/**
	* Method to help with gating distance calculation, calculates the distance itself for only_pos = true (only x and y)
	*
	* @param mean - 2 sized vector of current state mean
	* @param covariance - 2 sized vector of current state covariance
	* @param measurements - vector of 2 sized measurements
	* @returns vector<float> - vector of gating distance values
	*/
	virtual Eigen::VectorXf gating_solve(Eigen::Vector2f mean, Eigen::Matrix2f covariance, Eigen::VectorX<Eigen::Vector2f> measurements);

	/**
	* Method to convert vector of num_dim sized measurements into a matrix
	*
	* @param measurements - vector of num_dim sized measurements
	* @returns matrix<float> - matrix of measurements, each row represents a single measurement vector and has 4 columns
	*/
	virtual Eigen::MatrixX<float> nested_vector_to_matrix(Eigen::VectorX<Eigen::Vector<float, num_dim>>);
	/**
	* Method to convert vector of 2 sized measurements into a matrix
	*
	* @param measurements - vector of 2 sized measurements
	* @returns matrix<float> - matrix of measurements, each row represents a single measurement vector and has 2 columns
	*/
	virtual Eigen::MatrixX<float> nested_vector_to_matrix(Eigen::VectorX<Eigen::Vector2f>);
};