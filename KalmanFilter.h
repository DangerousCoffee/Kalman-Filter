#pragma once
#include "Eigen/Cholesky"
#include "iostream"
#include "map"

using Eigen::Matrix;

#define num_dim 4
#define double_dim num_dim*2
#define dt 1.0

class KalmanFilter {
public:
	KalmanFilter();
	virtual ~KalmanFilter();
	virtual std::pair<Eigen::Vector<float, double_dim>, Eigen::Matrix<float, double_dim, double_dim>> initiate(Eigen::Vector<float, num_dim>);
	virtual std::pair<Eigen::Vector<float, double_dim>, Eigen::Matrix<float, double_dim, double_dim>> predict(Eigen::Vector<float, double_dim>, Eigen::Matrix<float, double_dim, double_dim>);
	virtual std::pair<Eigen::Vector<float, num_dim>, Eigen::Matrix<float, num_dim, num_dim>> project(Eigen::Vector<float, double_dim>, Eigen::Matrix<float, double_dim, double_dim>);
	virtual std::pair<Eigen::Vector<float, double_dim>, Eigen::Matrix<float, double_dim, double_dim>> update(Eigen::Vector<float, double_dim>, Eigen::Matrix<float, double_dim, double_dim>, Eigen::Vector<float, num_dim>);
	virtual Eigen::VectorXf gating_distance(Eigen::Vector<float, double_dim>, Eigen::Matrix<float, double_dim, double_dim>, Eigen::VectorX<Eigen::Vector<float, num_dim>>, bool only_position=false);

private:
	Eigen::Matrix<float, double_dim, double_dim> motion_mat;
	Eigen::Matrix<float, num_dim, double_dim> update_mat;

	const float std_weight_pos = 1. / 20;
	const float std_weight_vel = 1. / 160;
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
	};S
};