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

