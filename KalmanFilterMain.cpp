// KalmanFilter.cpp : This file contains the 'main' function. Program execution begins and ends there.
//


#include "KalmanFilter.h"

int main()
{
    KalmanFilter filter = KalmanFilter();
    Eigen::Matrix<float, double_dim, double_dim> cov = Eigen::Matrix<float, double_dim, double_dim>::Ones();
    Eigen::VectorX<Eigen::Vector<float, num_dim>> measurements(2);

    measurements << Eigen::Vector<float, num_dim>(2, 4, 6, 8), Eigen::Vector<float, num_dim>(3, 5, 7, 9);

    filter.initiate(Eigen::Vector<float, num_dim>(2, 4, 6, 8));
	filter.predict(Eigen::Vector<float, double_dim>(1, 2, 3, 4, 5, 6, 7, 8), cov);
	filter.project(Eigen::Vector<float, double_dim>(1, 2, 3, 4, 5, 6, 7, 8), cov);
	filter.update(Eigen::Vector<float, double_dim>(1, 2, 3, 4, 5, 6, 7, 8), cov, Eigen::Vector<float, num_dim>(2, 4, 6, 8));
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
