// KalmanFilter.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <fstream>
#include "KalmanFilter.h"

int main()
{
    KalmanFilter filter = KalmanFilter();
    Eigen::Matrix<float, double_dim, double_dim> cov = Eigen::Matrix<float, double_dim, double_dim>::Ones();
    Eigen::VectorX<Eigen::Vector<float, num_dim>> measurements(2);

    measurements << Eigen::Vector<float, num_dim>(2, 4, 6, 8), Eigen::Vector<float, num_dim>(3, 5, 7, 9);

    /*filter.initiate(Eigen::Vector<float, num_dim>(2, 4, 6, 8));
    filter.predict(Eigen::Vector<float, double_dim>(1, 2, 3, 4, 5, 6, 7, 8), cov);
    filter.project(Eigen::Vector<float, double_dim>(1, 2, 3, 4, 5, 6, 7, 8), cov);
    filter.update(Eigen::Vector<float, double_dim>(1, 2, 3, 4, 5, 6, 7, 8), cov, Eigen::Vector<float, num_dim>(2, 4, 6, 8));
    filter.gating_distance(Eigen::Vector<float, double_dim>(1, 2, 3, 4, 5, 6, 7, 8), cov, measurements, true);*/

    std::ifstream readingFromFile("helicopter_in_hurricane.txt");
    std::ofstream writingToFile("helicopter_output.txt", std::ofstream::trunc);

    std::string line;
    Eigen::Vector<float, num_dim> vector;

    // get first line and copy to vector buffer
    std::getline(readingFromFile, line);
    std::istringstream stream(line);
    std::copy(std::istream_iterator<float>(stream), std::istream_iterator<float>(), vector.begin());

    // set up vector output formatting
    Eigen::IOFormat format(Eigen::StreamPrecision, Eigen::DontAlignCols, "", " | ", "", "");
    std::cout << std::endl << vector.format(format) << std::endl;
    
    // get initial values from filter
    std::pair<Eigen::Vector<float, double_dim>, Eigen::Matrix<float, double_dim, double_dim>> pair = filter.initiate(vector);

    writingToFile << pair.first.format(format) << std::endl;

    Eigen::Vector<float, double_dim> result_mean;

    // iterate over file and run predict-update loop
    while (std::getline(readingFromFile, line)) {
        vector = Eigen::Vector<float, num_dim>::Zero();
        std::istringstream stream(line);
        std::copy(std::istream_iterator<float>(stream), std::istream_iterator<float>(), vector.begin());
        pair = filter.predict(pair.first, pair.second);
        pair = filter.update(pair.first, pair.second, vector);

        
        writingToFile << pair.first.format(format) << std::endl;
        std::cout << std::endl << pair.first.format(format) << std::endl;
    }

    readingFromFile.close();
    writingToFile.close();
}

