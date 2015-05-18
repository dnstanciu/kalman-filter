/**
 * Designs and implements a Kalman filter for a thermometer. This is an example of a 1D Kalman filter.
 *
 * The sensor for the thermometer outputs a voltage that corresponds to the temperature that is being measured.
 * We have read the manufacturer's specifications for the sensor, and it tells us that the sensor exhibits white noise
 * with a standard deviation of 2.13.
 *
 * We do not have a real sensor to read, so we will simulate the sensor with the "volt" function.
 * We will add a bit of error to our prediction step to tell the filter not to discount changes in voltage over time.
 *
 * Idea for problem: chapter 5 (Kalman Filters) of https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python
 *
 * This implementation uses the OpenCV Kalman filter class.
 */


#include <iostream>
#include <vector>

#include "Utils.hpp"
#include "KalmanFilter1D.hpp"

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

using std::cout; using std::endl;

double volt(double voltage, double temp_std_dev)
{
    return Utils::sample_Gaussian(voltage, temp_std_dev);
}

int main()
{
    Utils::init();

    double movement = 0;
    double std_dev = 2.13;
    double movement_variance = 0.2;
    double actual_voltage = 16.3;

    int N = 20;
    std::vector<cv::Mat> zs;
    std::vector<cv::Mat> ps;
    std::vector<cv::Mat> estimates;

    // dimensionality for state, measurement, control vector
    cv::KalmanFilter KF(1, 1, 1);
    cv::Mat state(1, 1, CV_32F); // xs
    cv::Mat processNoise(1, 1, CV_32F); // Q (movement noise)
    cv::Mat measurement = cv::Mat::zeros(1, 1, CV_32F);


    // set the state constantly to actual_voltage
    state = cv::Scalar::all(actual_voltage);

    cv::setIdentity(KF.transitionMatrix);
    cv::setIdentity(KF.measurementMatrix);                                     // H matrix (1)
    cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(movement_variance));   // Q matrix
    cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(std_dev*std_dev)); // R matrix
    cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1000));                   // initial variance P (1000)s
    cv::setIdentity(KF.statePost, cv::Scalar::all(25));                        // initial estimate (25)

    for (int i = 0; i<N; ++i)
    {
        cv::Mat prediction = KF.predict();

        // generate measurement by getting the noise
        cv::randn( measurement, cv::Scalar::all(0), cv::Scalar::all(sqrt(KF.measurementNoiseCov.at<float>(0))) );
        // and then adding the actual state value
        measurement += KF.measurementMatrix*state;

        zs.push_back(measurement.clone());

        KF.correct(measurement);

        estimates.push_back(KF.statePost.clone());
        ps.push_back(KF.errorCovPost.clone());
    }

    // print measurements
    cout << "measurements: ";
    for (size_t i = 0; i<zs.size(); ++i)
        cout << zs[i] << " "; cout << endl;

    // print estimates
    cout << "estimates: ";
    for (size_t i = 0; i<estimates.size(); ++i)
        cout << estimates[i] << " "; cout << endl;

    // print covariances
    cout << "covariances: ";
    for (size_t i = 0; i<ps.size(); ++i)
        cout << ps[i] << " "; cout << endl;

    return 0;
}