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
 * This implementation uses own KalmanFilter1D class.
 */


#include <iostream>
#include <vector>

#include "Utils.hpp"
#include "KalmanFilter1D.hpp"

//#include "opencv2/video/tracking.hpp"
//#include "opencv2/highgui/highgui.hpp"

using namespace std;

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

    int N = 50;
    std::vector<double> zs(N);
    for (size_t i = 0; i<zs.size(); ++i)
        zs.at(i) = volt(actual_voltage, std_dev);
    std::vector<double> ps;
    std::vector<double> estimates;

    for (size_t i = 0; i<zs.size(); ++i)
        cout << zs[i] << " ";
    cout << endl;

    KalmanFilter1D kf(25,                 // initial state
                      1000,               // initial variance
                      std_dev*std_dev,    // sensor noise
                      movement_variance); // movement noise

    for (size_t i = 0; i<zs.size(); ++i)
    {
        kf.predict(movement);
        kf.update(zs[i]);

        // save for plotting
        estimates.push_back(kf.get_x());
        ps.push_back(kf.get_P());
    }

    for (size_t i = 0; i<estimates.size(); ++i)
        cout << estimates[i] << " ";

    cout << endl;

    for (size_t i = 0; i<ps.size(); ++i)
        cout << ps[i] << " ";

    return 0;
}