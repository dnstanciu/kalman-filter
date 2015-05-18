#ifndef UTILS_HPP
#define UTILS_HPP
#include <random>
#include <cmath>

class Utils
{
public:
    static void init();
    static double add_uniform_noise(double level, double x);
    static double sample_uniform(double from, double to);
    static double sample_Gaussian(double mu, double sigma);


private:
    Utils();
    ~Utils();

    static std::default_random_engine generator;
};
#endif