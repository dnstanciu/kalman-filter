cmake_minimum_required(VERSION 3.2)
project(kalman-filter-cpp)

find_package( OpenCV REQUIRED )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(include)

set(SOURCE_FILES kalmanThermometerOpenCV.cpp Utils.cpp KalmanFilter1D.cpp )
# main.cpp kalmanOpenCVsample.cpp kalmanThermometer.cpp


add_executable(kalman-filter-cpp ${SOURCE_FILES} )

target_link_libraries( kalman-filter-cpp ${OpenCV_LIBS} )