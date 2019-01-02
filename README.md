[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

# The Project
In this project an Unscented Kalman Filter is used to estimate the state of a moving object of interest with noisy lidar and radar measurements.

[//]: # (Image References)

[image1]: ./images/term2_simulator_dataset1.png "Result of Dataset 1"
[image2]: ./images/term2_simulator_dataset2.png "Result of Dataset 2"

## Prerequisites

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

Required tools are:
* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
 * Linux: make is installed by default on most Linux distros
 * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
 * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
 * Linux: gcc / g++ is installed by default on most Linux distros
 * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
 * Windows: recommend using [MinGW](http://www.mingw.org/)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

## Build and Install
Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

## Interaction with the Term 2 Simulator

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]


## Code Style
I tried to stick to the [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).
In order to check the guidelines I installed cpplint using 
`pip install cpplint`


## Results
The simulator provides noisy lidar and radar measurements which are shown as blue and red dots. The position that is calculated by the kalman filter is displayed as green dots.

The results for dataset 1 are shown in the following image
![alt text][image1]

The results for dataset 2 are shown in the following image
![alt text][image2]

## Further Reading
For further reading and to dive deeper into the concept of UKF and basic filtering I suggest these materials which I enjoyed reading and watching:

If you need math behind the Kalman Filter then this youtube series is awesome: [THE KALMAN FILTER](https://www.youtube.com/playlist?list=PLX2gX-ftPVXU3oUFNATxGXY90AULiqnWT)

This interactive tutorial is very helpful to visualize and have a better grasp of workings in this area: The Extended Kalman Filter: [An Interactive Tutorial for Non-Experts](https://home.wlu.edu/~levys/kalman_tutorial/)

If you are looking for what other filters might be apart from EKF and UKF then you can check this. It's little math heavy and based on Matlab: [Optimal Filtering with Kalman Filters and Smoothers](https://docs.google.com/viewer?url=http%3A%2F%2Fbecs.aalto.fi%2Fen%2Fresearch%2Fbayes%2Fekfukf%2Fdocumentation.pdf)

A nice explanation of choosing between EKF and UKF and then it's implementation guide. It's in two parts: [Understanding Nonlinear Kalman Filters Part I](https://docs.google.com/viewer?url=https%3A%2F%2Fweb.statler.wvu.edu%2F~irl%2FIRL_WVU_Online_EKF_vs_UKF_V1.0_06_28_2013.pdf)

And [Part II](https://docs.google.com/viewer?url=https%3A%2F%2Fweb.statler.wvu.edu%2F~irl%2FIRL_WVU_Online_UKF_Implementation_V1.0_06_28_2013.pdf)

A Python library for Kalman filtering which you can check out: [pykalman](https://pykalman.github.io/)

The UKF paper: [The Unscented Kalman Filter for Nonlinear Estimation](https://docs.google.com/viewer?url=https%3A%2F%2Fwww.seas.harvard.edu%2Fcourses%2Fcs281%2Fpapers%2Funscented.pdf)
