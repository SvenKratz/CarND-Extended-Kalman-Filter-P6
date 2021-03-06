# Extended Kalman Filter
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


This repository contains my solution to the Udacity Self-Driving Car NanoDegree Extended Kalman Filter Project.

The repository contains the following notable files:

* [WRITEUP.md](WRITEUP.md) Project writeup report with additional details on my implementation.
* `src/FusionEKF.cpp` : Implementation of sensor fusion for Lidar and Radar sensors using extended Kalman Filter
* `src/kalman_filter.cpp` : Basic Kalman filter class used by FusionEKF
* `src/tools.cpp` : utility class containing methods for calculating Jacobian, conversions between polar and cartesian coordinates and RMSE on data samples
* `src/main.cpp` : file contains main function and interfaces with simulator websocket in for data in and output  
* [carnd-ekf.mp4](carnd-ekf.mp4) video file of the terminal window and simulator output on dataset 1
* [dataset1_rmse_final.png](dataset1_rmse_final.png) image showing final RMSE values of the EKF after running through dataset 1

## Compilation and Run Instructions

Project Dependencies: see [Project Instructions](Docs/README.md)

From the project root directory

1. Create a build directory: `mkdir build`
2. `cd build`
3. Run cmake generator: `cmake ..`
4. Run the executable with `./ExtendedKF`
