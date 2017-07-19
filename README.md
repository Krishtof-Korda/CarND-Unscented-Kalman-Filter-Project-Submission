# Korda's Unscented Kalman Filter Project for Udacity Self-Driving Car Nanodegree

[YouTube Video](https://youtu.be/t_4gRkuGDSo)

[![alt text](https://img.youtube.com/vi/t_4gRkuGDSo/0.jpg)](https://youtu.be/t_4gRkuGDSo)



---

The goals / steps of this project are the following:

* Implement an Unscented Kalman Filter for tracking a pedestrian.
* Build on my C++ skills for coding, classes, compiling, etc.


## [Rubric Points](https://review.udacity.com/#!/rubrics/783/view) 

This project used a simulator to stream positional and velocity sensor data from Lidar and Radar returns of a cyclist. The Kalman filter then provided its estimate of the position and velocity of the pedestrian after each measurement. If the measurement was Lidar a standard Kalman filter was used. If the measurement was Radar, then the Unscented Kalman filter was used. The Unscented Kalman filter determines sigma points to pass through a non-linear measurement function which converts the predicted state value from x, y position, velocity, yaw and yaw rate into range, angle, and range rate in order to execute a parameter update. This is done because the measurement space of Radar is range, angle, and range rate.

I tested the Unscented Kalman Filter with the fusion of Lidar and Radar sensors. The fused sensors have positional root mean squared errors (RMSE) on the order of 0.06-0.08, while the velocity errors are on the order of 0.20-0.33. Here is an image of the output visualized with green being the filter estimation, and red and blue being the lidar and radar sensor data. Later I tested the filter with only radar or only lidar to see how the different sensors contribute to the errors.

In order to adjust the process noise values (std_a, std_yawdd) for longitudinal and yaw acceleration of the cyclist I used the Normalized Innovation Squared (NIS) value. I needed to verify that the NIS only went above the 7.8 chi squared value a few times to ensure I was not underestimating the process noise. The NIS values can be seen below for radar-only and lidar-only.

#### This is with fusion of Lidar and Radar (Dataset 1):
![alt text][1]

  [1]: ./images/PassingRMSE.jpeg


#### This is with fusion of Lidar and Radar (Dataset 2):
![alt text][11]

  [11]: ./images/Dataset2Fusion.jpeg 



I also tested the filter with Lidar data only which led to positional errors on the order of 0.09-0.10 and velocity errors of 0.24-0.59. The errors increased slightly for position and nearly doubled for velocity. This is to be expected since Lidar has very high positional accuracy with no sensing of velocity. The velocity is a derived parameter using distance covered of a certain time period. 

#### This is with Lidar only:
![alt text][2]

  [2]: ./images/LaserOnly.jpeg 


#### These are the NIS values for Lidar only:
![alt text][12]

  [12]: ./images/NIS_Laser_Chart.jpg



Lastly I tested the filter with Radar data only which led to positional errors of 0.15-0.20 and velocity errors of 0.26-0.35. The position error nearly doubled and the velocity error remained nearly unchanged from the fused solution. This also makes sense since Radar is considered very accurate for velocity, but not so for position. 

#### This is with Radar only:
![alt text][3]

  [3]: ./images/RadarOnly.jpeg 


#### These are the NIS values for Radar only:
![alt text][13]

  [13]: ./images/NIS_Radar_Chart.jpg



## Comparison with Extended Kalman Filter:

The Unscented Kalman Filter was better at handling the non-linear nature of the Radar sensor reducing the velocity RMSE from 0.45 to 0.33. In position there was some improvement as well although not as dramatic as velocity, which makes sense since the Lidar calculations are linear and not helped by the Unscented transformation. 

This was a very fun and informative project on Kalman Filters and C++. Thank you again to the Udacity creators for yet another smash hit!

Cheers!
  
