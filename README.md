## Korda's Unscented Kalman Filter Project for Udacity Self-Driving Car Nanodegree

[YouTube Video](https://youtu.be/Rp2Msc6z3nc)

[![alt text](https://img.youtube.com/vi/Rp2Msc6z3nc/0.jpg)](https://youtu.be/Rp2Msc6z3nc)



---

The goals / steps of this project are the following:

* Implement an Unscented Kalman Filter for tracking a pedestrian.
* Build on my C++ skills for coding, classes, compiling, etc.


## [Rubric Points](https://review.udacity.com/#!/rubrics/783/view) 

I tested the Unscented Kalman Filter with the fusion of Lidar and Radar sensors.

### This is with fusion of Lidar and Radar:
![alt text][1]

  [1]: ./images/PassingRMSE.jpeg 

I also tested the filter with Lidar data only which led to catastrophic failure of the filter as can be seen below. The Normalized Innovation Squared (NIS) is very high here as well on the order 2000, when it should be below 7.8 for 3 degrees of freedom. This requires further research to find a problem with the code.

### This is with Lidar only:
![alt text][2]

  [2]: ./images/LaserOnly.jpeg 

Lastly I tested the filter with Radar data only which led to only a slight increase in positional and velocity errors as seen below. I expected the velocity errors to remain fairly good for this and they did. This leads me to believe that my implementation of my laser only code definitely has a problem, since the radar only did very well compared to the fused solution. 

### This is with Radar only:
![alt text][3]

  [3]: ./images/RadarOnly.jpeg 
  
