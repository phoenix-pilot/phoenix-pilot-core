# Extended Kalman Filter

This directory contains an implementation of extended Kalman filter (EKF) for UAV state observation. 

Phoenix Pilot EKF implementation uses quaternion mathematics for rotation calculations and simple matrix calculations library

For notation and general information please reffer to [EKF wikipedia page](https://en.wikipedia.org/wiki/Extended_Kalman_filter).

# Current state

Version of EKF available in this repository is fed with dummy measurements of acceleration a=(1,0,0) and rotation w=(0,0,1) (in body frame of reference). To set different values one must change values in kalman_update.c:get_measurements() function.