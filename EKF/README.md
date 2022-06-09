# Extended Kalman Filter

This directory contains an implementation of extended Kalman filter (EKF) for UAV state observation. 

Phoenix Pilot EKF implementation uses quaternion mathematics for rotation calculations and simple matrix calculations library

For notation and general information please reffer to [EKF wikipedia page](https://en.wikipedia.org/wiki/Extended_Kalman_filter).

# Current state

Version of EKF available in this repository is designed to run on imx6ull evaluation board on Phoenix-RTOS. I2C driver from phoenix-rtos-devices subrepository __imx6ull-sensi2c__ must run in the background to provide simple `sensXxxx()` API, although this will be subject to change, as mainly SPI will be used in communication with sensors.

This version of EKF utilizes:
 - Inertial Measurement Unit with accelerometer and gyroscope,
 - 3 DoF magnetometer
 - barometer

and provides state information of following parameters (mentioned only those that are accurately estimated):
 - altitude from calibraition (start point)
 - rotation of the IMU relative to North-West-Down frame (will be subject to change to North-East-Down frame)

# Structure

 ### `kalman_core`
 Core EKF calculations on matrices that perform prediction and update steps using abstractions of measurement model (`update_engine_t`) and prediction model (`prediction_model_t`). Utilizes `phmatrix` matrix library from `/tools/`. Provides macros for declaring all necessary measurement model matrices of correct sizes and for inserting them into `update_engine_t`. 

 ### `kalman_implem`
 Kalman filter implementation specific code. Defines initialization parameters and performs initialization of covariance and state matrices values. Defines prediction step algorithms. Creates prediction engine.

 ### `kalman_update_*`
 Measurement model specific code. Creates measurement engine, and defines all necessary functions for it.

 ### `measurement`
 Data acquisition code. Performs all necessary initialization measurements, calibration of data. All communication with sensor is done via this module. Provides interface for measurements modules to acquire calibrated data as close to desired measurement vector form as possible.

 ### `main`
 Sample code performing kalman filtering, data outputting.