# CarND-Controls-Model Predictive Control Project
Self-Driving Car Engineer Nanodegree Program

---

## Project Objectives

Implement a model predictive control algorithm to enable a self driving car to navigate around a track provided by the Udacity simulator.

[//]: # (Image References)
[image1]: ./Images/StateDefine.PNG
[image2]: ./Images/Errors.PNG
[image3]: ./Images/StateEqns.PNG
[image4]: ./Images/Weights.PNG
[image5]: ./Images/MeasurementPrediction.PNG
[image6]: ./Images/UKFupdate1.PNG
[image7]: ./Images/NIS.PNG
[image8]: ./Images/ChiSquare.PNG

## Vehicle Motion Model

The fundamental premise of Model Predictive Control is that we have a dynamic model that can predict the vehicle's states in the future. The vehicle model uses 6 states to uniquely define the system. 

```
Px - X position of the car in the global coordinate system
Py - Y position of the car in the global coordinate system
Psi - Vehicle orientation
V- velocity of the vehicle
Cte - cross track error aka how far is the car off from the desired reference trajectory
Epsi - orientation error

```
![alt text][image1]

The cross track error and orientation error can be represented as shown below. The dashed white line indicates cross track error. 

![alt text][image2]

There are typically 3 control inputs in a vehicle - throttle, brake and steer. Throttle and brake can be combined into a single input with bounds [-1,1]. A negative value implies braking. Therefore, the system had 2 control inputs [steer, throttle].

The state equations are given below:

![alt text][image3]


