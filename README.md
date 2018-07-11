# CarND_Project06ExtendedKalmanFilter

Solution for Udacity Self driving car Nano degree sixth project: Extended Kalman Filter Project.

---

Abbreviations:
KF:  Kalman Filter.

EKF: Extended Kalman Filter.

[//]: # (Image Referensers)

[LRImg]: ./images/LaserAndRadar.png
[LR2Img]: ./images/LaserAndRadar_Dataset2.png
[LImg]: ./images/LaserOnly.png
[RImg]: ./images/RadarOnly.png
[R2Img]: ./images/RadarOnly_dataset2.png

---

## Objective

Implement an Extended Kalman filter tracking a vehicle motion, using measurements from a LIDAR and a RADAR to perform sensor fusion on those measuerments and reach a set of performance metrics defined by Udacity.

---

### Reflection

This project aims to introduce the concept of sensor fusion using EKF. There are two datasets representing measurements from a LIDAR and a RADAR including both the sensor measurements and the actual values to be used as a metrics in performance evaluation.

Udacity provided a simulator that allows visualize of the different lidar, radar measurement points and the output EKF data points.

There are two different types of readings based on the sensor type:-
  * LIDAR:
    - Provides a point cloud where each point has a cartesian coordinates system.
    - Not able to provide speed measurements.
  * RADAR:
    - Provides a range and bearing reading in a polar coordinates system.
    - Able to provide angular speed measurements.

The EKF needs to handle each type of measurements based on it's mentioned properties.

---

### KF Theory

KFs have two main internal data structures:
  * State: represent the tracked object/process state, in our project we have a 4 variable state:
    - Position in X axis.
    - Position in Y axis.
    - Velocity in X axis.
    - Velocity in Y axis.
  * State Covariance: can be thought of as representing the certainty in the measurements found in State variables.

KF has two main stages:
  * Prediction:
    - Based on the pre-defined motion model the KF can estimate, i.e. predict, the motion of the tracked object and determine it's new    state and covariance variables.
  * Measurement update:
    - In this stage the KF use the sensor measurements, taking in consideration the uncertainty in sensor measurements, to correct any errors in prediction state.
    
- KF vs EKF:
The KF has a restriction that the tracked process must have a linear property, if the tracked process is not linear then the equations of KF will not hold.

The EKF aim to fix this issue by linearizing the non-linear process at the point of estimation using a first Taylor expansion, i.e. Jacobian calculation.

---

### Implementation

Udacity have provided most of the control code as:
  * Communication between EKF and Udacity simulator.
  * Parsing of the measurements points and feeding the values to the EKF.
  * Triggering the performance evaluation process.
  
I have made some modifications to this control code to achieve some of C++ concepts as data locality and encapsulation by setting EKF data class members as private and re-locating the Jacobian calculation function to the EKF module itself since it is part of it's implementation.
  
I had to implement the following parts:
  * Fusion logic
    - Responsible for handling the measurements of the LIDAR and RADAR and processing each type of measurements based on the sensor properties mentioned before.
  * EKF
    - Responsible for the actual calculations of the sensor fusion EKF model.
  * RMSE
    - Used as a metric during performance evaluation
    
I have added some pre-processor switch as well to:
  * Enable of disabling/enabling of handling one of the two sensors.
  * Enable various level of debugging of the code.

---

### Results

Udacity define that for a successful project submission your code must produce RMSE values equal or lower to [.11, .11, 0.52, 0.52] for data set 1.

After finishing the implementation, connecting to the simulator I was able to see that the RMSE error values were within the acceptable range defined by Udacity. Using the RADAR and LIDAR on dataset 1 [0.09, 0.08, 0.45, 0.43].

Following this, I have made a number of trials to analyze how the EKF performance and how much is the actual gain from the sensor fusion ?

|   Index  |    Sensors    |   Data Set    |            RMSE          |
|----------| ------------- | ------------- |--------------------------|
|     1    | RADAR + LIDAR |       1       | [0.09, 0.08, 0.45, 0.43] |
|     2    | RADAR + LIDAR |       2       | [0.07, 0.09, 0.42, 0.49] |
|     3    |     LIDAR     |       1       | [0.14, 0.11, 0.63, 0.53] |
|     4    |     LIDAR     |       2       | [0.14, 0.11, 0.63, 0.53] |
|     5    |     RADAR     |       1       | [0.27, 0.38, 0.65, 0.92] |

It is quite clear that the standalone RADAR performs much worse than standalone LIDAR.

But with the fusion between the LIDAR and the RADAR we obtain a 39% average improve over standalone LIDAR RMSEs.

This is the beauty of sensor fusion is that using multiple of lower certainty measurements you can obtain a much more certain estimation of your state.

During my analysis and trials, I have made some minor tweaks to the code for better performance, this be seen specially in the radar measurement update functions.

The below images show the output of some of the sequences following the index in the table:

1) 
![LRImg][LRImg]

---

2) 
![LR2Img][LR2Img]

---

3) 
![LImg][LImg]

---

5)
![R2Img][R2Img]

---

### Conclusion

I think I have reached quite a good understanding of the theory and application of the EKF and how it works to achieve the goal of a high confidence in tracked objects.
