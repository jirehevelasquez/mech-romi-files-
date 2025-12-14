Analysis
======================

Overview
----------------------------

For an in-depth description of the analysis that we performed to characterize the dynamics of Romi, please reference the ME-405 Tuning file located in our GitHub repository under Calculations. A quick high-level summary of the actions we performed includes several experimental trials and data processing to identify the most accurate time constant for each motor, as well as identifying which KP and KI values (proportional and integral gains) created the best dynamics with little overshoot and good steady-state values.

Motor Time Constant Calculations
---------------------------------------

We directly measured the velocity of each wheel's motor versus time for a range of different setpoints. We then processed this data in an Excel sheet to calculate the time constant for each motor, with the time constant being 63% of the time it takes for the motor to reach steady-state values. We then averaged all of the calculated time constants that ranged from −100% effort to 100% effort to get the best time-constant values for our motors to be able to operate at a range of setpoints.

Motor KI and KP Tuning
------------------------------------

Once we had a calculated time constant, we then began varying our proportional gain (KP) and integral gain (KI) in our PI controller to identify which values led to the quickest time response with less than 10% overshoot and a steady-state value with less than 5% fluctuations of the intended setpoint.

Light Sensor Calibration
-------------------------------

In our code, we calibrated our light sensors using both our UI_gen file and our Light file. We accomplished this by prompting the user to place Romi over a section of black paper to represent the black line that Romi would follow. Then, while Romi is over the black paper, "b" is pressed to read from the light sensors, causing those values to be stored and used as a datum. Then we prompted the user to place Romi over a piece of white paper, where we measured the light sensors to store those values as another datum. After we had stored these values, each time our light task would run, it could grab the current values from the light sensors and calculate the centroid of the line using the two datums to identify where the line was at in that instance, which was then fed into our PI controller.

IMU Calibration
--------------------------

The IMU we used was the BNO055 9-axis sensor. Although this IMU continuously calibrates itself when used in IMU mode, we created a text document called calibration.txt that held the calibration offset coefficients for the accelerometer, gyroscope, and accelerometer radius. These calibration values were found experimentally with the Romi we used in the track space and stored in the text file. In our IMU task in our main file, these values are read from the text file and applied to the IMU once Romi is commanded to start the course.

Finite State Machine Calculations
----------------------------------------

To begin to make a state-estimation machine for Romi, we first needed to fully characterize the physical dynamics of Romi, which is a 2-wheel-drive robot that is not holonomic. We began by calculating the velocity of the center of mass of Romi relative to both wheels and identified basic equations that relate both the rotational velocity of Romi and its translational velocity as a function of the wheel velocities. These equations are represented in our X_dot variable, where X represents our wheel positions for both left and right motors, heading value, and arc distance. U represents the inputs into our system, which are the left and right voltage inputs from the motor driver. The Y variable represents the outputs of our system, which are the net distance each wheel has rotated (read from the encoders), as well as the yaw and yaw rate, which can be read from the IMU. The discrete state estimation that we used employs matrices A_d, B_d, and C_d, which represent the discretized versions of these matrices. For a more in-depth explanation, please reference the MATLAB script in the repository under calculations.


.. math::

   X =
   \begin{bmatrix}
      \Omega_L \\
      \Omega_R \\
      S \\
      \psi
   \end{bmatrix},
   \qquad
   U =
   \begin{bmatrix}
      V_L \\
      V_R
   \end{bmatrix},
   \qquad
   Y =
   \begin{bmatrix}
      S_L \\
      S_R \\
      \psi \\
      \dot{\psi}
   \end{bmatrix}.

.. math::

   \dot{X} =
   \begin{bmatrix}
      -\dfrac{1}{\tau_m} & 0                   & 0 & 0 \\
      0                   & -\dfrac{1}{\tau_m} & 0 & 0 \\
      \dfrac{r}{2}        & \dfrac{r}{2}       & 0 & 0 \\
      -\dfrac{r}{w}       & \dfrac{r}{w}       & 0 & 0
   \end{bmatrix}
   X
   +
   \begin{bmatrix}
      \dfrac{k_m}{\tau_m} & 0 \\
      0                   & \dfrac{k_m}{\tau_m} \\
      0                   & 0 \\
      0                   & 0
   \end{bmatrix}
   U


.. math::

   Y =
   \begin{bmatrix}
      0 & 0 & 1 & -\dfrac{w}{2} \\
      0 & 0 & 1 &  \dfrac{w}{2} \\
      0 & 0 & 0 & 1 \\
      -\dfrac{r}{w} & \dfrac{r}{w} & 0 & 0
   \end{bmatrix}
   X
   +
   \begin{bmatrix}
      0 & 0 \\
      0 & 0 \\
      0 & 0 \\
      0 & 0
   \end{bmatrix}
   U





.. math::
   \boxed{\hat{y} = C \hat{x} + D u}


.. math::

   \boxed{\dot{\hat{x}} = (A - L C)\hat{x} + (B - L D)u + L y}

