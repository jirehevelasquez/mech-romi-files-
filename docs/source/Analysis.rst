Analysis
======================

Overview
------------------
For an in depth description of the analysis that we performed to characterise the dynamics of romi please reference the "ME-405 tuning" file located in our Github repository under calculations. A quick high level summary of the actions we performed include several expereimetal trials and data proccessing to identify the most accurate time constant for each motor. Aswell as identify what KP amd KI values ( Proportional and Integral gains) created the best dynamics with little overshoot and good steady state values.

Motor Time Constant calculations 
-----------------
We direclty measured the velocity of each wheel's motor versus time for a range of different setpoints. We then proceesed this data in an excel sheet to calculate th time constant for each motor due to the time constant being 63% of the time it takes for the motor to reach steady state values. We than averaged all of the calculated time constants that ranged from -100% effort to 100% effort to get the best time constant values for our motors to be able to operate at a range of setpoints 

Motor KI and KP Tuning
------------------------
Once we had a calculated time constant we than began varying our proportional gain (KP) and integral gain (KI) in our PI controller to idenifty which values lead to the quickest time response with less than 10% overshoot and a steady state value with less than 5% fluctionons of the intended setpoint. 

Light Sensor Calibration
--------------------------------
In our code, we calibrated our light sensors using both our UI_gen file and our Light file. we accomplished this by prompting the user to place Romi over a section of black paper to represent the black line that Romi would follow. Then while romi is over the black paper, press b, to read from the light sensors casuing those values to be stored and used as a datum. Then after we prompted the user to place romi over a piece of white paper. Where we measured the light sensors to store those values as another datum. After we had stored these values, each instance our light task would run, it could grab the current values from the light sensors and calculate the centroid of the line using the 2 datums to identy where the line was at in that instance. which was then fed into out PI controler.

IMU Calibration
--------------------------------
Aubrey write here I dont remeber much about this topic

Finite State Machine Calculations
----------------------------------
To begin to make a state estimation machine for Romi, we first needed to fully characterize the physical dynamics of Romi, which are that romi is a 2 wheel drive robot that is not holonomic. We began by calculating the velocity of the center of mass of romi relative to both wheels and identifyied basic equations that relate both the rotational velocity of romi, and its translational velocity as a function of the wheel velocitys. These equations are represented in our X variable.

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
   \boxed{\hat{y} = C \hat{x} + D u}


.. math::

   \boxed{\dot{\hat{x}} = (A - L C)\hat{x} + (B - L D)u + L y}

