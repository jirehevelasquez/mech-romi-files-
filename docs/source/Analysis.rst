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

