Issues experienced
=================

1. Burnt Bluetooth modules
---------------------------------
In the beginning of this project, we first attempted to connect our Bluetooth module to our computers to be able to communicate with Romi without the need for a physical cable. We had early success with this, but one day while we were testing Romi, our Bluetooth module stopped working due to it being fried. The main issue we experienced was not fully understanding what actions led to the module getting burnt, whether it was software-related or hardware-related. This issue troubled us later down the road when we burnt another Bluetooth module and then lost the whole capability to communicate with Romi over Bluetooth.

2. Burnt Pins on our nucleo board
--------------------------------
One of the largest challenges we faced during the project was a burnt pin. The Nucleo we were provided had one bad pin that we attempted to use for the PWM of our right motor. This bad pin caused our right motor to not function properly, keeping the signal pin at a constant high. Unfortunately this took us a long time to figure out, however, once we found out that the pin was the problem, we switched the PWM pin of our right motor to a different pin, and the motor began functioning well. 

3. Inaccurate state estimaiton 
-------------------------------
Due to uncontrollable delays between running our different tasks, our IMU task responsible for calculating our state estimation was not able to run at a consistent period, leading to unpredictable jumps in our calculated state estimation values. This dramatically increased the error in our system, rendering most of our state estimation values unusable due to their sheer inaccuracy and fluctuations, making our track following exponentially more difficult.

4. fluctuationg IMU readings
-----------------------------
When Romi got close to the steel cage/garage, the IMU readings regarding Romi's heading became dramatically volatile, often ranging plus or minus 80 degrees. Due to this, we had to preemptively shut off the IMU's yaw reading because it would largely confuse our pre-existing code, causing Romi to go off course and crash into the cage. Thus, during this section of the track, we were forced to navigate only using our encoder velocities and known time displacements.
