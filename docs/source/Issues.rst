Issues experienced
=================

1. Burnt Bluetooth modules
---------------------------------
In the beginning of this project, we first attempted to connect our bluetooth module to our computers to be able to communicate with Romi without the need of a physical cable. We had early success with this but one day while we where test Romi, our bluetooth stopped working due to it being fried. The main issue we experienced was not fully understanding what actions we did that lead to the module getting burnt, whether it was software related or hardware related. This issue troubled use later down the road when we burnt another bluetooth module and then lost the whole capability to comunicate with Romi over bluetooth. 

2. Burnt Pins on our nucleo board
--------------------------------

3. Inaccurate state estimaiton 
-------------------------------
Due to uncontrollable delays between running our different tasks, our IMU task responsible for calculating our state estimation was not able to run at a consistant period leading to unpredictable jumps in our calculated state estimation values. This dramatically increased the error in our system rendering most of our state estimation values not usable due to there shear inaccurace and fluctuations making our track following exponentially more difficult 

4. fluctuationg IMU readings
-----------------------------
when Romi got close to the steel cage/garage the imu readings regarding romi's heading became dramatically volatile often ranging plus or minues 80 degrees. Due to this we had to preemptively shut off the imu's yaw reading because it would largely confuse our pre-existing code causeing romi to go off course and crash into the cage. So during this section of the track we were forced to navigate only using our encoder velocitys and known time displacements.
