INTRODUCTION
===================================

introduction
----------------------------------
Romi is a small differential drive robot that we built utilizing kit parts from pololu. We programed Rmoi to navigate a complex line course on its own as fast as possible. it follows a black line using an array of light sensors to measure where the line is under the robot. It uses this data with a PI contoller to adjust the wheel speeds to maintain the center of the robot alligned with the path. Moreover, Romi utulised more than just light sensros to navigate throught the course, it also uses bump sensors and a discrete state machine to predict where Romi is in a 2-dimensional grid.


Coding overview
------------------------------------
We utliliexed cooperative multitasking within our code. This means that we created different tasks within our code that are responsible for specific functions. All the code that we made for this project can be found in our github websited provided below. With our cooperative multitasking we assigned each task an intended run period and priority that would dictate when each task should run.

link: https://github.com/jirehevelasquez/mech-romi-files





