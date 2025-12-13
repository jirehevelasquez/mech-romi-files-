Introduction
===================================

Introduction
--------------------------------

Romi is a small differential drive robot that we built utilizing kit parts from Pololu. We programmed Romi to navigate a complex line course on its own as fast as possible. It follows a black line using an array of IR light sensors to measure where the line is under the robot. It uses this data with a PI controller to adjust the wheel speeds to maintain the center of the robot aligned with the path. Moreover, Romi utilized more than just light sensors to navigate through the course; it also uses bump sensors and a discrete state machine to predict where Romi is in a 2-dimensional grid.

Coding Overview
---------------------------------

We utilized cooperative multitasking within our code. This means that we created different tasks within our code that are responsible for specific functions. All the code that we made for this project can be found in our GitHub website provided below. With our cooperative multitasking, we assigned each task an intended run period and priority that would dictate when each task should run.

link:  https://github.com/jirehevelasquez/mech-romi-files-/ 





