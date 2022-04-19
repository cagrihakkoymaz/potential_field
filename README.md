## Potentiel Field

## Potential field planner
At this porject potential field planning implention try to realised using ROS and CPP.

### Theory

At first understand the theory one can refer:
http://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf
The most important part of the achive planning find the correct force functions for attraction and repulsion.


### Implenmentation

**Attraction force**

Basic hypotenus function used for calculate cost of distance to goal and multipled with gain for adjustment

`KP * (sqrt(pow((tf_goal_x - (x + 0.5)), 2) + pow((tf_goal_y - (y + 0.5)), 2)));`

**Repulsion force**

1/Distnace_of_closest_obstacle used for calculate of cost of how close robot to obstacle

`ETA * (pow(((1 / min_distance)), 3)`



**INPUT**

-Map

-Starting point

-Goal Point

**OUTPUT**

-Path


**Installation**

copy potential_field file to

`cd ~/catkin_ws/src/`

`cd ..`

`catkin build`

`source devel/setup.bash`

`roslaunch potential_field potantial_field.launch` 

**Implemented**

-Functions of Attraction and Repulsion forces

-Adjusting path to map origin and resolution

-Trap for uninitilized goal or start

-Stop when oscillation start



**Implemented after submission**

-Trap for the start or goal point is at outside the map

-Path drawing debugged with initlized robot_poses{} vector every publish

-Obstacle avoidance debugged with correction of obstacle coordinates

-ROS and CPP codes seperated for more abstraction

-Potential field parameters added parameter server for controlling without compile

-Start PFP object as a Local variable using boost bind

-Publish converted to function



**TODO**

-Adjusting attraction force function and repulsion force function for to planner work most of the map

-Starting rviz every time at the origin of the map

-Avoding from local minimum




![map](https://user-images.githubusercontent.com/59797805/163889998-f6c821a8-7453-42b7-8f78-8f110573c1a0.gif)




