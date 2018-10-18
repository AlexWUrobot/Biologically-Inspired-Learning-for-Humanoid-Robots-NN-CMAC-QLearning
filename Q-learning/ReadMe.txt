 *   Author:      Erhard Wieser
 *   Students:  Jakovleski Philipp, Lifan Wu, Siyuan Liu, Hajer Chebil, Chao Dong
 *
 *   JOB distribution:
 *   1. Leg Motion : Jakovleski Philipp and Hajer Chebil
 *   2. Vision :     LiFan Wu, Siyuan Liu, Chao Dong
 *   3. QL:          All group members do it together and modify the code

 
 
Include Coding:

Rein_learning.h
central_node.cpp
vision_node.cpp
Qtable is recorded in the first row because different states are in five different folders.



Steps:

roslaunch nao_apps tactile.launch
roslaunch nao_bringup nao_full_py.launch
rosservice call /body_stiffness/enable
rosrun bilhr central_node (Include: Training mode and Performance moed in Buttion 3)
rosrun bilhr vision_node

