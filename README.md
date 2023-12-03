# Path-Planning-Techniques-(APF,-BFS-and-A*)

This project involves testing different path planning techniques in a Gazebo simulator world comprising of obstacles and how Turtlebot3 managed to get to the goal position. It tackles 3 path planning technquies which are (Artificial potential field (APF), Breadth first search (BFS), and A*).

First, the control techniques used for the Turtlebot3 to move was mainly 2 control techniques which are Lyapunov and Goal to Goal compiled in a package called "Milestone 5". 

Artificial Path Planning (APF) Algorithm

The artificial potential field (APF) algorithm is one of the algorithms used in robot path planning in which its force FAPF is the sum of the attractive potential field Fatt and the repulsive potential field Frep as shown in the following equation:

Breadth First Search (BFS) Algorithm

A-star (A*) Algorithm



Instructions:

1. Prepare your workspace and copy the “Milestone5” package, this roslaunch Milestone 5 <<name of the launch file>>.

NOTE: 
The launch files available are: -

Turtlebot3_Astar.launch >>> after launching this file you will see the map and movement of the turtltebot3 using A* path planning technique. 

Turtlebot3_Astar_modified.launch >>> the same as A* but with modified algorithm, related to the mechanism of A* itself that it provides more optimized path.

Turtlebot3_BFS.launch >>>  after launching this file you will see the map and movement of the turtltebot3 using A* path planning technique. 

Turtlebot3_ID_world.launch >>>  after launching this file you will see the map and movement of the turtltebot3 using APF path planning technique. 

The related “Python” files associating for all these path planning technquies can be found easily in the src folder. 





