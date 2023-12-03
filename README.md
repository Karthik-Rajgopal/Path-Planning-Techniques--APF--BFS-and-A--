This project involves testing different path planning techniques in a Gazebo simulator world comprising of obstacles and how the UGV, here Turtlebot3 managed to get to the goal position. It tackles 3 path planning technquies which are (Artificial potential field (APF), Breadth first search (BFS), and A*).

First, the control techniques used for the Turtlebot3 to move was mainly 2 control techniques which are Lyapunov and Goal to Goal compiled in a package called "Milestone 5". 

## Artificial Path Planning (APF) Algorithm:

A potential field is any physical field that obeys Laplace’s equation. Some common examples of potential fields include electrical, magnetic, and gravitational fields. A potential field algorithm uses the artificial potential field to regulate a robot around in a certain space. For our ease, we consider a space to be divided into a grid of cells with obstacles and a goal node. The algorithm assigns an artificial potential field to every point in the world using the potential field functions which will be described further in the explanation. The robot simulates from the highest potential to the lowest potential. Here, the goal node has the lowest potential while the starting node will have the maximum potential. Hence, we can say that the UGV moves from lowest to the highest potential.\
The artificial potential field (APF) algorithm is one of the algorithms used in robot path planning in which its force $F_{APF}$ is the sum of the attractive potential field $F_{att}$ and the repulsive potential field $F_{rep}$ as shown in the following equation: 
$$F_{APF}=F_{att}+F_{rep}$$ 
To apply the APF force to the robot, it is inserted into the linear speed equation on the kinematic robot. The desired speed equation in Attractive APF force $v_{G}^{att}$ is as follows: 
$$v_{G}^{att}(x,y)=-\nabla U_{att}(x,y)$$ 
where the potential attractive equation is partially derived to the x and y-axis as follows:
$$v_{x}^{att}(x,y)=-\frac{\partial U_{att}(x,y)}{\partial x}$$ $$v_{y}^{att}(x,y)=-\frac{\partial U_{att}(x,y)}{\partial y}$$ 
The equation of the Khatib’s potential attractive $U_att(x,y)$ is as follows: 
$$U_{att}=\frac{1}{2}k_{a}((\delta_{x}-x_{ref})^2+(\delta_{y}-y_{ref})^2)$$ 
where $k_{a}$ is the potential attractive constant, $\delta_{x}$, $\delta_{y}$ is the position of the robot. $(x_{ref}, y_{ref})$ is the position of the goal point.The desired speed equation for the Attractive APF force $v_{G}^{att}$ on the x and y-axis is as follows: 
$$v_{G_{x}}^{att}=-k_{a}(\delta_{x}-x_{ref})$$ $$v_{G_{y}}^{att}=-k_{a}(\delta_{y}-y_{ref})$$
The desired speed equation in the repulsive force $v_{O}^{rep}$ is:
$$v_{O}^{rep}(x,y)=-\nabla U_{rep}(x,y)$$
where the potential repulsive equation is partially derived to the x and y-axis as follows:
$$v_{x}^{rep}(x,y)=-\frac{\partial U_{rep}(x,y)}{\partial x}$$ $$v_{y}^{rep}(x,y)=-\frac{\partial U_{rep}(x,y)}{\partial y}$$
The equation of the Sfeir’s et al potential repulsive $U_{rep}(x,y)$ is:
$$U_{rep}=\frac{1}{2}k_{r}((\frac{1}{\rho_{O}}-\frac{1}{r_{O}}))^2$$
$$CE(p, y) = \begin{cases}
    -\log(p) & \text{if } y = 1 \\ 
    -\log(1-p) & \text{otherwise.}
\end{cases}$$
where $k_{r}$ is the potential repulsive constant, $r_{O}$ is the distance limit of potential repulsive influence, and $\rho_{O}$ is the closest distance between the robot and the obstacle. \
The closest distance between the robot and the obstacle, $\rho_{O}$ is:
$$\rho_{O}=\sqrt{(x_{ro})^2+(y_{ro})^2}$$
where $x_{or}$ is the difference of the distance between the robot and the obstacle on thex-axis, and $y_{or}$ is the difference of the distance between the robot and the obstacle on the y-axis which the equation is as follows:
$$x_{or}=\delta_{x}-x_{O}$$ $$y_{or}=\delta_{y}-y_{O}$$ 
The following is the desired speed equation for the APF Repulsive force $v_{O}^{rep}$ on the x and y-axes:

Thus, the speed equations of the x and y-axes in the APF are as follows:
$$v_{x}^{APF}=v_{G_{x}}^{att}+v_{O_{x}}^{rep}$$ $$v_{y}^{APF}=v_{G_{y}}^{att}+v_{O_{y}}^{rep}$$

## Breadth-First Search (BFS) Algorithm
Breadth First search is known as an uninformed search because it does not use any information about how far the robot has traveled or how far the robot is from the goal.  BFS begins at the starting position of the robot (root node) and begins looking for the goal by  expanding all of the successors of the root node.  In the scenario stated at the very start of this tutorial, the successors of a node are all allowable directions that the robot could travel next. Allowable means that directions causing the robot to crash into an obstacle,  to move outside of the workspace will not be considered as successors of the node.  Nodes that have already been visited by the robot will not be considered successors either. To do this a queue is used. All the adjacent unvisited nodes of the current level are pushed into the queue and the nodes of the current level are marked visited and popped from the queue. Breadth first search is only optimal if the path cost is the same for each direction.\
![End-to-end process of Breadth-First Search Algorithm](https://github.com/Karthik-Rajgopal/Path-Planning-Techniques--APF--BFS-and-A--/blob/main/BFS-Example-Solution-Breadth-First-Search-Algorithm-Edureka-1.png)\

## A-star (A*) Algorithm



Instructions:

1. Prepare your workspace and copy the “Milestone5” package, this roslaunch Milestone 5 <<name of the launch file>>.

NOTE: 
The launch files available are: -

Turtlebot3_Astar.launch >>> after launching this file you will see the map and movement of the turtltebot3 using A* path planning technique. 

Turtlebot3_Astar_modified.launch >>> the same as A* but with modified algorithm, related to the mechanism of A* itself that it provides more optimized path.

Turtlebot3_BFS.launch >>>  after launching this file you will see the map and movement of the turtltebot3 using A* path planning technique. 

Turtlebot3_ID_world.launch >>>  after launching this file you will see the map and movement of the turtltebot3 using APF path planning technique. 

The related “Python” files associating for all these path planning technquies can be found easily in the src folder. 





