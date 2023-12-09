<img width="288" alt="image" src="https://github.com/Karthik-Rajgopal/Path-Planning-Techniques--APF--BFS-and-A--/assets/110279423/baa4f5b0-eb94-423a-a111-68c70e475ed7"># Path Planning Techniques (APF, BFS and A* algorithms)

This project involves testing different path planning techniques in a Gazebo simulator world comprising of obstacles and how the UGV, here Turtlebot3 managed to get to the goal position. It tackles 3 path planning techniques which are (Artificial potential field (APF), Breadth-first search (BFS), and A*).

First, the control techniques used for the Turtlebot3 to move were mainly 2 control techniques which are Lyapunov and Goal to Goal compiled in a package called "Milestone 5". 

## Artificial Path Planning (APF) Algorithm:

<div align="center">
<img src="https://github.com/Karthik-Rajgopal/Path-Planning-Techniques--APF--BFS-and-A--/blob/main/apf.jpg" width="600">
</div>

A potential field is any physical field that obeys Laplace’s equation. Some common examples of potential fields include electrical, magnetic, and gravitational fields. A potential field algorithm uses the artificial potential field to regulate a robot around in a certain space. For our ease, we consider a space to be divided into a grid of cells with obstacles and a goal node. The algorithm assigns an artificial potential field to every point in the world using the potential field functions which will be described further in the explanation. The robot simulates from the highest potential to the lowest potential. Here, the goal node has the lowest potential while the starting node will have the maximum potential. Hence, we can say that the UGV moves from the lowest to the highest potential.\
The artificial potential field (APF) algorithm is one of the algorithms used in robot path planning in which its force $F_{APF}$ is the sum of the attractive potential field $F_{att}$ and the repulsive potential field $F_{rep}$ as shown in the following equation: 
$$F_{APF}=F_{att}+F_{rep}$$ 
To apply the APF force to the robot, it is inserted into the linear speed equation on the kinematic robot. The desired speed equation in Attractive APF force $v_{G}^{att}$ is as follows: 
$$v_{G}^{att}(x,y)=-\nabla U_{att}(x,y)$$ 
where the potential attractive equation is partially derived to the x and y-axis as follows:
$$v_{x}^{att}(x,y)=-\frac{\partial U_{att}(x,y)}{\partial x}$$ $$v_{y}^{att}(x,y)=-\frac{\partial U_{att}(x,y)}{\partial y}$$ 
The equation of the Khatib’s potential attractive $U_{att}(x,y)$ is as follows: 
$$U_{att}=\frac{1}{2}k_{a}((\delta_{x}-x_{ref})^2+(\delta_{y}-y_{ref})^2)$$ 
where $k_{a}$ is the potential attractive constant, $\delta_{x}$, $\delta_{y}$ is the position of the robot. $(x_{ref}, y_{ref})$ is the position of the goal point. The desired speed equation for the Attractive APF force $v_{G}^{att}$ on the x and y-axis is as follows: 
$$v_{G_{x}}^{att}=-k_{a}(\delta_{x}-x_{ref})$$ $$v_{G_{y}}^{att}=-k_{a}(\delta_{y}-y_{ref})$$
The desired speed equation in the repulsive force $v_{O}^{rep}$ is:
$$v_{O}^{rep}(x,y)=-\nabla U_{rep}(x,y)$$
where the potential repulsive equation is partially derived to the x and y-axis as follows:
$$v_{x}^{rep}(x,y)=-\frac{\partial U_{rep}(x,y)}{\partial x}$$ $$v_{y}^{rep}(x,y)=-\frac{\partial U_{rep}(x,y)}{\partial y}$$
The equation of the Sfeir’s et al potential repulsive $U_{rep}(x,y)$ is:
<div align="center">
<img src="https://github.com/Karthik-Rajgopal/Path-Planning-Techniques--APF--BFS-and-A--/blob/main/equation.png" width= "350">
</div>

where $k_{r}$ is the potential repulsive constant, $r_{O}$ is the distance limit of potential repulsive influence, and $\rho_{O}$ is the closest distance between the robot and the obstacle. \
The closest distance between the robot and the obstacle, $\rho_{O}$ is:
$$\rho_{O}=\sqrt{(x_{ro})^2+(y_{ro})^2}$$
where $x_{or}$ is the difference of the distance between the robot and the obstacle on the x-axis, and $y_{or}$ is the difference of the distance between the robot and the obstacle on the y-axis which the equation is as follows:
$$x_{or}=\delta_{x}-x_{O}$$ $$y_{or}=\delta_{y}-y_{O}$$ 
The following is the desired speed equation for the APF Repulsive force $v_{O}^{rep}$ on the x and y-axes:
<div align="center">
<img src="https://github.com/Karthik-Rajgopal/Path-Planning-Techniques--APF--BFS-and-A--/blob/main/vx_rep.png" width= "350">
</div>
<div align="center">
<img src="https://github.com/Karthik-Rajgopal/Path-Planning-Techniques--APF--BFS-and-A--/blob/main/vy_rep.png" width= "350">
</div><br>
Thus, the speed equations of the x and y-axes in the APF are as follows:
$$v_{x}^{APF}=v_{G_{x}}^{att}+v_{O_{x}}^{rep}$$ $$v_{y}^{APF}=v_{G_{y}}^{att}+v_{O_{y}}^{rep}$$

<div align="center">
<img src="https://github.com/Karthik-Rajgopal/Path-Planning-Techniques--APF--BFS-and-A--/blob/main/apf-2.png" width="600"><br>
<strong>Potential in a local minima environment</strong>
</div><br>
## Breadth-First Search (BFS) Algorithm
Breadth First search is known as an uninformed search because it does not use any information about how far the robot has traveled or how far the robot is from the goal.  BFS begins at the starting position of the robot (root node) and begins looking for the goal by  expanding all of the successors of the root node.  In the scenario stated at the very start of this tutorial, the successors of a node are all allowable directions that the robot could travel next. Allowable means that directions causing the robot to crash into an obstacle,  to move outside of the workspace will not be considered as successors of the node.  Nodes that have already been visited by the robot will not be considered successors either. To do this a queue is used. All the adjacent unvisited nodes of the current level are pushed into the queue and the nodes of the current level are marked visited and popped from the queue. Breadth-first search is only optimal if the path cost is the same for each direction.\
![End-to-end process of Breadth-First Search Algorithm](https://github.com/Karthik-Rajgopal/Path-Planning-Techniques--APF--BFS-and-A--/blob/main/BFS-Example-Solution-Breadth-First-Search-Algorithm-Edureka-1.png)

## A-star (A*) Algorithm
A* Search is an informed best-first search algorithm that efficiently determines the lowest cost path between any two nodes in a directed weighted graph with non-negative edge weights. This algorithm is a variant of Dijkstra’s algorithm. A slight difference arises from the fact that an evaluation function is used to determine which node to explore next. \
The evaluation function, f(x), for the A* search algorithm is the following:
$$f(x)=g(x)+h(x)$$
where $g(x)$ represents the cost to get to node $x$ and $h(x)$ represents the estimated cost to arrive at the goal node from node $x$.

The A* algorithm is implemented in a similar way to Dijkstra’s algorithm. Given a weighted graph with non-negative edge weights, to find the lowest-cost path from a start node S to a goal node G, two lists are used:
* An open list, implemented as a priority queue, which stores the next nodes to be explored. Because this is a priority queue, the most promising candidate node (the one with the lowest value from the evaluation function) is always at the top. Initially, the only node in this list is the start node S.
* A closed list that stores the nodes that have already been evaluated. When a node is in the closed list, it means that the lowest-cost path to that node has been found.

To find the lowest cost path, a search tree is constructed in the following way:

1. Initialize a tree with the root node being the start node S.
2. Remove the top node from the open list for exploration.
3. Add the current node to the closed list.
4. Add all nodes that have an incoming edge from the current node as child nodes in the tree.
5. Update the lowest cost to reach the child node.
6. Compute the evaluation function for every child node and add them to the open list.

All nodes except for the start node start with the lowest cost of infinity. The start node has an initial lowest cost of 0. The algorithm concludes when the goal node G is removed from the open list and explored, indicating that the shortest path has been found. The shortest path is the path from the start node S to the goal node G in the tree.

### Difference between Best-first search and A*search:

The best-first search and A* search algorithm both define the evaluation function for each node $n$ of the graph. This evaluation function is denoted by $f(n)$. This evaluation function determines which node to be expanded first while searching. Both search algorithms mainly differ in their evaluation function. Here, $f(n)$ is the evaluation function, $g(n)$ is the past knowledge acquired while searching, and $h(n)$ is the heuristics function. 

The difference between the best-fit and A* search algorithms is represented by the following table.

<div align="center">
<img src="https://github.com/Karthik-Rajgopal/Path-Planning-Techniques--APF--BFS-and-A--/blob/main/diffbfsandastar.png" width="900">
</div>

# Instructions:

Prepare your workspace and copy the “Milestone5” package, this roslaunch Milestone 5 <name of the launch file>.

**NOTE :** \
The launch files available are: -

* Turtlebot3_Astar.launch >>> After launching this file you will see the map and movement of the turtltebot3 using the A* path planning technique. 

* Turtlebot3_Astar_modified.launch >>> the same as A* but with a modified algorithm, related to the mechanism of A* itself that provides a more optimized path.

* Turtlebot3_BFS.launch >>>  After launching this file you will see the map and movement of the turtltebot3 using the A* path planning technique. 

* Turtlebot3_ID_world.launch >>>  After launching this file you will see the map and movement of the turtltebot3 using the APF path planning technique. 

The related “Python” files associated with all these path-planning techniques can be found in the src folder. 





