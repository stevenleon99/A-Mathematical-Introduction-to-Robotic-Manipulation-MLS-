# Robot Path Planning

This folder contains MATLAB classes and scripts for performing path planning for robotic systems using Probabilistic Roadmap (PRM) and Rapidly-exploring Random Trees (RRT) algorithms.

## Features

- **PRM (Probabilistic Roadmap)**: Suitable for higher-dimensional spaces with holonomic constraints.
- **RRT (Rapidly-exploring Random Trees)**: Effective in high-dimensional spaces, particularly with non-holonomic constraints.

## Classes Overview

### PRM (Probabilistic Roadmap)

The `PRM` class is designed to implement the Probabilistic Roadmap method for path planning in robotics.

#### Properties

- `qI`: Initial configuration point of the robot in the configuration space.
- `qG`: Goal configuration point of the robot in the configuration space.
- `dim`: Dimensionality of the configuration space.
- `n`: Number of vertices to sample in the PRM.
- `K`: Number of closest neighbors to connect each vertex to.
- `O`: Cell array containing obstacles, where each obstacle is defined by a matrix of its vertex coordinates.
- `qmin`: Minimum boundary of the configuration space.
- `qmax`: Maximum boundary of the configuration space.
- `robot`: The robot object which may include specific methods such as updating joint angles or checking for collisions.
- `V`: Array storing the vertices of the graph.
- `G_matrix`: Sparse matrix representing the weighted adjacency matrix of the graph.
- `G`: Graph object representing the PRM.

#### Methods

- `constructor(qI, qG, n, K, O, qmin, qmax, robot)`: Initializes a new instance of PRM.
- `buildPRM()`: Generates the roadmap by sampling points and connecting them based on the proximity and absence of obstacles.
- `findPath()`: Computes the shortest path from the initial to the goal point using the constructed roadmap.
- `isInCollision(S)`: Checks if a proposed edge intersects any obstacle in the space.

### RRT (Rapidly-exploring Random Trees)

The `RRT` class implements the Rapidly-exploring Random Tree method, suitable for complex path planning problems.

#### Properties

- Similar properties as in the `PRM` class but includes `del_q` which is the step size used for extending the tree towards sampled points and `tolerance` for how close the tree must get to the goal to consider it reached.

#### Methods

- `constructor(qI, qG, n, del_q, O, qmin, qmax, robot, tolerance)`: Initializes a new instance of RRT.
- `buildRRT()`: Builds the RRT by iteratively extending towards randomly sampled points.
- `findPath()`: Finds a path from the initial to the goal point by tracing back through the tree.
- `Nearest_Vertex(qrand, V)`: Identifies the nearest vertex in the existing tree to a randomly sampled point.
- `New_Conf(qnear, qrand)`: Calculates a new configuration towards which the tree should grow.
- `isInCollision(S)`: Checks if a movement between two configurations would intersect with any obstacles.

## Usage

### Setting Up Your Robot and Obstacles

1. Define the obstacles in your environment.
2. Specify the initial (`qI`) and goal (`qG`) configurations.
3. Initialize the robot configuration according to your robot's specifications.

### Running PRM

To run the Probabilistic Roadmap method:

```matlab
prm = PRM(q_init, q_goal, N, K, B, 0, 2*pi, robot);
Path = prm.findPath();
```

![PRM](./.data/PRM.gif)

### Running RRT

To run the Rapidly-exploring Random Trees method:

```matlab
rrt = RRT(q_init, q_goal, N, delta_q, B, 0, 2*pi, robot, tolerance);
Path = rrt.findPath();
```

![RRT](./.data/RRT.gif)
