# Roboxi (Robotics Toolbox) Project
This Robotics Toolbox offers an easy to plug robotics algorithms into a simulation environment. The main idea create a modular simulation environment for robotics systems, in which several algorithms can be simple used.
We plan to cover the main areas in robotics such as perception, localization, mapping and motion planning. Any feedback is welcome =D!

## Models
The following models are currently available for the simulation:
* Simple car/bicycle model
* Differential Drive (progress)
* Extended Bicycle Kinematic model (progress)
* Front wheel drive (progress)
* Bicycle Dynamic model (todo)
* ...

## Solvers
The following solvers are currently available for the simulation:
* Euler solver
* Runge Kutta 4th Order
* ...

## Map
For now, simple grid map can be created with squared obstacles.
* Circular static obstacles (todo)
* Dynamical obstacles (todo)
* ...

## Localization/Maping
Currently no filters or localization algorithms are implemented. Ideas:
* Kalman Filter (todo)
* Particle Filter (todo)
* SLAM (todo)
* ...

## Planners
To find a path from the initial pose to the goal pose its possible to use a planning algorithm. The following planners are implemented:
* A* Planner
* RRT
* RRT* (todo)
* PRM (todo)
* ...

## Controllers
For each model one or more controllers can be used.
* Simple car/bicycle model
	* Simple pose controller
	* Linear Quadratic Regulator (Todo)
* Differential Drive (Todo)
	* Simple pose controller (Todo)
	* Linear Quadratic Regulator (Todo)
* Extended Bicycle Kinematic model (Todo)
	* Simple pose controller (Todo)
	* Linear Quadratic Regulator (Todo)
* Front wheel drive (Todo)
	* Simple pose controller (Todo)
	* Linear Quadratic Regulator (Todo)
* MPC and IT-MPC (todos for any model)
* ...

## Further Ideas
Additional ideas for the future, when all basic part are implemented
* Prediction of dynamical obstacles
* Behaviour planning
* Reiforcement Learning
* ...

### Next todos
* OOP
* transfer the models and controller to robots
* make an upper class planners