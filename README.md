# RG-RRT

## Introduction
This project plans motions for non-holonomic systems whose dynamics are described by an ordinary differential equation of the form ̇q=f(q,u),where q is a vector describing the current state of the system and uis a vector of control inputs to the system.  The systems planned for are a torque-controlled pendulum and a car-like vehicle moving in a street environment. Dynamically feasible and collision-free motions for these systems are computed using the RRT and KPIECE planners that are already implemented in OMPL. Additionally, a new planner called **Reachability-Guided RRT (RG-RRT)**, one of many variants of RRT, is implemented.

An article by Shkolnik et al. (2009) proposes a motion planning algorithm for dynamic systems. The authors propose that an RRTshould only be grown towards a random state that is likely to be reachablefrom itsnearest neighbor in the tree.  For dynamic systems, states that are near a given state are not always easilyreachable. This is the fundamental principle behind RG-RRT. The paper can be accessed at [RG-RRT, Shkolnik at al.](https://ieeexplore.ieee.org/document/5152874).

## Cloning the Repository and Installing Dependencies
To clone this repository through https:
```
git clone https://github.com/shloksobti/Reachability-Guided-Rapidly-Exploring-Random-Tree
```
The major dependency for this project is the Open Motion Planning Library (OMPL) hosted by [Kavraki Lab](http://www.kavrakilab.org/). The installation instructions can be found at [OMPL Installation](http://ompl.kavrakilab.org/installation.html).

## File Structure
```
.
|-- Makefile                    # A primitive makefile containing build instructions
|-- README                      # Repository Manual
|-- plotSolutionCar.py          # Python script to visualize solution for car
|-- plotSolutionPendulum.py     # Python script to visualize soluction for pendulum
`-- src                         # C++ source directory where all .cpp/.h files live
    |-- CollisionChecking.cpp   # Collision checking routine script contaning method implementations
    |-- "CollisionChecking.h    # Header file contaning definitions 
    |-- "Project4Car.cpp"       # C++ script for simulating Car motion planning
    |-- "Project4Pendulum.cpp"  # C++ script for simulating Pendulum motion planning
    |-- "RG-RRT.cpp"            # Definition of the RG-RRT motion planning algorithm
    `-- "RG-RRT.h"              # Header file 

```
## Usage
A Makefile is provided. Depending on your system, you may have to
adjust parts of the Makefile. Once compiled, two binary files are produced:
 1. Project3Pendulum, which prompts you for either planning or benchmarking,
 what torque for the pendulum, and in the case of planning what planner to use,
 and
 2. Project3Car, which prompts you for planning or benchmarking and in the case
of planning what planner to use.

