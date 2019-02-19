# RG-RRT

## Introduction


## Cloning the Repository and Installing Dependencies

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

