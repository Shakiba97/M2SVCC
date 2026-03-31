# M2SVCC (Multimodal Multiscale Signal-Vehicle Coupled Control)

## About
This project implements a Model Predictive Control (MPC)–based Mixed-Integer Nonlinear Programming (MINLP) **optimization framework** for multimodal signal–vehicle coupled control with Connected and Automated Vehicles (CAVs). Within a Python-based environment, the system operates in a closed loop where traffic states are continuously updated through SUMO, optimized control actions are generated using GAMS, and the resulting signal timings and vehicle trajectories are fed back into the simulation for real-time control. The framework jointly optimizes traffic signal timing and CAV trajectories to maximize throughput, minimize user delays, and reduce fuel and energy consumption at signalized intersections, and has been successfully **deployed and validated in the real-world** Mcity Test Facility.

## Requirements
Python 3.12   
GAMS 46.5 ([Download](https://www.gams.com/download/))  
SUMO 1.20.0  ([Download](https://eclipse.dev/sumo/))  
traci  
sumolib  
numpy  
gamsapi  
matplotlib  

### Installation
Please follow the instruction on [this link](https://www.gams.com/latest/docs/API_PY_GETTING_STARTED.html) for installing the GAMS dependencies and troubleshooting if needed. 

## Structure
- `agent`: Contains the MPC Agent Class which includes the MPC Optimization process.  
    - gams_models: Contains GAMS files solving the Slower-Scale (A2), and Faster-Scale (A3) Optimization problems (refer to the paper [Guo and Ban (2023)](https://www.sciencedirect.com/science/article/abs/pii/S0191261523001121))  
- `config`: Contains functions for setting up the model parameters.
- `environment`: Contains SUMO files and configurations as well as single_intersection class which covers the route builder and network generator of simulation as well as the Input, Output Interfaces (communication with SUMO) for the Unified 4-leg intersection.
- `Results`: Contains Output files generated at the end of simulation provide the following metrics to compare the MMSVCC project performance with other signal timing scenarios (e.g., fixed-time and actuated)
- `Slides`: Contains detailed diagram and documentation on the algorithm.

The whole process is summarized in the diagram below:  

![MPC Agent Diagram](Slides/Diagram2.png)
Detailed documentiations of the algorithm can be found in /Slides/documentation.docx.  


## Real-world testing and results
Deployment at Mcity physical automated vehicle testbed, University of Michigan:  
![Deployment at Mcity physical automated vehicle testbed, University of Michigan](Slides/Screencast-from-2024-07-09-12-28-35.gif)

#### Performance Metrics:  

<p align="center">
  <img src="Slides/Picture1.png" width="500">
</p>

## Contributers
- Qiangqiang Guo
- Shakiba Naderian (naderian@uw.edu)

