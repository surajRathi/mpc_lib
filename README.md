# mpc_lib
Uses Model Predictive Control for path planning of a differential drive vehicle. Uses [IPOPT](https://coin-or.github.io/Ipopt/) to solve the non-linear optimization problem.
Intended to be used with [surajRathi/mpc_local_planner](https://github.com/surajRathi/mpc_local_planner) as a local planner for the ROS Nav Stack.

Forked from [project-TARIV/mpc_lib](https://github.com/project-TARIV/mpc_lib) to add documentation.

## Dependencties
- **cmake**: Build System
- **catkin** OPTIONAL  
  Project is made to be compiled with catkin for the ROS framework, but does work with just cmake.
  
- **cppad**
For Ubuntu:
```bash
sudo apt-get install cppad
```

- **Ipopt**
This installs it to `/usr/local`. You can edit the `install_ipopt.sh` before execution to modify that
For Ubuntu:
```bash
sudo apt-get install gfortran
sudo apt-get install unzip
wget 'https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.zip'
unzip Ipopt-3.12.7.zip
rm Ipopt-3.12.7.zip
sudo bash install_ipopt.sh ./Ipopt-3.12.7/
```

## Working
We solve an optimisation problem for acceleration in each wheel.

1. A simple kinematic model has been selected where $v_r$ and $v_l$ are velocities of each wheel, $a_r$ and $a_l$ are the respective accelerations, and $L$ is the wheelbase.  

    - $x_i = x_{i-1} + \frac{v_r + v_l}{2} * cos(\theta_{i-1}) * dt$  
    - $y_i = y_{i-1} + \frac{v_r + v_l}{2} * sin(\theta_{i-1}) * dt$  
    - $\theta_i = \theta_{i-1} + \frac{v_r - v_l}{L} * dt$  

2. Constraints: We enforce maximum and minimum constraints 

3. Objective Function: This is of the form $\sum{w_i*(\text{cost}_i)^2}$. The factors, $\text{cost}_i$ are listed below. We must choose appropriate weights, $w_i$ for each of them. The path to follow has been defined as a polynomial $f(x)$.
    - $v_r + v_l - 2 * v_{ref}$
    - $v_r - v_l$
    - $a_r + a_l$
    - $a_r - a_l$
    - $f(x) - y$

## References
