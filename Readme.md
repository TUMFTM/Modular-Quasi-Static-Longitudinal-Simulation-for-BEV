

# Developer
The main developers of this simulation are: Adrian König, Lorenzo Nicoletti, and Korbinian Moller (Institute for Automotive Technology, Technical University of Munich).

------- An Open-Source Modular Quasi-Static Longitudinal Simulation for Full Electric Vehicles ------
Authors: Adrian König, Lorenzo Nicoletti, Korbinian Moller
Link: https://ieeexplore.ieee.org/document/9242981

This is the readme file for MATLAB longitudinal dynamics simulation. This simulation calculates the motor torque, motor power and energy consumption within a given driving cycle based on the given input parameters. 

To start the simulation you only have to call the function main.m. This function coordinates all further subfunctions. In the first function (initialize_inputparameters_xxx) different preset vehicles can be selected. These can easily be found with the tab key (press behind the last underscore). Furthermore, all parameters can be changed in the options. These are explained separately in the function. 

The output is given as a plot. Furthermore, all calculated parameters are output in the MATLAB struct. There information about the acceleration simulation (sim_acc), the speed simulation (max_speed_sim), the consumption simulation (sim_cons) as well as data about the engine (front and rear axle) and the transmission (front and rear axle) are stored. 

As an additional feature, a dynamic plot is available which illustrates the cycle and all operating points. Additionally, several gears can be simulated. For this purpose, several transmission ratios only have to be entered as line vectors in the gearbox input. 

For a four-wheel drive, the corresponding topologies must be defined. These are assigned according to the scheme (front wheel)_(rear wheel). X stands for an unoccupied axle. One motor with one gear per axle is called GM, two motors with two gears per axle are called 2G2M. Combinations are also possible. GM_2G2M thus designates a vehicle with one engine on the front axle and two engines on the rear axle. GM_X denotes for example a front wheel drive. 

Steps to execution:
- Select vehicle
- change parameters if required
- Start simulation
- Evaluate results



(c) 2020
