<h1><a ...section link code />A Modular Quasi Static Longitudinal Simulation for BEV</h1>
<h2><a ...section link code />Developers:</h2>
- Adrian König (Institute for Automotive Technology, Technical University of Munich): Simulation validation, Code detailing, Code documentation
- Lorenzo Nicoletti (Institute for Automotive Technology, Technical University of Munich): Creation of first simulation code, Code detailing, Code documentation
- Korbinian Moller (Technical University of Munich): Simulation validation, Code detailing, Code documentation

<h2><a ...section link code />Structure of the Longitudinal Dynamic Simulation (LDS):</h2>

The structure of the LDS is shown in the figure underneath, based on [[1]](#1):

<div align="center">
<img src="/04_Visualization/LDS_structure.svg?raw=true"
 alt="Structure of the LDS"
title="Structure of the LDS"
width = "555px"
/>
</div>

The LDS inputs include performance requirements (maximum vehicle speed, acceleration time t0􀀀100, target range, and gearbox transmission ratio), parameters for estimating vehicle losses (external dimensions and vehicle mass), and topology specifications (number, type, and position of the electric machines). Furthermore, for estimating the vehicle range, a test cycle must be selected. It is possible to choose among several driving cycles such as the New European Driving Cycle (NEDC) and the Worldwide Harmonized Light Vehicles Test Procedure (WLTP).

In step 1, the machine torque is scaled until the desired acceleration time is reached. This step also derives the torque curve, which depicts the profile of the maximum torque as a function of its rotational speed. 

Subsequently, a top speed simulation (step 2 in the Figure) is employed to select the maximum machine rotational speed depending on a given transmission ratio and vehicle speed. If the previously calculated (step 1 in the Figure) torque curve does not extend up to the required rotational speed, the algorithm scales the curve accordingly. 

Finally in the last step an energy consumption conducted. For this scope a test cycle (which has to be selected by the user) is used. Based on the speed and profile of the test cycle the losses of the vehicle are calculated and ultimatively its consumption dervied. For the machine losses, an efficiency map database is created with the design software developed by Kalt et al. [[2]](#2). The simulation selects the efficiency map from the database that best approximates the torque curve derived in steps 1 and 2. The gearbox, battery, and power electronics losses are modeled with constant efficiencies. 

The LDS outputs include the electric machine requirements (maximum torque Tmach,max and rotational speed nmach,max), the vehicle consumption in the test cycle, and the required battery energy to reach the target range.

<h2><a ...section link code />Implementation:</h2>

This section describes the Implementation of the LDS.  The LDS can be simply started by calling [```Main_LDS.m```](../Main_LDS.m). The description of the LDS implementation bases on the structure of ```Main_LDS.m```.
 

<h3><a ...section link code />Preprocessing:</h3>
In the preprocessing the to-be-simulated vehicle is initialized. To initialize the vehicle you can use one of the initialization scripts contained in HERE LINK. Otgherwise you can also create your own script based on the scripts contained in HERE LINK. After this step, the structures  ```Main_LDS.m``` and ```Main_LDS.m``` are created:

- Struct ```vehicle```: This struct is used to store the results of the LDS. It is created and initialized by the initialization scripts and is filled with results during the calculation. The ```vehicle``` struct is also improtant for the postprocessing: for example, the results of the LDS can be visualized by calling the following line of code: ```plot_result_LDS(vehicle)```
- Struct ```Parameters```: Differently from the ```vehicle``` struct, this struct is not updated during the calculation. It contains empirical models as well as different constant values which are required for the calculation (such as gearbox, battery, and power electronic efficiencies).

<h3><a ...section link code />Load cycles, efficiency map, and calculate missing inputs:</h3>
This section is divided in four main steps:
- The function 


- Load the cycle: This tep is conducted at the following line of Main_LDS. Based on the chosen consumption cycle (will be exmplained later) the speed time profile is loaded from the  [```03_Drive_Cycle```](../03_Drive_Cycle/) folder.
- 
- 

This calculation step is implemented in the script [```acceleration_sim```](../01_Functions/simulation_functions/acceleration_sim.m)

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


## References
<a id="1">[1]</a> Lorenzo Nicoletti, "Parametric Modeling of Battery Electric Vehicles in the Early Development Phase", Ph.D. Thesis, Technical University of Munich, Institute of Automotive Technology, 2022.

<a id="2">[2]</a> Svenja Kalt et al., „Electric Machine Design Tool for Permanent Magnet Synchronous Machines and Induction Machines,“ Machines, vol. 8, no. 1, p. 15, 2020, DOI: 10.3390/machines8010015.

