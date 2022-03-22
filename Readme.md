<h1><a ...section link code />A Modular Quasi Static Longitudinal Simulation for BEV</h1>

<h2><a ...section link code />Developers:</h2>

- Adrian König (Institute for Automotive Technology, Technical University of Munich): Simulation validation, Code detailing, Code documentation
- Lorenzo Nicoletti (Institute for Automotive Technology, Technical University of Munich): Creation of first simulation code, Code detailing, Code documentation
- Korbinian Moller (Technical University of Munich): Simulation validation, Code detailing, Code documentation

<h2><a ...section link code />Structure of the Longitudinal Dynamic Simulation (LDS):</h2>

The structure of the LDS is shown in the figure underneath, based on [[1]](#1):

<div align="center">
<img src="/04_Visualization/LDS_structure.svg"
 alt="Structure of the LDS"
title="Structure of the LDS"
width = "1000px"
/>
</div>


The LDS inputs include performance requirements (maximum vehicle speed, acceleration time, target range, and gearbox transmission ratio), parameters for estimating vehicle losses (external dimensions and vehicle mass), and topology specifications (number, type, and position of the electric machines). Furthermore, for estimating the vehicle range, a test cycle must be selected. It is possible to choose among several driving cycles such as the New European Driving Cycle (NEDC) and the Worldwide Harmonized Light Vehicles Test Procedure (WLTP).

In step 1, the machine torque is scaled until the desired acceleration time is reached. This step also derives the torque curve, which depicts the profile of the maximum torque as a function of its rotational speed. 

Subsequently, a top speed simulation (step 2 in the Figure) is employed to select the maximum machine rotational speed depending on a given transmission ratio and vehicle speed. If the previously calculated (step 1 in the Figure) torque curve does not extend up to the required rotational speed, the algorithm scales the curve accordingly. 

Finally in the last step an energy consumption conducted. For this scope a test cycle (which has to be selected by the user) is used. Based on the speed and profile of the test cycle the losses of the vehicle are calculated and ultimatively its consumption dervied. For the machine losses, an efficiency map database is created with the design software developed by Kalt et al. [[2]](#2). The simulation selects the efficiency map from the database that best approximates the torque curve derived in steps 1 and 2. The gearbox, battery, and power electronics losses are modeled with constant efficiencies. 

The LDS outputs include the electric machine requirements (maximum torque Tmach,max and rotational speed nmach,max), the vehicle consumption in the test cycle, and the required battery energy to reach the target range.

<h2><a ...section link code />Implementation:</h2>

This section describes the Implementation of the LDS.  The LDS can be simply started by calling [```Main_LDS.m```](/Main_LDS.m). The description of the LDS implementation is based on the structure of ```Main_LDS.m```.
 

<h3><a ...section link code />Preprocessing:</h3>

In the preprocessing, the to-be-simulated vehicle is initialized. To initialize the vehicle you can use one of the initialization scripts contained in [```initialize_vehicle```](/01_Functions/initialize_functions/initialize_vehicle/). Otherwise you can also create your own script based on the scripts contained in ```initialize_vehicle```. After this step, the structures  ```vehicle``` and ```Parameters``` are created:

- Struct ```vehicle```: This struct contains the input given by the user and will also be used to store the results of the LDS. It is created and initialized by the initialization scripts and is filled with results during the calculation. The ```vehicle``` struct is also important for the postprocessing: for example, the results of the LDS can be visualized by calling the following line of code: ```plot_result_LDS(vehicle)```
- Struct ```Parameters```: Differently from the ```vehicle``` struct, this struct is not updated during the calculation. It contains empirical models as well as different constant values which are required for the calculation (such as gearbox, battery, and power electronic efficiencies).

<h3><a ...section link code />Load cycles, efficiency map, and calculate missing inputs:</h3>

This section is divided into four main steps:
- The function [```load_cycle.m```](/01_Functions/helper_functions/load_cycle.m) loads the chosen test cycle. The speed-time profile of the test cycle is appended to the structure ```vehicle``` and will be used in the consumption simulation.
- The function [```initialize_topology.m```](/01_Functions/initialize_functions/initialize_topology/initialize_topology.m) adds new fields to the vehicle struct. These fields store the type, postion and number of machines and gearbox. The machines and gearboxes are initialized based on the chosen topology.
- The function [```calc_missing_inputs.m```](/01_Functions/calc_functions/calc_missing_inputs.m) calculates further parameters required for the LDS using the user inputs.
- The function [```load_engine.m```](/01_Functions/engine_functions/load_engine.m) loads a basis machine efficiency map which will be used as starting point for the consumption simulation.

<h3><a ...section link code />Acceleration simulation:</h3>

The acceleration simulation is conducted by the script [```acceleration_sim.m```](/01_Functions/simulation_functions/acceleration_sim.m). This function will iteratively adjust the torque of the electric machine until it is possible to reach the required vehicle acceleration time. Precise documentation of this function is contained in [[3]](#3).

<h3><a ...section link code />Top speed simulation:</h3>

The top speed simulation is conducted by the script [```max_speed_sim.m```](/01_Functions/simulation_functions/max_speed_sim.m). This function will iteratively adjust the machine rotational speed until the desired vehicle speed can be reached. Precise documentation of this function is contained in [[3]](#3).

<h3><a ...section link code />Consumption simulation:</h3>

The consumption simulation is conducted by the script [```energy_consumption_sim.m```](/01_Functions/simulation_functions/energy_consumption_sim.m). This function will iteratively use the derived machine characteristics (torque and rotational speed) and simulate the machine efficiency in the chosen test cycle. Depending on the chosen test cycle, it may happen that the machine can fulfill the vehicle speed and acceleration requirement but is not enough powerful to be used in the cycle. If this is the case, the machine will be scaled once again to ensure that the vehicle can follow the given test cycle. 


<h3><a ...section link code />Postprocessing:</h3>

The results of the LDS can be plotted by calling the function [```plot_result_LDS.m```](../04_Visualization/plot_result_LDS.m)

<h2><a ...section link code />References:</h2>

<a id="1">[1]</a> Lorenzo Nicoletti, "Parametric Modeling of Battery Electric Vehicles in the Early Development Phase", Ph.D. Thesis, Technical University of Munich, Institute of Automotive Technology, 2022.

<a id="2">[2]</a> Svenja Kalt et al., „Electric Machine Design Tool for Permanent Magnet Synchronous Machines and Induction Machines,“ Machines, vol. 8, no. 1, p. 15, 2020, DOI: 10.3390/machines8010015.

<a id="3">[3]</a> Adrian König, Lorenzo Nicoletti et. al. „An Open-Source Modular Quasi-Static Longitudinal Simulation for Full Electric Vehicles,“ in 15th International Conference on Ecological Vehicles and Renewable Energies, Monte-Carlo, Monaco, 2020, pp. 1–9, DOI: 10.1109/EVER48776.2020.9242981.
