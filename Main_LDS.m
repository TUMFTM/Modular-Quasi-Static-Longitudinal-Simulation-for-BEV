% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich), Korbinian Moller
%-------------
% Finalized on: 22.03.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This Main can be used to conduct a simulation og the LDS without using the entire tool
%              For more information regarding the LDS function see the
%              function calc_longitudinal_simulation. 
% ------------
% Sources: General information regarding the LDS
%          [1] A. König, L. Nicoletti, S. Kalt, K. Moller, A. Koch and M. Lienkamp, „An Open-Source Modular Quasi-Static Longitudinal Simulation for Full Electric Vehicles,“ in 15th International Conference on Ecological Vehicles and Renewable Energies, Monte-Carlo, Monaco, 2020, pp. 1–9, DOI: 10.1109/EVER48776.2020.9242981.
%          [2] Lorenzo Nicoletti, "Parametric Modeling of Battery Electric Vehicles in the Early Development Phase", Ph.D. Thesis, Technical University of Munich, Institute of Automotive Technology, 2022

%          The following theses developed and improved the longitudinal simulation
%          [3] K. Moller, „Validierung einer MATLAB Längsdynamiksimulation für die Auslegung von Elektrofahrzeugen,“ Bachelor thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2020. 
%          [4] K. Moller, „Antriebsstrangmodellierung zur Optimierung autonomer Elektrofahrzeuge,“Semester thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2021.
%          [5] R. Hefele, „Implementierung einer MATLAB Längsdynamiksimulation für Elektrofahrzeuge,“Semester thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2019.         
% ------------
% Input: vehicle: The vehicle structure -> Stores the calculated component volumes and masses
%        Parameters: The Parameters structure -> Stores the constant values and regressions for volume and mass models
%        This inputs are initialized by the scripts
%        'initialize_input_parameters_XXX' it is possible to select
%        different scripts to simulate different vehicle. These scripts are
%        contained in the folder 03_LDS/01_Function/initialize_functions
%------------
% Output: The vehicle structure updated with:
%         -the vehicle consumption
%         -the required battery energy
%         -the resistance forces
%         -the achievable range
%         -the machine torque, rotational speed etc.
%         -the results of acceleration and max speed simulation
% ------------    
%% Implementation
%1) Preprocessing
%2) Load cycles, efficiency map, and calculate missing inputs
%3) Acceleration Simulation
%4) Top Speed simulation    
%5) Energy Consumption Simulation   
%6) Postprocessing

%% 1) Preprocessing:
clear vehicle Parameters %clear structs from previous simulations

%Here the User can assign the Inputs and tune the Fixed Parameter (constant value used for the simulation)!
[vehicle,Parameters] = initialize_inputparameters_etron55();

vehicle.LDS.settings.suppress_LDS_warnings=1; %Set to 1 to suppress the LDS warnings and to 0 to show them
vehicle.settings.suppress_LDS_warnings=1; %Set to 1 to suppress the LDS warnings and to 0 to show them

vehicle.LDS.First_Run=1;

%% 2) Load cycles, efficiency map, and calculate missing inputs
%Load the selected consumption cycle
vehicle = load_cycle(vehicle,Parameters);

%Initialize the topology
vehicle=initialize_topology(vehicle,Parameters); 

%Calculate missing inputs (dependent on the given Inputs): n_max, gear ratio
vehicle=calc_missing_inputs(vehicle,Parameters);

%Load the efficiency map for the motor
vehicle.LDS = load_engine(vehicle.LDS);

%% 3) Acceleration simulation
%-This simulation is necessary to scale the motor characteristic, which will be needed for the energy consumption simulation
%-The machine torque will be scaled to reach the given acceleration time
%-The output is a definition of the required motor characteristic (T_max, n_max and the gear ratios)
vehicle = acceleration_sim(vehicle, Parameters);

%% 4) Top speed simulation
vehicle=max_speed_sim(vehicle,Parameters);

%% 5) Energy consumption simulation
%-Also using the results of the acceleration simulation, an energy consumptuon calculate the cosumption for the choosen cycle
%-If the motor/motors do not have enough torque for the cycle, the torque will be scaled accordingly
vehicle = energy_consumption_sim(vehicle, Parameters);  %Calculation of energy consumption 

%% 6) Postprocessing:
%Plot the results of the energy consumption and acceleration simulations
plot_result_LDS(vehicle)