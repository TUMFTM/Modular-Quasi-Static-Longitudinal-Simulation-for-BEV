%% Description:
%This simulation has following main steps:
%1) Preprocessing: setting the Inputs. The inputs are divided in 3 categories:
    %a) The vehicle Input are contained in section 1 of the function initialize_inputparameter
    %b) The fixed values (constant values used for the simulation) are contained in section 2 of initialize_inputparameter

%2) Acceleration Simulation
    %a) This simulation is necessary to scale the motor characteristic, which will be needed for the energy consumption simulation
    %b) If a required acceleration time is given, the torque will be scaled to reach this acceleration time
    %c) If no acceleration time is given, the acceleration time will be calculated from the given torque
    %d) Result is a definition of the needed motor characteristic (T_max, n_max and the gear ratios)

%3) Energy Consumption Simulation
    %a) Also using the results of the acceleration simulation, an energy consumptuon calculate the cosumption for the choosen cycle
    %b) If the motor/motors do not have enough torque for the cycle, the torque will be scaled accordingly
    
%4) Speed simulation:
    %a) If the User gives at the same time an i_gearbox and a max rotational speed, it may happen, that the resulting max speed is different from the Input.
    
%5) Postprocessing:
    %a) All the results are saved in the struct vehicle
    %b) Acceleration simulation results saved in vehicle.LDS.sim_acc
    %c) Energy consumption saved in struct vehicle.LDS.sim_cons
    %d) Motor and gearbox values saved in vehicle.LDS.GEARBOX and vehicle.LDS.MOTOR
    %e) If the user Inputs have been modified during the simulation (for example given torque was insufficient for given acceleration time)
        %this will be documented in the struct vehicle.LDS.ErrorLog

%This simulation was created from a concept of Lorenzo Nicoletti, implemented by Lorenzo Nicoletti and Ruben Hefele with the support of
%Alexander Koch and Sebastian Krapf. It was revised and extended by Korbinian Moller. 
        
%Authors:    Lorenzo Nicoletti, Phd Candidate, Institute of Automotive Technology, Technical University of Munich
%            Ruben Hefele, Student, Technical University of Munich
%            Korbinian Moller, Student, Technical University of Munich

%Support:   Alexander Koch, Phd Candidate, Institute of Automotive Technology, Technical University of Munich
%           Sebsatian Krapf, Phd Candidate, Institute of Automotive Technology, Technical University of Munich

%date:      10.08.2020
    
clear vehicle Parameters %clear structs from previous simulations

%% IMPORTANT: Here the User can assign the Inputs and tune the Fixed Parameter (constant value used for the simulation)!
[vehicle,Parameters] = initialize_inputparameters();

%% Load cycle
vehicle = load_cycle(vehicle,Parameters);

%% Fill the topology dependent values, fill values from the inputs:
vehicle=initialize_topology(vehicle,Parameters); 

%% Calculate missing inputs (dependent on the given Inputs): n_max, gear ratio 
vehicle=calc_missing_inputs(vehicle,Parameters);

%% Load and scale Motor characteristics
vehicle.LDS = load_engine(vehicle.LDS);

%% Acceleation simulation:
vehicle = acceleration_sim(vehicle,Parameters);

%% Max speed simulation:
vehicle=max_speed_sim(vehicle,Parameters);

%% Energy consumption simulation
vehicle = energy_consumption_sim(vehicle, Parameters);  %Calculation of energy consumption 

%% gradeability calculation
vehicle = calc_gradeability(vehicle, Parameters);
%% Plot
plotter(vehicle)

%plot_dynamic(vehicle)


