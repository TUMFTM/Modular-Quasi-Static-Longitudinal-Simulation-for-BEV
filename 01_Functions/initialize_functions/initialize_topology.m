function [vehicle] = initialize_topology(vehicle,Par)
% Designed by: Lorenzo Nicoletti (FTM, Technical University of Munich), Korbinian Moller
%-------------
% Created on: 06.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: According to the given topology, the needed values for the LDS get moved
%              into the LDS struct. The function is structured in parameters which are
%              valid for all topologies and which are only valid for the exact topology
% ------------
% Sources: More information regarding the implementation of the LDS functions is available at:
%          [2] K. Moller, „Validierung einer MATLAB Längsdynamiksimulation für die Auslegung von Elektrofahrzeugen,“ Bachelor thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2020. 
%          [3] K. Moller, „Antriebsstrangmodellierung zur Optimierung autonomer Elektrofahrzeuge,“Semester thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2021.
% ------------
% Input: veh: The vehicle structure -> Stores the calculated component volumes and masses
%        Par: The Parameters structure -> Stores the constant values and regressions for volume and mass models
% ------------
% Output: The vehicle structure updated with the LDS simulation parameters
% ------------
%% Implementation:
%1) Assign topology-dependent parameters, initialize MOTOR and GEARBOX struct
%2) Assign topology-indipendent parameters, initialize weight, wheel, parameters and setting struct

%% 1) Assign topology-dependent parameters, initialize MOTOR and GEARBOX struct
vehicle.LDS.settings.topology=vehicle.Input.topology;

%Check if max_speed input is lower as max cycle speed:
vehicle=check_max_speed(vehicle,Par);                                           

%initialize powertrain only at first simulation run
if vehicle.LDS.First_Run == 1
    initialize_powertrain = str2func(['initialize_powertrain_', vehicle.LDS.settings.topology]);                                            
    vehicle = initialize_powertrain(vehicle,Par);
end

%Find which axles are filled: [front_axle, rear_axle] and save for later
vehicle.LDS.settings.filled_axles = [~isempty(vehicle.LDS.MOTOR{1}),~isempty(vehicle.LDS.MOTOR{2})];

%Save characteristics path
vehicle.LDS.settings.path = Par.LDS.char_path;

%% 2) Assign topology-indipendent parameters, initialize weight, wheel, parameters and setting struct
%The acceleration and cosumption sumulation run with vehicle empty weight with driver in kg
vehicle.LDS.weights.vehicle_empty_weight_EU = vehicle.masses.vehicle_empty_weight_EU;

%if different mass for consumption simulation is required, it is set here
if ~isnan(vehicle.masses.vehicle_sim_cons_weight)
    vehicle.LDS.weights.vehicle_sim_cons_weight = vehicle.masses.vehicle_sim_cons_weight;   
else
    vehicle.LDS.weights.vehicle_sim_cons_weight = vehicle.masses.vehicle_empty_weight_EU;
end

%Wheel parameters:
if isfield(vehicle.dimensions.CX,'wheel_f_diameter')
    vehicle.LDS.wheel.r_tire = vehicle.dimensions.CX.wheel_f_diameter/2;  %static radius of the tire in mm
    vehicle.LDS.wheel.w_tire = vehicle.dimensions.CY.wheel_f_width;       %tire width in mm
    vehicle.LDS.wheel.r_dyn  = vehicle.LDS.wheel.r_tire*1.02;             %dynamic radius in mm
else
    vehicle.LDS.wheel.r_dyn  = vehicle.Input.r_tire*1.02;                 %dynamic radius in mm
    vehicle.LDS.wheel.r_tire = vehicle.Input.r_tire;                      %static radius of the tire in mm 
    vehicle.LDS.wheel.w_tire = vehicle.Input.w_tire;                      %width of tire in mm 
end

%Vehicle parameters:                                                                                                         %Assign c_r
vehicle.LDS.parameters.wheelbase      = vehicle.Input.wheelbase;                                                             %wheelbase in mm
vehicle.LDS.parameters.vehicle_height = vehicle.Input.vehicle_height;                                                        %total height in mm
vehicle.LDS.parameters.vehicle_width  = vehicle.Input.vehicle_width;                                                         %total width in mm
vehicle.LDS.parameters.A              = vehicle.Input.vehicle_width *vehicle.Input.vehicle_height*Par.LDS.correction_area;   %cross sectional area in mm^2
vehicle.LDS.parameters.height_COG     = Par.regr.LDS.height_COG.eq(vehicle.LDS.parameters.vehicle_height);                   %Height of COG from the ground in mm

%Simulation settings:
vehicle.LDS.settings.v_max_sim         = Par.LDS.v_max_sim;                   %Required speed for testing acceleration time (standard valuze 100 Km/h)
vehicle.LDS.settings.t_sim_max_acc     = Par.LDS.t_sim;                 %Assign the max simulation time in s for the acceleration from 0 to 100
vehicle.LDS.settings.power_auxiliaries = vehicle.Input.power_auxiliaries;   %Assign power auxiliaries value

%Engine characteristic settings:
vehicle.LDS.settings.characteristic_forced = vehicle.Input.characteristic_forced;       %forced characteristic per axle, otherwise NaN
vehicle.LDS.settings.ratio_req = vehicle.Input.ratio_req;                               %required angular speed ratio per axle
vehicle.LDS.settings.weight_T = vehicle.Input.weight_T;                                 %Torque weight factor per axle
vehicle.LDS.settings.weight_n = vehicle.Input.weight_n;                                 %speed weight factor per axle
vehicle.LDS.settings.weight_ratio = vehicle.Input.weight_ratio;                         %angular speed ratio weight factor per axle

%Torque optimizer if only one engine is overloaded - on/off (1/NaN)
vehicle.LDS.settings.torque_optimizer = Par.LDS.torque_optimizer;

%sim acc requirements:
vehicle.LDS.sim_acc.acc_time_req = vehicle.Input.acceleration_time_req;  %Required acceleration time in s
%The max speed was already set by the function check v_max
end