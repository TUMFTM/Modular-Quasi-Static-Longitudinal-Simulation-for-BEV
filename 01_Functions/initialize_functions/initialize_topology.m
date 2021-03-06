function [v] = initialize_topology(v,Par)
%% Description: 
%according to the given topology, the needed values for the LDS get moved
%into the LDS struct. The function is structured in parameters which are
%valid for all topologies and which are only valid for the exact topology

%author:    Lorenzo Nicoletti, FTM, TUM

%date:      20.05.2020

%% Inputs:
% vehicle vector
% Parameters vector
%% Outputs:
% vehicle vector
%% Sources:
%%source for height cog: Berechnung und Rekonstruktion des Bremsverhaltens von
%PKW - Burckhardt, S. 167
%% Implementation:
%1) Assign topology-dependent parameters, initialize MOTOR and GEARBOX struct
%2) Assign topology-indipendent parameters, initialize weight, wheel, parameters and setting struct

%% 1) Assign topology-dependent parameters, initialize MOTOR and GEARBOX struct
v.LDS.settings.topology=v.Input.topology;

%Check if max_speed input is lower as max cycle speed:
v=check_max_speed(v,Par);                                           

initialize_powertrain = str2func(['initialize_powertrain_', v.LDS.settings.topology]);                                            
v = initialize_powertrain(v,Par);

%Find which axles are filled: [front_axle, rear_axle] and save for later
v.LDS.settings.filled_axles = [~isempty(v.LDS.MOTOR{1}),~isempty(v.LDS.MOTOR{2})];

%Save characteristics path
v.LDS.settings.path = Par.LDS.char_path;

%% select parameters valid for all topologies

%The acceleration and cosumption sumulation run with vehicle empty weight with driver in kg
v.LDS.weights.vehicle_empty_weight_EU = v.masses.vehicle_empty_weight_EU;

%if different mass for consumption simulation is required, it is set here
if ~isnan(v.masses.vehicle_sim_cons_weight)
    v.LDS.weights.vehicle_sim_cons_weight = v.masses.vehicle_sim_cons_weight;   
else
    v.LDS.weights.vehicle_sim_cons_weight = v.masses.vehicle_empty_weight_EU;
end

%wheel parameters:
if isfield(v.dimensions.CX,'wheel_f_diameter')
    v.LDS.wheel.r_tire=v.dimensions.CX.wheel_f_diameter/2;  %static radius of the tire in mm
    v.LDS.wheel.w_tire=v.dimensions.CY.wheel_f_width;       %tire width in mm
    v.LDS.wheel.r_dyn =v.LDS.wheel.r_tire*1.02;             %dynamic radius in mm
else
    v.LDS.wheel.r_dyn  = v.Input.r_tire*1.02;               %dynamic radius in mm
    v.LDS.wheel.r_tire = v.Input.r_tire;                    %static radius of the tire in mm 
    v.LDS.wheel.w_tire = v.Input.w_tire;                    %width of tire in mm 
end
%vehicle parameters:
v=calc_c_d(v,Par);                                                                                              %Assign c_d
v=calc_c_r(v,Par);                                                                                              %Assign c_r
v.LDS.parameters.wheelbase= v.Input.wheelbase;                                                                  %wheelbase in mm
v.LDS.parameters.vehicle_height=v.Input.vehicle_height;                                                         %total height in mm
v.LDS.parameters.vehicle_width=v.Input.vehicle_width;                                                           %total width in mm
v.LDS.parameters.A = v.Input.vehicle_width *v.Input.vehicle_height*Par.LDS.correction_area;                     %cross sectional area in mm^2
v.LDS.parameters.height_COG = v.Input.wheelbase*(0.26-0.04*(v.LDS.weights.vehicle_empty_weight_EU/1000));       %cog heigth in mm

%Simulation settings:
v.LDS.settings.v_max_sim = Par.LDS.v_max_sim;                   %Required speed for testing acceleration time (standard valuze 100 Km/h)
v.LDS.settings.t_sim_max_acc   = Par.LDS.t_sim;                 %Assign the max simulation time in s for the acceleration from 0 to 100
v.LDS.settings.power_auxiliaries = v.Input.power_auxiliaries;   %Assign power auxiliaries value

%engine characteristic settings:
v.LDS.settings.characteristic_forced = v.Input.characteristic_forced;       %forced characteristic per axle, otherwise NaN
v.LDS.settings.ratio_req = v.Input.ratio_req;                               %required angular speed ratio per axle
v.LDS.settings.weight_T = v.Input.weight_T;                                 %Torque weight factor per axle
v.LDS.settings.weight_n = v.Input.weight_n;                                 %speed weight factor per axle
v.LDS.settings.weight_ratio = v.Input.weight_ratio;                         %angular speed ratio weight factor per axle

%Torque optimizer if only one engine is overloaded - on/off (1/NaN)
v.LDS.settings.torque_optimizer = Par.LDS.torque_optimizer;

%sim acc requirements:
v.LDS.sim_acc.acc_time_req = v.Input.acceleration_time_req;  %Required acceleration time in s
%The max speed was already set by the function check v_max


end

