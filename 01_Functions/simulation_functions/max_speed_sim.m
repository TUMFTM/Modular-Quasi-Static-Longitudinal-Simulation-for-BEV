function [vehicle] = max_speed_sim(vehicle,Par)
% Designed by: Lorenzo Nicoletti (FTM, Technical University of Munich), Korbinian Moller
%-------------
% Created on: 06.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: Adapts machine operating map to fulfill v_max requirement
% ------------
% Sources: More information regarding the implementation of the LDS functions is available at:
%          [1] K. Moller, „Validierung einer MATLAB Längsdynamiksimulation für die Auslegung von Elektrofahrzeugen,“ Bachelor thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2020. 
%          [2] K. Moller, „Antriebsstrangmodellierung zur Optimierung autonomer Elektrofahrzeuge,“Semester thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2021.
% ------------
% Input: vehicle: The vehicle structure -> Stores the calculated component volumes and masses
%        Par: The Parameters structure -> Stores the constant values and regressions for volume and mass models
% ------------
% Output: vehicle struct with updated machine characteristics
% ------------
%% Implementation:
%1) Initialize needed inputs
%2) Calculate actual max speed, save original max speed:
%3) Scale engine to reach max speed in lowest gear combination and update acceleration time

%% 1) Initialize needed Inputs:
%Required maximum speed for max speed simulation
vehicle.LDS.sim_speed.max_speed_req = vehicle.Input.max_speed;

%% 2) Calculate actual max speed, save original max speed:
vehicle.LDS.sim_speed.max_speed_is = calc_max_speed(vehicle,Par);

%save unscaled speed
vehicle.LDS.sim_speed.max_speed_unscaled = vehicle.LDS.sim_speed.max_speed_is;

%% 3) Scale engine to reach desired max speed in lowest gear combination
%If actual max speed is lower than required max speed, torque scaling gets activated
if round(vehicle.LDS.sim_speed.max_speed_req,2) > round(vehicle.LDS.sim_speed.max_speed_is,2)
    
    %The User has given an Input Torque, which is unsufficient to reach the required max speed
    if ~isnan(vehicle.Input.T_max_Mot_r) || ~isnan(vehicle.Input.T_max_Mot_f)
       
        vehicle.LDS = errorlog(vehicle.LDS,'Vehicle maximum speed is lower than required maximum speed. The given torque Input is too low and will be scaled'); 
    end
   
    %Assign the tolerance to max required speed in km/h
    max_diff = Par.LDS.max_diff_to_max_speed;
   
    %Absolute difference from required max speed in km/h
    v_delta = vehicle.LDS.sim_speed.max_speed_req - vehicle.LDS.sim_speed.max_speed_is;
   
    %Relative difference from required max speed in km/h
    v_delta_rel = vehicle.LDS.sim_speed.max_speed_req / vehicle.LDS.sim_speed.max_speed_is;
   
    %if speed difference is higher than tolerance motor torque has to be adapted
    while v_delta > max_diff
        
        %scale motor torque and rotational speed of front and rear motor if used with relative difference
        vehicle.LDS = rescale_characteristics(vehicle.LDS,NaN,v_delta_rel,NaN,v_delta_rel,NaN); %(vehLDS,n_max,n_factor,T_max,T_factor,axles)
        
        %recalculate actual maximum speed 
        vehicle.LDS.sim_speed.max_speed_is = calc_max_speed(vehicle,Par);
        
        %calculate new absolute and relative speed difference 
        v_delta = vehicle.LDS.sim_speed.max_speed_req - vehicle.LDS.sim_speed.max_speed_is; 
        v_delta_rel = vehicle.LDS.sim_speed.max_speed_req / vehicle.LDS.sim_speed.max_speed_is;         
   end
   
    %Recalculate updated acceleration time
    vehicle = calc_acceleration(vehicle,Par);
  
end
end