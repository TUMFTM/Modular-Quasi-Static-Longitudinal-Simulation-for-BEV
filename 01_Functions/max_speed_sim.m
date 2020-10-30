function [veh] = max_speed_sim(veh,Par)
%% Description
% adapts machine operating map to fulfill v_max requirement

%author:    Lorenzo Nicoletti, Adrian König, Korbinian Moller, FTM, TUM
%date:      26.05.2020
%% Input: 
%   v:    struct with the vehicle parameters
%   Par:  struct with fixed parameters
%% Output:
%   v:    updated struct with the vehicle parameters
%% Sources: 
%   --
%% Implementation:

%1) Initialize needed inputs
%2) Calculate actual max speed, save original max speed:
%3) Scale engine to reach max speed in lowest gear combination
%4) Calculate max speed in highest gear (-> lowest i_gear) combination 

%% 1) Initialize needed Inputs:

%Required maximum speed for max speed simulation
veh.LDS.sim_speed.max_speed_req = veh.Input.max_speed;

%declare variable marker to check if torque has been scaled

%Find which axles are filled: [front_axle, rear_axle]
filled_axles = veh.LDS.settings.filled_axles;

%% 2) Calculate actual max speed, save original max speed:

veh.LDS.sim_speed.max_speed_is = calc_max_speed(veh,Par,[1,1]);

%save unscaled speed
veh.LDS.sim_speed.max_speed_unscaled = veh.LDS.sim_speed.max_speed_is;


%% 3) Scale engine to reach desired max speed in lowest gear combination

%If actual max speed is lower than required max speed, torque scaling gets activated
if round(veh.LDS.sim_speed.max_speed_req,2) > round(veh.LDS.sim_speed.max_speed_is,2)
    
   %The User has given an Input Torque, which is unsufficient to reach the required max speed
   if ~isnan(veh.Input.T_max_Mot_r) || ~isnan(veh.Input.T_max_Mot_f)
       
        veh.LDS = errorlog(veh.LDS,'Vehicle maximum speed is lower than required maximum speed. The given torque Input is too low and will be scaled');
   
   end

   
   %Assign the tolerance to max required speed in km/h
   max_diff = Par.LDS.max_diff_to_max_speed;
   
   %Absolute difference from required max speed in km/h
   v_delta = veh.LDS.sim_speed.max_speed_req - veh.LDS.sim_speed.max_speed_is;
   
   %Relative difference from required max speed in km/h
   v_delta_rel = veh.LDS.sim_speed.max_speed_req / veh.LDS.sim_speed.max_speed_is;
   
   %if speed difference is higher than tolerance motor torque has to be adapted
   while v_delta > max_diff
        
        %scale motor torque of front and rear motor if used with relative difference
        veh.LDS = rescale_characteristics(veh.LDS,NaN,NaN,NaN,v_delta_rel,NaN); %(vehLDS,n_max,n_factor,T_max,T_factor,axles)
        
        %recalculate actual maximum speed 
        veh=calc_max_speed(veh,Par);
        
        %calculate new absolute and relative speed difference 
        v_delta = veh.LDS.sim_speed.max_speed_req - veh.LDS.sim_speed.max_speed_is; 
        v_delta_rel = veh.LDS.sim_speed.max_speed_req / veh.LDS.sim_speed.max_speed_is;
           
   end
  
end

%% 4) Calculate max speed in highest gear (-> lowest i_gear) combination 

gear_max = [NaN,NaN];
gear_max_i = [NaN,NaN];

for i = find(filled_axles)
    gear_max(i) = find((veh.LDS.GEARBOX{i}.i_gearbox) == min(veh.LDS.GEARBOX{i}.i_gearbox));
    gear_max_i(i) = veh.LDS.GEARBOX{i}.i_gearbox(gear_max(i));
end

if gear_max(1) > 1 || gear_max(2) > 1
    veh.LDS.sim_speed.v_gear_max = calc_max_speed(veh,Par,gear_max); 
    veh.LDS.sim_speed.gears = gear_max;
    veh.LDS.sim_speed.gears_i = gear_max_i;   
end

end

