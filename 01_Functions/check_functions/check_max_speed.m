function veh = check_max_speed(veh,Par)
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich), Korbinian Moller
%-------------
% Created on: 06.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: Depending on the inputs, also using v_max, the n_max of the motor and the
%              required gear ratio are calculated. The v_max is an Input, which is always
%              needed, and determines gear ration and n_max (if not given).
%              This function makes sure that the given v_max is above the max speed of
%              the choosen cycle.
% ------------
% Input: veh: The vehicle structure -> Stores the calculated component volumes and masses
%        Par: The Parameters structure -> Stores the constant values and regressions for volume and mass models
% ------------
% Output: Corrected vehicle max speed (if required)
% ------------
%% Implementation: 
%Calculate the v_max for the cycle in km/h
v_max_cycle = max(veh.LDS.sim_cons.v) * 3.6;

if isnan(veh.Input.max_speed)
    
    veh.LDS = errorlog( veh.LDS, 'Error! There is no given input speed. It will be overwritten with maximal cycle speed' );
    veh.LDS.sim_acc.max_speed = v_max_cycle;

elseif v_max_cycle>veh.Input.max_speed
    
    %Cycle speed is bigger than input max speed. The input max speed will be overwritten
    veh.LDS.sim_acc.max_speed = v_max_cycle;
    veh.LDS = errorlog( veh.LDS, 'Error! The given input speed is lower as the cycle maximum speed and will be overwritten' );    
else
    
    %The Input is bigger than max cycle speed and can be assigned for further calculations of gear ratio and n_max
    veh.LDS.sim_acc.max_speed=veh.Input.max_speed;    
end
end