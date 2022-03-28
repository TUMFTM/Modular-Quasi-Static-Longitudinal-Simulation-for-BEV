function [vehicle] = acceleration_sim(vehicle,Parameters)
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich), Korbinian Moller
%-------------
% Created on: 06.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: Adapts machine operating map to fulfill acceleration requirement
% ------------
% Sources: More information regarding the implementation of the LDS functions is available at:
%          [1] K. Moller, „Validierung einer MATLAB Längsdynamiksimulation für die Auslegung von Elektrofahrzeugen,“ Bachelor thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2020. 
%          [2] K. Moller, „Antriebsstrangmodellierung zur Optimierung autonomer Elektrofahrzeuge,“Semester thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2021.
% ------------
% Input: vehicle: The vehicle structure -> Stores the calculated component volumes and masses
%        Parameters: The Parameters structure -> Stores the constant values and regressions for volume and mass models
% ------------
% Output: vehicle struct with maximum required machine torque
% ------------
%% Implementation:
%1) Execute a first acceleration simulation
%2) Scale torque to realize needed acceleration time -> find needed motor characteristic

%% 1) Execute a first acceleration simulation
%Calculate e_i value
vehicle = calc_e_i(vehicle,Parameters.LDS);

%Calculate the acceleration time
vehicle = calc_acceleration(vehicle,Parameters);                                              

% calculation of current acceleration time
acc_time_is_stf = max(vehicle.LDS.sim_acc.t);  

%If acceleration time higher than setted max simulation time OR a required time is given, torque scaling gets activated
if acc_time_is_stf > vehicle.LDS.settings.t_sim_max_acc || ~isnan(vehicle.LDS.sim_acc.acc_time_req)
    
    %Assign the tolerance of acceleration time in s
    max_diff_to_acc = Parameters.LDS.max_diff_to_acc; 

    %In case the acceleration time is higher than setted max simulation time, 
    %but no required time is given the max simulation time is assigned as required time
    if isnan(vehicle.LDS.sim_acc.acc_time_req)
        vehicle.LDS.sim_acc.acc_time_req=vehicle.LDS.settings.t_sim_max_acc;
        vehicle.LDS = errorlog(vehicle.LDS,'Vehicle acceleration time is equal or above max simulation time. Motor torque will be scaled to reach max simulation time!');
    end

    %Absolute difference from required acceleration in m/s^2
    acc_delta = acc_time_is_stf - vehicle.LDS.sim_acc.acc_time_req;                              

    %Relative difference from required acceleration
    acc_delta_proz = acc_time_is_stf / vehicle.LDS.sim_acc.acc_time_req;                            

    %% 2) Scale torque to realize needed acceleration time -> find needed motor characteristic  
    %if acceleration difference is smaller or higher than tolerance motor torque has to be adapted
    while acc_delta > max_diff_to_acc || acc_delta < (-max_diff_to_acc) 

        % Traction limit is reached in at least 80% of the timesteps --> required acceleration time is too low, increase required time in input parameters
        if acc_delta >= 0 && sum(vehicle.LDS.sim_acc.a_limited == vehicle.LDS.sim_acc.traction_lim) > 0.8 * numel(vehicle.LDS.sim_acc.traction_lim)
            
            vehicle.LDS.sim_acc.traction_limit_reached_rel_timesteps = sum(vehicle.LDS.sim_acc.a_limited == vehicle.LDS.sim_acc.traction_lim) / numel(vehicle.LDS.sim_acc.traction_lim); % relative count of times where the traction limit is reached
            vehicle.LDS.sim_acc.acc_time_not_reachable = 1;
            vehicle.LDS = errorlog(vehicle.LDS,'Required acceleration time cannot be achieved because the traction limit has been reached in at least 80% of timesteps. Restart simulation with higher required acceleration time !');
            if vehicle.LDS.settings.suppress_LDS_warnings == 0
                fprintf(2,'Required acceleration time cannot be achieved because the traction limit has been reached in at least 80 percent of timesteps. Restart simulation with higher required acceleration time ! \nTry more than %.1f seconds next time! \n \n \n',vehicle.LDS.sim_acc.acc_time_is);
            end
            return;
        end
        
        %scale motor torque of front and rear motor if used with relative difference
        vehicle.LDS = rescale_characteristics(vehicle.LDS,NaN,NaN,NaN,acc_delta_proz,NaN); %(vehLDS,n_max,n_factor,T_max,T_factor,axles)

        %run acceleration simulation in order to get new acceleration
        [vehicle] = calc_acceleration(vehicle,Parameters);

        %read new acceleration time difference in m/s^2
        acc_time_is_stf = vehicle.LDS.sim_acc.acc_time_is;                                 

        %calculate new absolute and relative difference
        acc_delta = acc_time_is_stf - vehicle.LDS.sim_acc.acc_time_req;       
        acc_delta_proz = acc_time_is_stf / vehicle.LDS.sim_acc.acc_time_req; 
    end
end 
end