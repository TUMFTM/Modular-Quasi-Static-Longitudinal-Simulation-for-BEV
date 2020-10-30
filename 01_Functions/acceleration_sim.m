function [v] = acceleration_sim(v,Par)
%% Description
% adapts machine operating map to fulfill acceleration requirement

%author:    Lorenzo Nicoletti, Ruben Hefele, FTM, TUM
%date:      18.12.19
%% Input: 
% characteristic
% required acceleration time
%% Output:
% T_max
%% Sources: 
%Stephan Fuchs
%% Implementation:

%1) Execute a first acceleration simulation
%2) Scale torque to realize needed acceleration time -> find needed motor characteristic
%3) Sclae the efficiency map using the values of the calculated motor characteristic

%% 1) Execute a first acceleration simulation

%Calculate e_i value
v = calc_e_i(v,Par.LDS);

%Calculate the acceleration time
v = calc_acceleration(v,Par.LDS);                                              

% calculation of current acceleration time
acc_time_is_stf = max(v.LDS.sim_acc.t);  

%If acceleration time higher than setted max simulation time OR a required time is given, torque scaling gets activated
if acc_time_is_stf > v.LDS.settings.t_sim_max_acc || ~isnan(v.LDS.sim_acc.acc_time_req)
    
    %Assign the tolerance of acceleration time in s
    max_diff_to_acc = Par.LDS.max_diff_to_acc; 

    %In case the acceleration time is higher than setted max simulation time, 
    %but no required time is given the max simulation time is assigned as required time
    if isnan(v.LDS.sim_acc.acc_time_req)
        
        v.LDS.sim_acc.acc_time_req=v.LDS.settings.t_sim_max_acc;
        v.LDS = errorlog(v.LDS,'Vehicle acceleration time is equal or above max simulation time. Motor torque will be scaled to reach max simulation time!');

        
    end
    
    %The User has given an Input, which is unsufficient to reach req acc time or the setted max simulation time
    if ~isnan(v.Input.T_max_Mot_r) || ~isnan(v.Input.T_max_Mot_f) && abs(acc_time_is_stf - v.LDS.sim_acc.acc_time_req) > max_diff_to_acc
        
        v.LDS = errorlog(v.LDS,'The acceleration time differs too much from the specified value. Motor torque will be scaled to achieve the required acceleration time!');
        
    end
    
    
    %Absolute difference from required acceleration in m/s^2
    acc_delta = acc_time_is_stf - v.LDS.sim_acc.acc_time_req;                              

    %Relative difference from required acceleration
    acc_delta_proz = acc_time_is_stf / v.LDS.sim_acc.acc_time_req;                            

    %% 2) Scale torque to realize needed acceleration time -> find needed motor characteristic  

    %if acceleration difference is smaller or higher than tolerance motor torque has to be adapted
    while acc_delta > max_diff_to_acc || acc_delta < (-max_diff_to_acc)

        %scale motor torque of front and rear motor if used with relative difference
        v.LDS = rescale_characteristics(v.LDS,NaN,NaN,NaN,acc_delta_proz,NaN); %(vehLDS,n_max,n_factor,T_max,T_factor,axles)

        %run acceleration simulation in order to get new acceleration
        [v] = calc_acceleration(v,Par.LDS);

        %read new acceleration time difference in m/s^2
        acc_time_is_stf = v.LDS.sim_acc.acc_time_is;                                 

        %calculate new absolute and relative difference
        acc_delta = acc_time_is_stf - v.LDS.sim_acc.acc_time_req;       
        acc_delta_proz = acc_time_is_stf / v.LDS.sim_acc.acc_time_req; 

    end

end
    
end

