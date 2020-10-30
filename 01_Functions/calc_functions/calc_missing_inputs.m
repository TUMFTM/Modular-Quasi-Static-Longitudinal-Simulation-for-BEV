function [veh] = calc_missing_inputs(veh,Par)
%% Description:
%Aim of the function is to identifiy the case and calculate missing parameters.

%author:    Lorenzo Nicoletti, Korbinian Moller
%date:      28.05.2020
%% Inputs:
%vehicle struct: n_max, i_gear, T_max (motor), v_max, acc_time_req
%Parameters struct: needed to be sent to other function
%% Outputs:
%vehicle struct with calculated missing values
%% Implementation:
% 1) Define local variables required for the case identification
% 2) Calculate missing values

%% 1) Define local variables required for the case identification:
%axles that are filled: [front_axle, rear_axle]
filled_axles = veh.LDS.settings.filled_axles;

       
%% 2) Calculate missing values

for i=find(filled_axles)
    
    n_max=veh.LDS.MOTOR{i}.n_max;
    i_gea=veh.LDS.GEARBOX{i}.i_gearbox;
    
if ~isnan(n_max) %n_max is an input for current motor
    
    if isnan(i_gea) %gear ratio is not an input current gearbox
        %Inputs: n_max, max_speed. Wheteher acc_time or T_max or both are given. i_Gear is not given
        
        %calculate max rotational speed of wheels in 1/min
        n_max_wheel = (veh.LDS.sim_acc.max_speed/3.6)/(2*pi*veh.LDS.wheel.r_dyn*1e-3)*60;                               
        
        %calculate gear ratios the gearbox(gearboxes        
        veh.LDS.GEARBOX{i}.i_gearbox = (veh.LDS.MOTOR{i}.n_max/n_max_wheel)*Par.LDS.correction_gearbox;
        if i==1
        veh.LDS=errorlog(veh.LDS,'There is no gearbox ratio given at the front axle. It will be calculated from n_max and max_speed');
        else
        veh.LDS=errorlog(veh.LDS,'There is no gearbox ratio given at the rear axle. It will be calculated from n_max and max_speed');
        end
        
    end
    
elseif ~isnan(i_gea) %gear ratio is an input for all motors
        %Inputs: gear ratio, max_speed.  Wheteher acc_time or T_max or both are given. n_max is not given.
        
        %calculate max rotational speed of wheels in 1/min
        n_max_wheel = (veh.LDS.sim_acc.max_speed/3.6)/(2*pi*veh.LDS.wheel.r_dyn*1e-3)*60;                               
        
        %calculate max rotational speed of motors in 1/min
        veh.LDS.MOTOR{i}.n_max = max(veh.LDS.GEARBOX{i}.i_gearbox*n_max_wheel); %gear with highest gear ratio determines n_max
        if i==1
        veh.LDS=errorlog(veh.LDS,'There is no maximum rotation speed given at the front axle. It will be calculated from gear ratio and max_speed');
        else
        veh.LDS=errorlog(veh.LDS,'There is no maximum rotation speed given at the rear axle. It will be calculated from gear ratio and max_speed');
        end
    
end
    
end

end

