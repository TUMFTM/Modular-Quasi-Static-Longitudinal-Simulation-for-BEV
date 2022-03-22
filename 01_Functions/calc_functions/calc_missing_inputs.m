function [vehicle] = calc_missing_inputs(vehicle,Par)
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich), Korbinian Moller
%-------------
% Created on: 06.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: Aim of the function is to identifiy the case and calculate missing parameters.
% ------------
% Sources: More information regarding the implementation of the LDS functions is available at:
%          [1] K. Moller, „Validierung einer MATLAB Längsdynamiksimulation für die Auslegung von Elektrofahrzeugen,“ Bachelor thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2020. 
%          [2] K. Moller, „Antriebsstrangmodellierung zur Optimierung autonomer Elektrofahrzeuge,“Semester thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2021.
% ------------
% Input: vehicle: The vehicle structure -> Stores the calculated component volumes and masses
%        Par: The Parameters structure -> Stores the constant values and regressions for volume and mass models
% ------------
% Output: vehicle struct with calculated missing values
% ------------
%% Implementation:
% 1) Define local variables required for the case identification
% 2) Calculate missing values

%% 1) Define local variables required for the case identification:
%axles that are filled: [front_axle, rear_axle]
filled_axles       = vehicle.LDS.settings.filled_axles; %Used to see which axes are "filled" with machines
correction_gearbox = Par.LDS.correction_gearbox;        %Correction factor for the gearbox -> see [1]
       
%% 2) Calculate missing values
% load/calculate resistance coefficents

% load c_d
if ~isnan(vehicle.Input.c_d) && ~isempty(vehicle.Input.c_d)
    
    vehicle.LDS.parameters.c_d=vehicle.Input.c_d;
 
else %if c_d is no input, the value from Fixparameters will be chosen
    
    vehicle.LDS.parameters.c_d=Par.LDS.c_d;
end

% call function to load/calc c_r
vehicle = calc_c_r(vehicle,Par); 

for i=find(filled_axles)

    i_gea=vehicle.LDS.GEARBOX{i}.i_gearbox;
       
    if ~isnan(i_gea) %gear ratio is an input for all motors
            %Inputs: gear ratio, max_speed.  Wheteher acc_time or T_max or both are given. n_max is not given.

            %calculate max rotational speed of wheels in 1/min
            n_max_wheel = (vehicle.LDS.sim_acc.max_speed/3.6)/(2*pi*vehicle.LDS.wheel.r_dyn*1e-3)*60;                               

            %calculate max rotational speed of motors in 1/min
            vehicle.LDS.MOTOR{i}.n_max = max(vehicle.LDS.GEARBOX{i}.i_gearbox*n_max_wheel)*((correction_gearbox)^-1); %gear with highest gear ratio determines n_max
    else
        vehicle = errorlog(vehicle,'ERROR: YOU HAVE TO ASSIGN A TRANSMISSION RATIO',1);
        return
    end    
end
end