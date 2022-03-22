function [traction_limit] = calc_traction_lim(vehicle, Par, F_L , alpha)
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich), Korbinian Moller
%-------------
% Created on: 06.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function determines the max. acceleration due to the traction limit
% ------------
% Sources: More information regarding the implementation of the LDS functions is available at:
%          [1] R. Hefele, „Implementierung einer MATLAB Längsdynamiksimulation für Elektrofahrzeuge,“Semester thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2019.
%          [2] L. v. Hacken, "Grundlagen der Kraftfahrzeugtechnik", ISBN: 9783446426047, 2011
% ------------
% Input: vehicle: The vehicle structure -> Stores the calculated component volumes and masses
%        Par: The Parameters structure -> Stores the constant values and regressions for volume and mass models
%        F_L: Air resistance
%        alpha: Slope angle
% ------------
% Output: traction_limit: vector containing the max. acceleration at each timestep of the acceleration sim
% ------------
%% Implementation
%Initialize variables to make the equation shorter:
m     = vehicle.weights.vehicle_empty_weight_EU; %Vehicle mass in kg (empty weight + driver)
g     = Par.LDS.g;                               %gravitational acceleration in m/s^2
mu    = Par.LDS.mue_max;                         %driving traction coefficient
h_COG = vehicle.parameters.height_COG;           %height COG in mm
wb    = vehicle.parameters.wheelbase;            %wheelbase in mm
c_r   = vehicle.parameters.c_r;                  %roll resistance coefficient

%Calculate traction limit for the different drive types. Fromula for traction_limit taken from [2]
if strcmp(vehicle.settings.drive,'rear_wheel')
    load_f = Par.masses.loads.axle_load_front.RWD / 100;
    load_r = 1 - load_f;
    traction_limit = (g*(mu*(load_f*cosd(alpha)+(h_COG/wb)*sind(alpha))-c_r*(load_r*cosd(alpha)-(h_COG/wb)*sind(alpha))-sind(alpha))-(F_L/m))/(1-(h_COG/wb)*(mu+c_r)); 

elseif strcmp(vehicle.settings.drive,'front_wheel')
    load_f = Par.masses.loads.axle_load_front.FWD / 100;
    load_r = 1 - load_f;
    traction_limit = (g*(mu*(load_r*cosd(alpha)-(h_COG/wb)*sind(alpha))-c_r*(load_f*cosd(alpha)+(h_COG/wb)*sind(alpha))-sind(alpha))-(F_L/m))/(1+(h_COG/wb)*(mu+c_r));

elseif strcmp(vehicle.settings.drive,'all_wheel')
    traction_limit = g*(mu*cosd(alpha)-sind(alpha))-(F_L/m); 
end