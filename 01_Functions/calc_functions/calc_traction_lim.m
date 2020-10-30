function [traction_limit] = calc_traction_lim(vehLDS, ParLDS, F_L , alpha)
%% Description:
%this function determines the max. acceleration due to the traction limit

% Author:   Lorenzo Nicoletti,Ruben Hefele, FTM, TUM
% Date:     26.12.19
%% Inputs:
%   vehLDS:    struct with the vehicle parameters
%   ParLDS:    struct with fix parameters
%   F_L:       air resistance
%   alpha:     slope angle
%% Outputs:
%  traction_limit: vector containing the max. acceleration at each timestep of the acceleration sim
%% Sources: 
%Haken - Grundlagen der Kraftfahrzeugtechnik
%% Implementation

%Initialize variables to make the equation shorter:
m=vehLDS.weights.vehicle_empty_weight_EU;   %Vehicle mass in kg (empty weight + driver)
g=ParLDS.g;                                 %gravitational acceleration in m/s^2
mu=ParLDS.mue_max;                          %driving traction coefficient
h_COG=vehLDS.parameters.height_COG;         %height COG in mm
wb=vehLDS.parameters.wheelbase;             %wheelbase in mm
c_r=vehLDS.parameters.c_r;                  %roll resistance coefficient
load_f=ParLDS.axle_load_front;               %load distribution front in %
load_r=ParLDS.axle_load_rear;                %load distribution rear in %

% calculate traction limit for the different drive types:
if strcmp(vehLDS.settings.drive,'rear_wheel')
    
    traction_limit = (g*(mu*(load_f*cosd(alpha)+(h_COG/wb)*sind(alpha))-c_r*(load_r*cosd(alpha)-(h_COG/wb)*sind(alpha))-sind(alpha))-(F_L/m))/1-(h_COG/wb)*(mu+c_r); 

elseif strcmp(vehLDS.settings.drive,'front_wheel')
    
    traction_limit = (g*(mu*(load_r*cosd(alpha)-(h_COG/wb)*sind(alpha))-c_r*(load_f*cosd(alpha)+(h_COG/wb)*sind(alpha))-sind(alpha))-(F_L/m))/1+(h_COG/wb)*(mu+c_r);

elseif strcmp(vehLDS.settings.drive,'all_wheel')
    
    traction_limit = g*(mu*cosd(alpha)-sind(alpha))-(F_L/m); 

end

