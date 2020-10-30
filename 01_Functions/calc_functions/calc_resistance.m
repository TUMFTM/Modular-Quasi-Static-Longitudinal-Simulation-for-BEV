function [resistance, T_wheels, n_wheel]=calc_resistance(v,a,alpha, vehLDS, ParLDS)
%% Description:
% This function calculates driving resistances, rotational speed and torque for driving cycle

% Author:    Lorenzo Nicoletti, Ruben Hefele, FTM, TUM
% date:      14.07.20
%% Inputs:
%v - velocity m/s
%a - acceleration m/s^2
%alpha - slope
%struct with vehicle data
%struct with LDS Parameters
%% Outputs:
%torque at wheels (total torque, i.e. sum of needed torque at front and rear axles)
%n at wheels (for simplicity the speed at rear and front wheels is taken as identical)
%different resistance components
%% Sources: 
% Koch - Krapf
%% Implementation:

% Calculation of resistance - all forces in N (fundamental equation of longitudinal dynamics)
F_a         = 0.5 * ParLDS.rho_L * vehLDS.parameters.c_d * vehLDS.parameters.A/(1000^2) * (v).^2;         %air resistance in N
F_r         = vehLDS.weights.vehicle_sim_cons_weight * ParLDS.g * vehLDS.parameters.c_r * cos(alpha);     %roll resistance in N
F_g         = vehLDS.weights.vehicle_sim_cons_weight * ParLDS.g * sin(alpha);                             %slope resistance in N
F_m         = vehLDS.weights.vehicle_sim_cons_weight .* vehLDS.parameters.e_i  .* a;                        %acceleration resistance in N

%If the speed or the acceleration is 0, i. e. vehicle is not moving, F_r is set to 0
F_r(find(v<1e-2))=0;

%Calculate total resistance in N
F_tot     = F_a + F_r + F_g + F_m;    
%data = readtable('/Users/Korbinian/lds_paumani/03_LDS_Hefele/04_Evaluation/02_Moller/01_D3_Test_Data/2015_VW_Egolf/61511028 Test Data.txt'); %egolf wltp
%data = readtable('/Users/Korbinian/lds_paumani/03_LDS_Hefele/04_Evaluation/02_Moller/01_D3_Test_Data/2014_BMW_i3-BEV/61505021 Test Data.txt'); %i3 
%data = data(101:18101,:); %egolf
%data = data(101:41480,:); %i3
%F_tot = data.Dyno_Tractive_Effort_N_ + F_r;
%F_tot = F_tot + abs(F_tot(1)) * ones(length(F_tot),1);

%% Assign Output

%Torque at the wheels in Nm
T_wheels     = F_tot.*vehLDS.wheel.r_dyn/1000;  

%Rotational speed of wheels in 1/min
n_wheel      = v./(vehLDS.wheel.r_dyn/1000).*60/2/pi;                            

%store resistance values in resistance struct
resistance.F_tot    =   F_tot;              %total resistance in N
resistance.F_a      =   F_a;                %air resistance in N 
resistance.F_r      =   F_r;                %roll resistance in N
resistance.F_g      =   F_g;                %slope resistance in N
resistance.F_m      =   F_m;                %acceleration resistance in N

end