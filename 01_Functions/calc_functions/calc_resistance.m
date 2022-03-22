function [resistance, T_wheels, n_wheel]=calc_resistance(v,a,alpha, vehLDS, ParLDS)
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich), Korbinian Moller
%-------------
% Created on: 06.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function calculates driving resistances, rotational speed and torque for driving cycle
% ------------
% Sources: More information regarding the implementation of the LDS functions is available at:
%          [1] K. Moller, „Validierung einer MATLAB Längsdynamiksimulation für die Auslegung von Elektrofahrzeugen,“ Bachelor thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2020. 
%          [2] K. Moller, „Antriebsstrangmodellierung zur Optimierung autonomer Elektrofahrzeuge,“Semester thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2021.
% ------------
% Input: vehicle: The vehicle structure -> Stores the calculated component volumes and masses
%        Par: The Parameters structure -> Stores the constant values and regressions for volume and mass models
%        v: velocity m/s
%        a: acceleration m/s^2
%        alpha:slope
% ------------
% Output: torque at wheels (total torque, i.e. sum of needed torque at front and rear axles)
%         and at wheels (for simplicity the speed at rear and front wheels is taken as identical)
%         different resistance components
% ------------
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