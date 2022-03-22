function [vehicle] = calc_power_all_wheel(vehicle, Par)
% Designed by: Lorenzo Nicoletti (FTM, Technical University of Munich), Korbinian Moller, Ruben Hefele
%-------------
% Created on: 06.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function calculates the required power at the battery for allwheel drives.
%              For this case, the optimal torque distribution at each timstep is calculated
% ------------
% Sources: More information regarding the implementation of the LDS functions is available at:
%          [1] K. Moller, „Validierung einer MATLAB Längsdynamiksimulation für die Auslegung von Elektrofahrzeugen,“ Bachelor thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2020. 
%          [2] K. Moller, „Antriebsstrangmodellierung zur Optimierung autonomer Elektrofahrzeuge,“Semester thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2021.
% ------------
% Input: veh: vehicle structure
%        Par: parameter structure
% ------------
% Output: Operating point of motors and their power, as well as the battery power
% ------------
%% Implementation
%1) Generate possible Torque matrix from the torque split factor and evaluate gearbox model --> needed mechanical engine power
%2) Calculate eta matrix for the motors
%3) Calculate resulting Power matrix and find the optimal distribution
%4) Check that the motor does not remain too long in the overload zone
%5) Evaluate Torque Distribution
%6) Assign Power Outputs

%% 1) Generate possible Torque matrix from the torque split factor and evaluate gearbox model --> needed mechanical engine power
%define torque split vector 
torquesplit=0:0.01:1;

%Rotational speed of motor over cycle in 1/min 
n_mot_f = repmat(vehicle.LDS.sim_cons.n_wheels.*vehicle.LDS.GEARBOX{1}.i_gearbox,1,numel(torquesplit));  
n_mot_r = repmat(vehicle.LDS.sim_cons.n_wheels.*vehicle.LDS.GEARBOX{2}.i_gearbox,1,numel(torquesplit)); 

% define gearbox transmission ratios
i_gearbox_f = vehicle.LDS.GEARBOX{1}.i_gearbox;
i_gearbox_r = vehicle.LDS.GEARBOX{2}.i_gearbox;

%Find eta of gearbox
eta_gear_f = vehicle.LDS.GEARBOX{1}.eta;
eta_gear_r = vehicle.LDS.GEARBOX{2}.eta;

%matrix of torque at wheels
T_wheel_f = vehicle.LDS.sim_cons.T_wheels * torquesplit;    % torque of front wheels in Nm
T_wheel_r = vehicle.LDS.sim_cons.T_wheels * (1-torquesplit);% torque of rear wheels in Nm

%matrix of power at wheels
P_wheel_f = vehicle.LDS.sim_cons.P_wheels * torquesplit;    % torque of front wheels in Nm
P_wheel_r = vehicle.LDS.sim_cons.P_wheels * (1-torquesplit);% torque of rear wheels in Nm

%matrix of torque needed for each electric machine in Nm
T_mot_f = (T_wheel_f./(vehicle.LDS.MOTOR{1}.quantity.*i_gearbox_f))./eta_gear_f.^sign(T_wheel_f(:,(round(numel(torquesplit)/2))));      % torque of front motor in Nm
T_mot_r = (T_wheel_r./(vehicle.LDS.MOTOR{2}.quantity.*i_gearbox_r))./eta_gear_r.^sign(T_wheel_r(:,(round(numel(torquesplit)/2))));      % torque of rear motor in Nm

% calculate mechanical engine power
P_mot_f = P_wheel_f ./ (vehicle.LDS.MOTOR{1}.quantity*(eta_gear_f).^sign(P_wheel_f(:,(round(numel(torquesplit)/2)))));
P_mot_r = P_wheel_r ./ (vehicle.LDS.MOTOR{2}.quantity*(eta_gear_r).^sign(P_wheel_r(:,(round(numel(torquesplit)/2)))));

%% 2) Calculate eta matrix for the motors
%find front motor eta for current operating point  
%create grid
[xd,yd] = ndgrid(vehicle.LDS.MOTOR{1}.diagram.n_scaled,vehicle.LDS.MOTOR{1}.diagram.T_scaled);

%create interpolation function
F_f = griddedInterpolant(xd,yd,vehicle.LDS.MOTOR{1}.diagram.etages','linear','none');

%find etas of front motor
eta_mot_f = F_f(abs(n_mot_f),abs(T_mot_f));

%find rear motor eta for current operating point if there's an efficiency map
%create grid
[xd,yd] = ndgrid(vehicle.LDS.MOTOR{2}.diagram.n_scaled,vehicle.LDS.MOTOR{2}.diagram.T_scaled);

%create interpolation function
F_r = griddedInterpolant(xd,yd,vehicle.LDS.MOTOR{2}.diagram.etages','linear','none');

%find etas of rear motor
eta_mot_r = F_r(abs(n_mot_r),abs(T_mot_r));

%% 3) Calculate resulting Power matrix and find the optimal distribution
%For the sign function, only one column instead of the whole matrix T or w is needed
id = round(numel(torquesplit)/2);

%Calculate electrical power of the motor for each possible torque distribution (in W)
P_el_mot_f = P_mot_f./(eta_mot_f.^(sign(P_mot_f(:,id)))) * vehicle.LDS.MOTOR{1}.quantity;
P_el_mot_r = P_mot_r./(eta_mot_r.^(sign(P_mot_r(:,id)))) * vehicle.LDS.MOTOR{2}.quantity;
P_el_mot_tot = P_el_mot_f + P_el_mot_r;

%Find the operationg point, where to reach the required torque, the resulting electrical power is the lowest
[P_el_min,I_mot] = min(P_el_mot_tot,[],2,'omitnan');                                                                                   
idx = sub2ind(size(P_el_mot_tot), (1:size(P_el_mot_tot,1))', I_mot);

%Vector of battery power in W over drive cycle
P_batt = P_el_min./(Par.LDS.eta_battery.*Par.LDS.eta_power_electronics).^(sign(P_el_min)); 

%% 4) Check that the motor does not remain too long in the overload zone
%Check overload for motor/motors on the front axle
[vehicle] = check_overload(vehicle,Par,T_mot_f(idx),n_mot_f(idx),1);
%Check overload for motor/motors on the rear axle
[vehicle] = check_overload(vehicle,Par,T_mot_r(idx),n_mot_r(idx),2);

%% 5) Evaluate Torque Distribution
[distr_trq_f, distr_trq_r, distr_trq_rel] = torque_distribution(I_mot);

%% 6) Assign Power Outputs
%Vector describing the operation points of motor/motors
vehicle.LDS.sim_cons.eta_mot_f      = eta_mot_f(idx);   %[-]
vehicle.LDS.sim_cons.eta_mot_r      = eta_mot_r(idx);   %[-]
vehicle.LDS.sim_cons.T_mot_f        = T_mot_f(idx);     %[Nm]
vehicle.LDS.sim_cons.T_mot_r        = T_mot_r(idx);     %[Nm]
vehicle.LDS.sim_cons.n_mot_f        = n_mot_f(idx);     %[1/min]
vehicle.LDS.sim_cons.n_mot_r        = n_mot_r(idx);     %[1/min]

%Gear, gear distribution and torque distribution
vehicle.LDS.sim_cons.torque_distribution           = table;                     %declaration
vehicle.LDS.sim_cons.torque_distribution_cycle     = distr_trq_rel;             %relative use of front or rear motor referred to total use in cycle
vehicle.LDS.sim_cons.torque_distribution.trq_front = distr_trq_f;               % in percent
vehicle.LDS.sim_cons.torque_distribution.trq_rear  = distr_trq_r;               % torque distribution table between front and rear axle in percent
vehicle.LDS.sim_cons.torque_distribution.trq_sum   = distr_trq_f + distr_trq_r; %in percent

%Mechanical and electrical power of each machine, as well as battery power. All powers in W:
vehicle.LDS.sim_cons.P_mech_mot_f = P_mot_f(idx); 
vehicle.LDS.sim_cons.P_mech_mot_r = P_mot_r(idx);
vehicle.LDS.sim_cons.P_el_mot_f   = P_el_mot_f./vehicle.LDS.MOTOR{1}.quantity;
vehicle.LDS.sim_cons.P_el_mot_r   = P_el_mot_r./vehicle.LDS.MOTOR{2}.quantity;
vehicle.LDS.sim_cons.P_batt       = P_batt;
end

%% Additional Function
function [distr_trq_f, distr_trq_r, distr_trq_rel] = torque_distribution(I_mot)
%distribution torque front - rear
%calculate distribution between front and rear axle
distr_trq_f = mod(I_mot,101);

%"translate" vector to vector with values from 0-100 percent
distr_trq_f(distr_trq_f == 0) = 101;
distr_trq_f = distr_trq_f - 1;

%create distribution vector for rear axle
distr_trq_r = 100-distr_trq_f;

% calculate number of total timesteps * 100% and calculate relative motor usage 
sum_step = sum(distr_trq_f + distr_trq_r); 
sum_f = sum(distr_trq_f);
sum_r = sum(distr_trq_r);

%relative usage of front/rear motor
distr_trq_rel = table;
distr_trq_rel.rel_front = sum_f/sum_step;
distr_trq_rel.rel_rear = sum_r/sum_step;
distr_trq_rel.rel_all = sum_step/sum_step;
end