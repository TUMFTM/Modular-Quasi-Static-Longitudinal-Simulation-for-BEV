function [veh] = calc_power_all_wheel(veh, Par)
%% Description:
%this function calculates the required power at the battery for allwheel drives.
% For this case, the optimal torque distribution at each timstep is calculated

%Authors: Lorenzo Nicoletti, Ruben Hefele, Korbinian Moller, FTM, TUM
%date:   18.08.2020

%% Inputs:
%P_wheel - power at wheels in W
%vehicle struct
%parameters struct
%% Outputs:
%Operating point of motors and their power, as well as the battery power
%% Sources:
%Christian Angerer 2018 - consumption function
%% Implementation
%1) Generate possible Torque matrix from the torque split factor
%2) Calculate eta matrix for the motors
%3) Calculate resulting Power matrix and find the optimal distribution
%4) Check that the motor does not remain too long in the overload zone
%5) Evaluate gear combination
%6) Assign Power Outputs

%% 1) Generate possible Torque matrix from the torque split factor
num_gears_f = veh.LDS.GEARBOX{1}.num_gears;
num_gears_r = veh.LDS.GEARBOX{2}.num_gears;
num_comb = num_gears_f * num_gears_r;

%define torque split vector and preallocate cells
torquesplit=0:0.01:1; 
F_drive_f = cell(1,num_comb);
F_drive_r = cell(1,num_comb);

%matrix of drive force in N for front and rear axle for every possible gear combination
for i = 1:num_comb
    F_drive_f{i} = veh.LDS.sim_cons.resistance.F_tot(:,i)*torquesplit;   
    F_drive_r{i} = veh.LDS.sim_cons.resistance.F_tot(:,i)*(1-torquesplit);
end                                                                                                                                                           
F_drive_f = cell2mat(F_drive_f);
F_drive_r = cell2mat(F_drive_r);

%matrix angular velocity at the wheels in rad/s 
w_wheels=((veh.LDS.sim_cons.v*ones(1, numel(torquesplit)))./(veh.LDS.wheel.r_dyn/1000));                                                                                                                                                                                                       
 
%matrix of torque at wheels
T_wheel_f=F_drive_f.*(veh.LDS.wheel.r_dyn/1000);  % torque of front wheels in Nm for every possible gear combination
T_wheel_r=F_drive_r.*(veh.LDS.wheel.r_dyn/1000);  % torque of rear wheels in Nm for every possible gear combination

%matrix of gearbox ratio and efficiency front motor
i_gearbox_mat_f = cell(1,num_gears_f);
eta_gearbox_mat_f = cell(1,num_gears_f);
for i = 1:num_gears_f
    i_gearbox_mat_f{i} = repmat((ones(size(w_wheels)) * veh.LDS.GEARBOX{1}.i_gearbox(i)),1,num_gears_r);
    eta_gearbox_mat_f{i} = repmat((ones(size(w_wheels)) * veh.LDS.GEARBOX{1}.eta(i)),1,num_gears_r);
end
i_gearbox_mat_f = cell2mat(i_gearbox_mat_f);
eta_gearbox_mat_f = cell2mat(eta_gearbox_mat_f);

%matrix of gearbox ratio and efficiency rear motor
i_gearbox_mat_r = cell(1,num_gears_r);
eta_gearbox_mat_r = cell(1,num_gears_r);
for i = 1:num_gears_r
    i_gearbox_mat_r{i} = ones(size(w_wheels)) * veh.LDS.GEARBOX{2}.i_gearbox(i);
    eta_gearbox_mat_r{i} = ones(size(w_wheels)) * veh.LDS.GEARBOX{2}.eta(i);
end
%i_gearbox_rear convert to matrix and repmat
i_gearbox_mat_r = cell2mat(i_gearbox_mat_r);
i_gearbox_mat_r = repmat(i_gearbox_mat_r,1,num_gears_f);
%eta_gearbox_rear convert to matrix and repmat
eta_gearbox_mat_r = cell2mat(eta_gearbox_mat_r);
eta_gearbox_mat_r = repmat(eta_gearbox_mat_r,1,num_gears_f);

%matrix of angular velocity of electric machine
w_mot_f = repmat(w_wheels,1,num_comb) .* i_gearbox_mat_f; % rotational speed of front motor in rad/s for gear combination matrix at front axle
w_mot_r = repmat(w_wheels,1,num_comb) .* i_gearbox_mat_r; % rotational speed of front motor in rad/s for gear combination matrix at rear axle
    
%matrix of torque needed for each electric machine in Nm
T_mot_f=(T_wheel_f./(veh.LDS.MOTOR{1}.quantity.*i_gearbox_mat_f))./eta_gearbox_mat_f.^sign(T_wheel_f(:,(round(numel(torquesplit)/2))));      % torque of front motor in Nm
T_mot_r=(T_wheel_r./(veh.LDS.MOTOR{2}.quantity.*i_gearbox_mat_r))./eta_gearbox_mat_r.^sign(T_wheel_r(:,(round(numel(torquesplit)/2))));      % torque of rear motor in Nm

%% 2) Calculate eta matrix for the motors

%find front motor eta for current operating point  
%create grid
[xd,yd]=ndgrid(veh.LDS.MOTOR{1}.diagram.n_scaled,veh.LDS.MOTOR{1}.diagram.T_scaled);

%create interpolation function
F=griddedInterpolant(xd,yd,veh.LDS.MOTOR{1}.diagram.etages','linear','none');

%find current eta
eta_mot_f=F(abs(w_mot_f)*60/(2*pi),abs(T_mot_f));                                                            % efficiency of front motor

%find rear motor eta for current operating point if there's an efficiency map
%create grid
[xd,yd]=ndgrid(veh.LDS.MOTOR{2}.diagram.n_scaled,veh.LDS.MOTOR{2}.diagram.T_scaled);

%create interpolation function
F=griddedInterpolant(xd,yd,veh.LDS.MOTOR{2}.diagram.etages','linear','none');

%find current eta
eta_mot_r=F(abs(w_mot_r)*60/(2*pi),abs(T_mot_r));                                                            % efficiency of rear motor

%% 3) Calculate resulting Power matrix and find the optimal distribution

%For the sign function, only one column instead of the whole matrix T or w is needed
id=round(numel(torquesplit)/2);

%Calculate mechanical power of the motor for each possible torque distribution (in W)
P_mech_mot_f=veh.LDS.MOTOR{1}.quantity*T_mot_f.*w_mot_f;
P_mech_mot_r=veh.LDS.MOTOR{2}.quantity*T_mot_r.*w_mot_r;

%Calculate electrical power of the motor for each possible torque distribution (in W)
P_el_mot_f=P_mech_mot_f./eta_mot_f.^(sign(P_mech_mot_f(:,id)));
P_el_mot_r=P_mech_mot_r./eta_mot_r.^(sign(P_mech_mot_r(:,id)));
P_el_mot_tot=P_el_mot_f+P_el_mot_r;


%Find the operationg point, where to reach the required torque, the resulting electrical power is the lowest
[P_el_min,I_mot]=nanmin(P_el_mot_tot,[],2);                                                                                   
idx = sub2ind(size(P_el_mot_tot), (1:size(P_el_mot_tot,1))', I_mot);

%Vector of battery power in W over drive cycle
P_batt=P_el_min./(Par.LDS.eta_battery.*Par.LDS.eta_power_electronics).^(sign(P_el_min)); 

%% 4) Check that the motor does not remain too long in the overload zone

%Check overload for motor/motors on the front axle
[veh]=check_overload(veh,Par,T_mot_f(idx),w_mot_f(idx)*30/pi,1);
%Check overload for motor/motors on the rear axle
[veh]=check_overload(veh,Par,T_mot_r(idx),w_mot_r(idx)*30/pi,2);

%% 5) Evaluate gear combination
% evaluate I_mot vector to get used gear at each timestep and torque distribution between front and rear axle
[gear_f, gear_r, distr_f, distr_r, distr_trq_f, distr_trq_r, distr_trq_rel] = eval_gear_all(num_gears_f,num_gears_r, num_comb, I_mot);

%% 6) Assign Power Outputs
%Vector describing the operation points of motor/motors
veh.LDS.sim_cons.eta_mot_f      = eta_mot_f(idx);   %[-]
veh.LDS.sim_cons.eta_mot_r      = eta_mot_r(idx);   %[-]
veh.LDS.sim_cons.T_mot_f        = T_mot_f(idx);     %[Nm]
veh.LDS.sim_cons.T_mot_r        = T_mot_r(idx);     %[Nm]
veh.LDS.sim_cons.n_mot_f        = w_mot_f(idx)*30/pi; %[1/min]
veh.LDS.sim_cons.n_mot_r        = w_mot_r(idx)*30/pi; %[1/min]

%Gear, gear distribution and torque distribution
veh.LDS.sim_cons.gear = table;                                              %declaration
veh.LDS.sim_cons.torque_distribution = table;                               %declaration
veh.LDS.sim_cons.torque_distribution_cycle = distr_trq_rel;                 %relative use of front or rear motor referred to total use in cycle
veh.LDS.sim_cons.gear.gear_front = gear_f;                                  %vector with used gears at front axle
veh.LDS.sim_cons.gear.gear_rear = gear_r;                                   %vector with used gears at rear axle
veh.LDS.sim_cons.gear_distribution_f = distr_f;                             %gear distribution at front axle (first row: absolute number, second row: relative number, column 1-#gear_front: gears, last column: sum)
veh.LDS.sim_cons.gear_distribution_r = distr_r;                             %gear distribution at rear axle (first row: absolute number, second row: relative number, column 1-#gear_rear: gears, last column: sum)
veh.LDS.sim_cons.torque_distribution.trq_front = distr_trq_f;               %
veh.LDS.sim_cons.torque_distribution.trq_rear = distr_trq_r;                % torque distribution table between front and rear axle in percent
veh.LDS.sim_cons.torque_distribution.trq_sum = distr_trq_f + distr_trq_r;   %


%Mechanical and electrical power of each machine, as well as battery power. All powers in W:
veh.LDS.sim_cons.P_mech_mot_f = P_mech_mot_f(idx)./veh.LDS.MOTOR{1}.quantity; 
veh.LDS.sim_cons.P_mech_mot_r = P_mech_mot_r(idx)./veh.LDS.MOTOR{2}.quantity;
veh.LDS.sim_cons.P_el_mot_f   = P_el_mot_f./veh.LDS.MOTOR{1}.quantity;
veh.LDS.sim_cons.P_el_mot_r   = P_el_mot_r./veh.LDS.MOTOR{2}.quantity;
veh.LDS.sim_cons.P_batt       = P_batt;

end