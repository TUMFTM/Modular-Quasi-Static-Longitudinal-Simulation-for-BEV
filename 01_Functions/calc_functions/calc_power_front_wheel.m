function [veh] = calc_power_front_wheel(veh, Par)
%% Description:
%this function calculates the required power at the battery for front wheel drives

%Authors: Lorenzo Nicoletti, Ruben Hefele, FTM, TUM
%date:   09.08.2020

%% Inputs:
%P_wheel - power at wheels in W
%vehicle struct
%parameters struct
%% Outputs:
%Operating point of motors and their power, as well as the battery power
%% Sources:
%Christian Angerer 2018 - consumption function
%% Implementation
%1) Calculate torque
%2) Calculate eta matrix for the motor
%3) Check that the motor does not leave the allowed overload time
%4) Assign Outputs

%% 1) Calculate torque
%calculate current operating point torque of motor over cycle in Nm
%T_mot_f = veh.LDS.sim_cons.T_wheels./(veh.LDS.MOTOR{1}.quantity.*veh.LDS.GEARBOX{1}.i_gearbox.*veh.LDS.GEARBOX{1}.eta); wrong
T_mot_f = veh.LDS.sim_cons.T_wheels./(veh.LDS.MOTOR{1}.quantity.*veh.LDS.GEARBOX{1}.i_gearbox.*(veh.LDS.GEARBOX{1}.eta).^sign(veh.LDS.sim_cons.T_wheels));

%Rotational speed of motor over cycle in 1/min 
n_mot_f = veh.LDS.sim_cons.n_wheels.*veh.LDS.GEARBOX{1}.i_gearbox;                          

%% 2) Calculate eta matrix for the motor
   
%create grid
[xd,yd]=ndgrid(veh.LDS.MOTOR{1}.diagram.n_scaled,veh.LDS.MOTOR{1}.diagram.T_scaled);

%create interpolation function
F=griddedInterpolant(xd,yd,veh.LDS.MOTOR{1}.diagram.etages','linear','none');

%find current eta (efficiency of front motor)
eta_mot_f=F(abs(n_mot_f),abs(T_mot_f));

%force gear if required
gear_forced = NaN;

%find best gear combination
[T_mot_f_opt,n_mot_f_opt,eta_mot_f_opt,P_el_mot_f_opt, gear,gear_distr] = Det_gear(T_mot_f,n_mot_f,eta_mot_f,gear_forced,veh.LDS.GEARBOX{1}.num_gears);


%% 3) Check that the motor does not leave the allowed overload time
%Check overload for motor/motors on the front axle
[veh]=check_overload(veh,Par,T_mot_f_opt,n_mot_f_opt,1);

%% 4) Assign Outputs
%Operating point of motor/motors for each gear
veh.LDS.sim_cons.T_mot_f_gear=T_mot_f;       %[Nm]
veh.LDS.sim_cons.n_mot_f_gear=n_mot_f;       %[1/min]
veh.LDS.sim_cons.eta_mot_f_gear=eta_mot_f;   %[-]        
%Best operating point
veh.LDS.sim_cons.T_mot_f=T_mot_f_opt;       %[Nm]
veh.LDS.sim_cons.n_mot_f=n_mot_f_opt;       %[1/min]
veh.LDS.sim_cons.eta_mot_f=eta_mot_f_opt;   %[-]    

%Mechanical and electrical power of each machine, as well as battery power. All powers in W:
%P_mech
veh.LDS.sim_cons.P_mech_mot_f_gears=(veh.LDS.sim_cons.P_wheels./(veh.LDS.MOTOR{1}.quantity*(veh.LDS.GEARBOX{1}.eta).^sign(veh.LDS.sim_cons.P_wheels)));
%P_el
veh.LDS.sim_cons.P_el_mot_f_gears = veh.LDS.sim_cons.P_mech_mot_f_gears./(veh.LDS.sim_cons.eta_mot_f_gear).^sign(veh.LDS.sim_cons.P_mech_mot_f_gears);
veh.LDS.sim_cons.P_el_mot_f = P_el_mot_f_opt;
%P_batt
veh.LDS.sim_cons.P_batt_gears=(veh.LDS.sim_cons.P_el_mot_f_gears*veh.LDS.MOTOR{1}.quantity)./((Par.LDS.eta_battery.*Par.LDS.eta_power_electronics).^sign(veh.LDS.sim_cons.P_el_mot_f_gears));            %power of battery in W
veh.LDS.sim_cons.P_batt = (veh.LDS.sim_cons.P_el_mot_f*veh.LDS.MOTOR{1}.quantity)./((Par.LDS.eta_battery.*Par.LDS.eta_power_electronics).^sign(veh.LDS.sim_cons.P_el_mot_f));
%gear
veh.LDS.sim_cons.gear_f = gear;
veh.LDS.sim_cons.gear_distribution_f = gear_distr;
end