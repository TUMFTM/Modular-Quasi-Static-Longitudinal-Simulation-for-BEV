function [veh] = calc_power_rear_wheel (veh, Par)
%% Description:
%this function calculates the required power at the battery for rear_wheels

%Authors: Lorenzo Nicoletti, Ruben Hefele, FTM, TUM
%date:   18.12.2019

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
%Torque of motor over cycle in Nm
%T_mot_r=veh.LDS.sim_cons.T_wheels/(veh.LDS.MOTOR{2}.quantity*veh.LDS.GEARBOX{2}.i_gearbox*veh.LDS.GEARBOX{2}.eta); %wrong
T_mot_r = veh.LDS.sim_cons.T_wheels./(veh.LDS.MOTOR{2}.quantity.*veh.LDS.GEARBOX{2}.i_gearbox.*(veh.LDS.GEARBOX{2}.eta).^sign(veh.LDS.sim_cons.T_wheels));

%Rotational speed of motor over cycle in 1/min  
n_mot_r= veh.LDS.sim_cons.n_wheels.*veh.LDS.GEARBOX{2}.i_gearbox;                                                                      

%% 2) Calculate eta matrix for the motor

%create grid
[xd,yd]=ndgrid(veh.LDS.MOTOR{2}.diagram.n_scaled,veh.LDS.MOTOR{2}.diagram.T_scaled);

%create interpolation function
F=griddedInterpolant(xd,yd,veh.LDS.MOTOR{2}.diagram.etages','linear','none');

%find current eta (efficiency of rear motor)
eta_mot_r=F(abs(n_mot_r),abs(T_mot_r));    

%force gear if required
gear_forced = NaN;

%find best gear combination
[T_mot_r_opt,n_mot_r_opt,eta_mot_r_opt,P_el_mot_r_opt, gear,gear_distr] = Det_gear(T_mot_r,n_mot_r,eta_mot_r,gear_forced,veh.LDS.GEARBOX{2}.num_gears);


%% 3) Check that the motor does not leave the allowed overload time
%Check overload for motor/motors on the front axle
[veh]=check_overload(veh,Par,T_mot_r_opt,n_mot_r_opt,2);

%% 4) Assign Outputs
%Operating point of motor/motors for each gear
veh.LDS.sim_cons.T_mot_r_gear=T_mot_r;       %[Nm]
veh.LDS.sim_cons.n_mot_r_gear=n_mot_r;       %[1/min]
veh.LDS.sim_cons.eta_mot_r_gear=eta_mot_r;   %[-]        
%Best operating point
veh.LDS.sim_cons.T_mot_r=T_mot_r_opt;       %[Nm]
veh.LDS.sim_cons.n_mot_r=n_mot_r_opt;       %[1/min]
veh.LDS.sim_cons.eta_mot_r=eta_mot_r_opt;   %[-]    


%Mechanical and electrical power of each machine, as well as battery power. All powers in W:
%P_mech
veh.LDS.sim_cons.P_mech_mot_r_gears=(veh.LDS.sim_cons.P_wheels./(veh.LDS.MOTOR{2}.quantity*(veh.LDS.GEARBOX{2}.eta).^sign(veh.LDS.sim_cons.P_wheels)));
%P_el
veh.LDS.sim_cons.P_el_mot_r_gears = veh.LDS.sim_cons.P_mech_mot_r_gears./(veh.LDS.sim_cons.eta_mot_r_gear).^sign(veh.LDS.sim_cons.P_mech_mot_r_gears);
veh.LDS.sim_cons.P_el_mot_r = P_el_mot_r_opt;
%P_batt
veh.LDS.sim_cons.P_batt_gears=(veh.LDS.sim_cons.P_el_mot_r_gears*veh.LDS.MOTOR{2}.quantity)./((Par.LDS.eta_battery.*Par.LDS.eta_power_electronics).^sign(veh.LDS.sim_cons.P_el_mot_r_gears));            %power of battery in W
veh.LDS.sim_cons.P_batt = (veh.LDS.sim_cons.P_el_mot_r*veh.LDS.MOTOR{2}.quantity)./((Par.LDS.eta_battery.*Par.LDS.eta_power_electronics).^sign(veh.LDS.sim_cons.P_el_mot_r));
%gear
veh.LDS.sim_cons.gear_r = gear;
veh.LDS.sim_cons.gear_distribution_r = gear_distr;
end