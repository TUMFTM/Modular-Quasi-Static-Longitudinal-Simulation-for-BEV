function [vehicle] = calc_power_front_wheel(vehicle, Par)
% Designed by: Lorenzo Nicoletti (FTM, Technical University of Munich), Korbinian Moller, Ruben Hefele
%-------------
% Created on: 06.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function calculates the required power at the battery for front wheel drives
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
%1) Calculate torque
%2) Calculate eta matrix for the motor
%3) Check that the motor does not leave the allowed overload time
%4) Assign Outputs

%% 1) Calculate torque
% assign axle variable 
axle = 1;

%Rotational speed of motor over cycle in 1/min 
n_mot = vehicle.LDS.sim_cons.n_wheels.*vehicle.LDS.GEARBOX{axle}.i_gearbox;  

%Find eta of gearbox
eta_gear = vehicle.LDS.GEARBOX{axle}.eta;

%calculate current operating point torque of motor over cycle in Nm
T_mot = vehicle.LDS.sim_cons.T_wheels./(vehicle.LDS.MOTOR{axle}.quantity.*vehicle.LDS.GEARBOX{axle}.i_gearbox.*eta_gear.^sign(vehicle.LDS.sim_cons.T_wheels));

% calculate mechanical engine power
P_mot = (vehicle.LDS.sim_cons.P_wheels./(vehicle.LDS.MOTOR{axle}.quantity*(eta_gear).^sign(vehicle.LDS.sim_cons.P_wheels)));                           

%% 2) Calculate eta matrix for the motor 
%create grid
[xd,yd] = ndgrid(vehicle.LDS.MOTOR{axle}.diagram.n_scaled,vehicle.LDS.MOTOR{axle}.diagram.T_scaled);

%create interpolation function
F = griddedInterpolant(xd,yd,vehicle.LDS.MOTOR{axle}.diagram.etages','linear','none');

%find current eta (efficiency of front motor)
eta_mot = F(abs(n_mot),abs(T_mot));

P_el_mot = T_mot .* n_mot * 2 * pi./(60 * (eta_mot).^sign(T_mot));

%% 3) Check that the motor does not leave the allowed overload time
%Check overload for motor/motors on the front axle
[vehicle]=check_overload(vehicle,Par,T_mot,n_mot,axle);

%% 4) Assign Outputs  
% operating points
vehicle.LDS.sim_cons.T_mot_f   = T_mot;     %[Nm]
vehicle.LDS.sim_cons.n_mot_f   = n_mot;     %[1/min]
vehicle.LDS.sim_cons.eta_mot_f = eta_mot;   %[-]    

%Mechanical and electrical power of each machine, as well as battery power. All powers in W:
%P_mech (right side of engine - output)
vehicle.LDS.sim_cons.P_mech_mot_f = P_mot;

%P_el (left side of engine - input)
vehicle.LDS.sim_cons.P_el_mot_f = P_el_mot;

%P_batt (right side of battery - output)
vehicle.LDS.sim_cons.P_batt = (vehicle.LDS.sim_cons.P_el_mot_f*vehicle.LDS.MOTOR{axle}.quantity)./((Par.LDS.eta_battery.*Par.LDS.eta_power_electronics).^sign(vehicle.LDS.sim_cons.P_el_mot_f)); %power of battery in W
end