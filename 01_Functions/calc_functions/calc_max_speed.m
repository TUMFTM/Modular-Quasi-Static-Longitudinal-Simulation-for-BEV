function v_max_output = calc_max_speed(vehicle,Par)
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich), Korbinian Moller
%-------------
% Created on: 06.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function calculates the maximum attainable speed of the vehicle
% ------------
% Sources: More information regarding the implementation of the LDS functions is available at:
%          [1] K. Moller, „Validierung einer MATLAB Längsdynamiksimulation für die Auslegung von Elektrofahrzeugen,“ Bachelor thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2020. 
%          [2] K. Moller, „Antriebsstrangmodellierung zur Optimierung autonomer Elektrofahrzeuge,“Semester thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2021.
% ------------
% Input: vehicle: The vehicle structure -> Stores the calculated component volumes and masses
%        Par: The Parameters structure -> Stores the constant values and regressions for volume and mass models
% ------------
% Output: vehicle struct with v_max of vehicle
% ------------
%% Implementation:
%1) Initialize needed Inputs
%2) Calculate speed at the axles
%3) Calculate max speed
%4) Assign Output

%% 1) Initialize needed Inputs:
%Number of speed and rotational speed steps
steps = 100;

%initialize needed variables:
c_d = vehicle.LDS.parameters.c_d;                   %drag coefficient in [-]
rho = Par.LDS.rho_L;                                %air density in kg/m^3
c_r = vehicle.LDS.parameters.c_r;                   %roll resistance coefficient in [-]
A   = vehicle.LDS.parameters.A/(1000^2);            %cross sectional areain m^2
m   = vehicle.LDS.weights.vehicle_empty_weight_EU;	%empty weight with driver in kg
g   = Par.LDS.g;                                    %gravitational acceleration in m/s^2            

%% 2) Calculate speed at the axles
%Find which axles are filled: [front_axle, rear_axle]
filled_axles = vehicle.LDS.settings.filled_axles;

%Preallocate variables for speed
F_d_base = zeros(2,steps);
v_axle = zeros(2,steps);

for axle = find(filled_axles) %i=1 for front and i=2 for rear axle
    
    %Motor moment (in Nm) and Motor speed (in 1/s): 
    T_mot = vehicle.LDS.MOTOR{axle}.characteristic(:,1)*vehicle.LDS.MOTOR{axle}.quantity;       
    n_mot = vehicle.LDS.MOTOR{axle}.characteristic(:,2); 
    
    %Find eta of gearbox
    eta_gear = vehicle.LDS.GEARBOX{axle}.eta;

    %Wheel moment (in Nm) and wheel speed (in rad/s): 
    T_wheel_base = T_mot.* eta_gear * vehicle.LDS.GEARBOX{axle}.i_gearbox;
    n_wheel_base = n_mot/vehicle.LDS.GEARBOX{axle}.i_gearbox;
    
    %Interpolate
    F = griddedInterpolant(n_wheel_base, T_wheel_base);
    
    %Reactualize rotational wheel speed and torque:
    n_wheel = linspace(1,max(n_wheel_base), steps);
    T_wheel = F(n_wheel);

    %Calculate driving force in N
    F_d_base(axle,:) = T_wheel/(vehicle.LDS.wheel.r_dyn/1000);
    
    %Calculate v at the axle in m/s
    v_axle(axle,:) = n_wheel * 2*pi/60 * (vehicle.LDS.wheel.r_dyn/1000);

end

%% 3) Calculate max speed
%In the AWD case the max speed corresponds to the smallest reachable speed between front and rear axle, i.e. the speed when one of the two axes reaches its power limit.
if sum(filled_axles)==2 %all wheel drive
    v_max=min(max(v_axle,[],2));
else
    v_max=max(v_axle(find(filled_axles),:));
end

%Reactualize vector with max possible speed in m/s
v = linspace(0, v_max, steps);

%Calculate total driving force (sum between driving force at the front and the rear axle:
for ii = find(filled_axles) 
    
    F2=griddedInterpolant(v_axle(ii,:), F_d_base(ii,:));
    
    %Axle driving force in N
    F_d(ii,:)=F2(v);
    
end

%Total driving force in N (both rear and front axle)
if sum(filled_axles)==2     %all wheel drive
    F_d_tot=sum(F_d);
elseif filled_axles(1)      %front wheel drive
    F_d_tot=F_d;
else                        %rear wheel drive
    F_d_tot=F_d(2,:);
end

%Calculate vehicle resistance
F_m = 0;                                %No acceleration at v_max, i.e. no acceleration resistance
F_a = 0.5*c_d*rho*A*(v.^2);             %Air resistance in N 
F_g = 0;                                %Slope resistance is considered as 0 (no slope)
F_r = m*g*c_r*ones(1,numel(v));         %Rolling resistance in N
F_tot = F_m+F_a+F_g+F_r;                %Total resistance in N

%Find the point when the difference between driving force and resistance is the smallest (i.e. close to power limit)
[~,i_vmax] = min(abs(F_d_tot-F_tot));

%% 4) Assign Output
%Maximum vehicle speed in km/h
v_max_output = v(i_vmax)*3.6;
end