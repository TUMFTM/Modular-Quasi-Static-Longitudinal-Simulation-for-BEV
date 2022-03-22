function [vehicle] = calc_acceleration(vehicle,Par)
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich), Korbinian Moller
%-------------
% Created on: 06.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function determines the time of the vehicle concept for acceleartion from 0-100 km/h 
%              Acceleration simulation uses only the first gear
% ------------
% Sources: More information regarding the implementation of the LDS functions is available at:
%          [1] K. Moller, „Validierung einer MATLAB Längsdynamiksimulation für die Auslegung von Elektrofahrzeugen,“ Bachelor thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2020. 
%          [2] K. Moller, „Antriebsstrangmodellierung zur Optimierung autonomer Elektrofahrzeuge,“Semester thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2021.
% ------------
% Input: vehicle: The vehicle structure -> Stores the calculated component volumes and masses
%        Parameters: The Parameters structure -> Stores the constant values and regressions for volume and mass models
% ------------
% Output: The acceleration time and further results of the acceleration simulation
% ------------
%% Implementation
%1) Assign simulation variables
%2) Calculate the maximum Torque on the wheels
%3) Calculate acceleration time
%4) Error Log and Assign Output

%% 1) Assign simulation variables
%maximum simulation time in s and maximum step value in s
t_max = vehicle.LDS.settings.t_sim_max_acc;                                                                                                                   
delta_t = Par.LDS.delta_t;     

%define velocity vector in km/h
v = 0:1:vehicle.LDS.settings.v_max_sim;                                                                                                             

%Start timer
i = 1;                                                                                                                                 

%Assign elevation value in °
alpha = Par.LDS.slope_angle_sim_acc;         

%Traction limit marker. It will be set to 1, if the traction limit is exceeded for at least one timestep
exceeded_traction_limit_sim_acc=0;
                                                                                                           
%% 2) Calculate the maximum Torque on the wheels
%Find which axles are filled: [front_axle, rear_axle]
filled_axles = vehicle.LDS.settings.filled_axles;

%preallocating of vectors for speed
n_motor = zeros(2,length(v));
T_motors = zeros(2,length(v));
T_wheels = zeros(2,length(v));

for axle = find(filled_axles)
        
    % calculate the motor speed at each vehicle speed in 1/min - only gear 1 is used
    n_wheel = v/3.6/(vehicle.LDS.wheel.r_dyn/1000)*60/2/pi;                                                                                       
    n_motor(axle,:) = n_wheel.*vehicle.LDS.GEARBOX{axle}.i_gearbox(1);                                                                                 
        
    %interpolate values from the T_max motor curve
    F = griddedInterpolant(vehicle.LDS.MOTOR{axle}.characteristic(:,2),vehicle.LDS.MOTOR{axle}.characteristic(:,1),'linear','none');

    %Find torque of all motors on the axle in Nm
    T_motors(axle,:) = F(n_motor(axle,:));
    
    %Find eta of gearbox
    eta_gear = vehicle.LDS.GEARBOX{axle}.eta;

    %torque of wheels in Nm    
    T_wheels(axle,:) = vehicle.LDS.MOTOR{axle}.quantity * T_motors(axle,:).*vehicle.LDS.GEARBOX{axle}.i_gearbox(1).*eta_gear;   

end

%Find torque of all motors on both axes in Nm
T_wheels_sum =sum(T_wheels,1);

%% 3) Calculate acceleration time:
%define T (in Nm) which is available wheel torque dependant on rotational speed of motor (at timestep 1)
T(1)=T_wheels_sum(1);                                                                                                              

%interpolation of torque
F1 = griddedInterpolant(v,T_wheels_sum);  

%precalculate vector of air resistance force (in N) before loop. Needed for traction limit
F_a= 0.5 * Par.LDS.rho_L * vehicle.LDS.parameters.c_d * vehicle.LDS.parameters.A/(1000^2) * ((0:(vehicle.LDS.settings.v_max_sim))/3.6).^2;      

%run function and calculate traction limit vector
[traction_limit] = calc_traction_lim(vehicle.LDS, Par,F_a,alpha); 

%interpolate F_a and the traction limit
F2 = griddedInterpolant(F_a,traction_limit); 

%preallocate velocity in km/h
v_actual = zeros(1,10e3);         

%start acceleration maneuver while velocity is smaller than max velocity and simulation time is smaller than max simulation time
while v_actual(i) < max(v) && (delta_t*(i-1)) < t_max
    
    %count timer up in s
    i = i+1;                                                                                                                          
    
    %maximum available torque (Nm) at wheels from all motors dependent on velocity
    T(i-1) = F1(v_actual(i-1)); %use interpolation function to interpolate                                                          
    
    %calculate drive force in N from the T of the previous timestep
    F_d(i) = T(i-1) / (vehicle.LDS.wheel.r_dyn/1000);                                                                                            

    %calculate air resistance in N
    F_a(i) = 0.5 * Par.LDS.rho_L * vehicle.LDS.parameters.c_d * vehicle.LDS.parameters.A/(1000^2) * (v_actual(i-1)/3.6).^2;                                           
    
    %calculate roll resistance in N
    F_r(i) = vehicle.LDS.weights.vehicle_empty_weight_EU * Par.LDS.g * vehicle.LDS.parameters.c_r * cosd(alpha);                                                                          
    
    %calculate slope resistance in case of alpha is not zero in N
    F_g(i) = vehicle.LDS.weights.vehicle_empty_weight_EU * Par.LDS.g * sind(alpha);                                                                                         
    
    %calculate acceleration resistance in N
    F_m(i) = F_d(i) -  F_a(i) - F_r(i) - F_g(i);                                                                                      
    
    %resulting acceleration of vehicle in m/s^2
    acc(i) = F_m(i)/(vehicle.LDS.weights.vehicle_empty_weight_EU*vehicle.LDS.parameters.e_i(1));                                                                                           
    
    %interpolate max acceleration due to traction in m/s^2
    traction_limit_vec(i)= F2(F_a(i));                                                                                                         
     
    %if the acceleration is above the traction limit, assign the traction limit as max acceletation!
    acc_limited(i) = min(acc(i),traction_limit_vec(i));       %vector with the acceleration of each simulation step in m/s^2
    
    % resulting velocity of next time step
    v_actual(i) = v_actual(i-1)+ acc_limited(i)*3.6 * delta_t;     %vector with the veloctiy of each simulation step in km/h
    
end

%% 4) Error log and assign Output
%Check traction limit:
if sum(acc>traction_limit_vec)>=1    
    %Traction limit has been exceeded at least once
    exceeded_traction_limit_sim_acc=1;  
end

%Results acceleration simulation:
vehicle.LDS.sim_acc.a           = acc;                     %struct that stores result: acceleration vector in m/s^2
vehicle.LDS.sim_acc.a_limited   = acc_limited;             %Vector containing the point where the acceleration time has to be limited due to traction loss
vehicle.LDS.sim_acc.v           = v_actual(1:i);           %velocity vector in km/h
vehicle.LDS.sim_acc.t           = 0:delta_t:delta_t*(i-1); %time vector in s
vehicle.LDS.sim_acc.acc_time_is = delta_t*(i-1);           %t_max_acc in s

%Resistance and max torque:
vehicle.LDS.sim_acc.T_wheels         = [T,F1(v_actual(i))]; %torque at the wheels in Nm
vehicle.LDS.sim_acc.resistance.F_a   = F_a(1:i);            %air resistance in N
vehicle.LDS.sim_acc.resistance.F_r   = F_r;                 %roll resistance in N
vehicle.LDS.sim_acc.resistance.F_g   = F_g;                 %gradient resistance in N
vehicle.LDS.sim_acc.resistance.F_m   = F_m;                 %acceleration resistance in N
vehicle.LDS.sim_acc.resistance.F_tot = F_d;                 %driving force in N

%Marker traction limit:
vehicle.LDS.sim_acc.traction_lim            = traction_limit_vec(1:i);         %Vector with the traction limits
vehicle.LDS.sim_acc.exceeded_traction_limit = exceeded_traction_limit_sim_acc; %Marker, to be used later for ErrorLog
end