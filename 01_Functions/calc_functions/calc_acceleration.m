function [veh] = calc_acceleration(veh,ParLDS)
%% Description:
%this function determines the time of the vehicle concept for acceleartion from 0-100 km/h 
%Acceleration simulation uses only the first gear

% Author:   Ruben Hefele, Sebastian Krapf, Korbinian Moller, FTM, TUM
% Date:     20.06.19
%% Inputs:
%   vehicle:    struct with the vehicle parameters
%   v_max_sim:  max. simulated velocity in km/h. If vehicle reaches this velocity,
%               the simulation ends
%   t_sim:      simulation time in s. Simulation ends after this time
%               automatically
%% Outputs:
%   sim_acc
%% Sources: 
% Simulation Koch, Krapf
%% Implementation
%Simulationsparameter
%1) Assign simulation variables
%2) Calculate the maximum Torque on the wheels
%3) Calculate acceleration time
%4) Error Log and Assign Output

%% 1) Assign simulation variables

%maximum simulation time in s and maximum step value in s
t_max = veh.LDS.settings.t_sim_max_acc;                                                                                                                   
delta_t = ParLDS.delta_t;     

%define velocity vector in km/h
v = 0:1:veh.LDS.settings.v_max_sim;                                                                                                             

%Start timer
i = 1;                                                                                                                                 

%Assign elevation value in °
alpha = ParLDS.slope_angle_sim_acc;         

%Traction limit marker. It will be set to 1, if the traction limit is exceeded for at least one timestep
exceeded_traction_limit_sim_acc=0;
                                                                                                           
%% 2) Calculate the maximum Torque on the wheels

%Find which axles are filled: [front_axle, rear_axle]
filled_axles = veh.LDS.settings.filled_axles;

%preallocating of vectors for speed
n_motor = zeros(2,length(v));
T_motors = zeros(2,length(v));
T_wheels = zeros(2,length(v));

for nm = find(filled_axles)
        
    % calculate the motor speed at each vehicle speed in 1/min - only gear 1 is used
    n_wheel = v/3.6/(veh.LDS.wheel.r_dyn/1000)*60/2/pi;                                                                                       
    n_motor(nm,:) = n_wheel.*veh.LDS.GEARBOX{nm}.i_gearbox(1);                                                                                 
        
    %interpolate values from the T_max motor curve
    F = griddedInterpolant(veh.LDS.MOTOR{nm}.characteristic(:,2),veh.LDS.MOTOR{nm}.characteristic(:,1),'linear','none');

    %Find torque of all motors on the axle in Nm
    T_motors(nm,:) = veh.LDS.MOTOR{nm}.quantity*F(n_motor(nm,:));                                                 

    %torque of wheels in Nm
    T_wheels(nm,:) = T_motors(nm,:).*veh.LDS.GEARBOX{nm}.i_gearbox(1).*veh.LDS.GEARBOX{nm}.eta(1);                                                                                                                                                     

end

%Find torque of all motors on both axes in Nm
T_wheels_sum =sum(T_wheels,1);

%% 3) Calculate acceleration time:

%define T (in Nm) which is available wheel torque dependant on rotational speed of motor (at timestep 1)
T(1)=T_wheels_sum(1);                                                                                                              

%interpolation of torque
F1 = griddedInterpolant(v,T_wheels_sum);  

%precalculate vector of air resistance force (in N) before loop. Needed for traction limit
F_a= 0.5 * ParLDS.rho_L * veh.LDS.parameters.c_d * veh.LDS.parameters.A/(1000^2) * ((0:(veh.LDS.settings.v_max_sim))/3.6).^2;      

%run function and calculate traction limit vector
[traction_limit] = calc_traction_lim(veh.LDS, ParLDS,F_a,alpha); 

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
    F_d(i) = T(i-1) / (veh.LDS.wheel.r_dyn/1000);                                                                                            

    %calculate air resistance in N
    F_a(i) = 0.5 * ParLDS.rho_L * veh.LDS.parameters.c_d * veh.LDS.parameters.A/(1000^2) * (v_actual(i-1)/3.6).^2;                                           
    
    %calculate roll resistance in N
    F_r(i) = veh.LDS.weights.vehicle_empty_weight_EU * ParLDS.g * veh.LDS.parameters.c_r * cosd(alpha);                                                                          
    
    %calculate slope resistance in case of alpha is not zero in N
    F_g(i) = veh.LDS.weights.vehicle_empty_weight_EU * ParLDS.g * sind(alpha);                                                                                         
    
    %calculate acceleration resistance in N
    F_m(i) = F_d(i) -  F_a(i) - F_r(i) - F_g(i);                                                                                      
    
    %resulting acceleration of vehicle in m/s^2
    acc(i) = F_m(i)/(veh.LDS.weights.vehicle_empty_weight_EU*veh.LDS.parameters.e_i(1));                                                                                           
    
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
veh.LDS.sim_acc.a     = acc;                                        %struct that stores result: acceleration vector in m/s^2
veh.LDS.sim_acc.a_limited = acc_limited;
veh.LDS.sim_acc.v     = v_actual(1:i);                              %velocity vector in km/h
veh.LDS.sim_acc.t     = 0:delta_t:delta_t*(i-1);                    %time vector in s
veh.LDS.sim_acc.acc_time_is   = delta_t*(i-1);                      %t_max_acc in s

%Resistance and max torque:
veh.LDS.sim_acc.T_wheels     = [T,F1(v_actual(i))];                 %torque at the wheels in Nm
veh.LDS.sim_acc.resistance.F_a   = F_a(1:i);                        %air resistance in N
veh.LDS.sim_acc.resistance.F_r   = F_r;                             %roll resistance in N
veh.LDS.sim_acc.resistance.F_g   = F_g;                             %gradient resistance in N
veh.LDS.sim_acc.resistance.F_m   = F_m;                             %acceleration resistance in N
veh.LDS.sim_acc.resistance.F_tot  = F_d;                            %driving force in N

%Marker traction limit:
veh.LDS.sim_acc.traction_lim=traction_limit_vec(1:i);               %Vector with the traction limits
veh.LDS.sim_acc.exceeded_traction_limit=exceeded_traction_limit_sim_acc;  %Marker, to be used later for Errorlog

end