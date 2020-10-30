function[veh] = calc_gradeability(veh,Par)
%% Description:
%
%author:    Korbinian Moller, TUM
%date:      21.09.2020
%% Input:
%vehicle struct
%parameter struct
%% Outputs:
%updated vehicle struct
%% Implementation:
% 1) Load local variables
% 2) calculate alpha
% 3) calculate alpha symbolically - if activated
% 4) Text Output - if activated

%% 1) Load local variables
%initialize needed variables:
text_output = 1;                                %output text on or off
solver = 0;                                     %solve equation symbolically
v_step = 1;                                     %step for speed vector
c_d=veh.LDS.parameters.c_d;                     %drag coefficient in [-]
rho=Par.LDS.rho_L;                              %air density in kg/m^3
c_r=veh.LDS.parameters.c_r;                     %roll resistance coefficient in [-]
A=veh.LDS.parameters.A/(1000^2);                %cross sectional areain m^2
m=veh.LDS.weights.vehicle_empty_weight_EU;      %empty weight with driver in kg
g=Par.LDS.g;                                    %gravitational acceleration in m/s^2  
v = 0:v_step:100;                               %velocity vector in km/h
speed_points = [0, 10, 30, 50];                 %Speed values at which the gradeability is output

%Find which axles are filled: [front_axle, rear_axle]
filled_axles = veh.LDS.settings.filled_axles;

%preallocating of vectors for speed
n_motor = zeros(2,length(v));
T_motors = zeros(2,length(v));
T_wheels = zeros(2,length(v));

%% 2) calculate alpha
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

%calculate F_dr matrix
F_dr = T_wheels_sum/(veh.LDS.wheel.r_dyn/1000);

%Calculate vehicle resistance
F_a = 0.5*c_d*rho*A*((v/3.6).^2);                                           %Air resistance in N 
F_r = (m*g*c_r);                                                            %Rolling resistance in N

%calculate alpha - equation from "Fahrzeugdynamik - Mechanik des bewegten
%Fahrzeugs - Stefan Breuer, Andrea Rohrbach-Kerl - DOI
%10.1007/978-3-658-09475-1 - S. 94
alpha_deg = asind(((F_dr- F_r - F_a)/(m*g)));
q_prz = 100 * tand(alpha_deg);

% write output
veh.LDS.gradeability.T_wheels = T_wheels_sum;
veh.LDS.gradeability.F_dr = F_dr;
veh.LDS.gradeability.v = v;
veh.LDS.gradeability.alpha_deg = alpha_deg;
veh.LDS.gradeability.q_prz = q_prz;

%% 3) calculate alpha symbolically - if activated
if solver == 1
    syms alpha_sym
    %Calculate vehicle resistance
    F_m = 0;                                                                    %No acceleration
    F_a = 0.5*c_d*rho*A*((v/3.6).^2);                                           %Air resistance in N 
    F_g = (m * g * sind(alpha_sym));                                                %Slope resistance in N
    F_r = (m*g*c_r* cosd(alpha_sym));                                       %Rolling resistance in N
    F_tot = F_m + F_a + F_g + F_r;                                              %total resistance in N
    %calculate solution
    x = cell(1,numel(v));
    for i = 1:numel(v)
        x{i} = solve(F_dr(i) - F_tot(i) == 0,alpha_sym);
    end
    
    alpha_solver = zeros(2,numel(v));
    %convert 2 alpha
    for i = 1:numel(v)
        alpha_solver(1,i) = double(abs(x{i}(1)));
        alpha_solver(2,i) = double(abs(x{i}(2)));
    end
    
    %write additional output
    veh.LDS.gradeability.sym_alpha_deg = alpha_solver;
    veh.LDS.gradeability.sym_q_prz = 100*tand(alpha_solver);
    veh.LDS.gradeability.sym_F_tot = F_tot;
    veh.LDS.gradeability.sym_F_m = F_m;
    veh.LDS.gradeability.sym_F_a = F_a;
    veh.LDS.gradeability.sym_F_g = F_g;
    veh.LDS.gradeability.sym_F_r = F_r;

end

%% 4) Text Output
if text_output == 1
    for i = 1:numel(speed_points)
        fprintf('gradeability at %i km/h: %.2f percent \n', v(speed_points(i)+1), q_prz(speed_points(i)+1));
    end
end

end




