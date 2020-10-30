function [v] = initialize_powertrain_GM_X(v, Par)
%% Description:
%This function initializes a powertrain configuration for one machine, one gear box on the front axle.

% Author:   Lorenzo Nicoletti, Ruben Hefele, Korbinian Moller, FTM, TUM
% Date:     07.09.2020
%% Inputs
% v:    struct with the vehicle parameters
% Par:  struct with LDS input parameters
%% Outputs
% v:    struct with filled MOTOR and GEARBOX fields, according to the choosen topology
%% Implementation:

%initialize empty gearbox struct
v.LDS.GEARBOX = cell(1,2); 

%initialize empty motor struct
v.LDS.MOTOR   = cell(1,2); 

%define drive of topology
v.LDS.settings.drive   = 'front_wheel';

%define variable for recently loaded characteristic
v.LDS.MOTOR{1}.last_characteristic.T = NaN;
v.LDS.MOTOR{1}.last_characteristic.n = NaN;

%define number of gears
v.LDS.GEARBOX{1}.num_gears = length(v.Input.i_gearbox_f);

%% ---------------------------------------------------    Front axle    --------------------------------------------------- 

% Gearbox front 
% Multiple gears possible. Include also the gear ratio of the differential
v.LDS.GEARBOX{1}.i_gearbox              = v.Input.i_gearbox_f*Par.LDS.i_differential;    % gear ratio     

%Assign efficiency of gearbox (including efficiency of differential)
if ~isempty(v.Input.eta_gearbox_f) && all(~isnan(v.Input.eta_gearbox_f))
    v.LDS.GEARBOX{1}.eta                = v.Input.eta_gearbox_f.*Par.LDS.eta_differential;                % efficiency of gearbox
else
    v.LDS.GEARBOX{1}.eta                = Par.LDS.eta_gearbox.*Par.LDS.eta_differential;                  % efficiency of gearbox
end

%check if required inputs are given
if length(v.LDS.GEARBOX{1}.i_gearbox) ~= length(v.LDS.GEARBOX{1}.eta)
    v.LDS = errorlog(v.LDS,'Front axle: gearbox ratio vector and gearbox efficiency vector do not have the same length. eta from gear 1 was assumed for all gears');
    v.LDS.GEARBOX{1}.eta = repmat(v.LDS.GEARBOX{1}.eta(1),1,length(v.LDS.GEARBOX{1}.i_gearbox));
    
end

% Motor front 
v.LDS.MOTOR{1}.type              = v.Input.machine_type_f;                              % ASM or PSM
v.LDS.MOTOR{1}.n_max             = v.Input.n_max_Mot_f;                                 % rotational speed in 1/min
v.LDS.MOTOR{1}.T_max             = v.Input.T_max_Mot_f;                                 % machine torque in Nm
v.LDS.MOTOR{1}.quantity          = 1;                                                   % quantity of motors on front axis
v.LDS.MOTOR{1}.J_M               = Par.LDS.inertia.(v.LDS.MOTOR{1}.type);               % inertia of motor and gearbox in kg m^2
v.LDS.MOTOR{1}.overload_factor   = Par.LDS.overload_factor.(v.LDS.MOTOR{1}.type);       % overload factor motor
v.LDS.MOTOR{1}.overload_duration = Par.LDS.overload_duration.(v.LDS.MOTOR{1}.type);     %overload duration motor
end

