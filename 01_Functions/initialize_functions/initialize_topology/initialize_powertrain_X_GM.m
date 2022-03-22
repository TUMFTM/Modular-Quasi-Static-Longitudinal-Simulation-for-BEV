function [v] = initialize_powertrain_X_GM(v, Par)
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich), Korbinian Moller
%-------------
% Created on: 06.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function initializes a powertrain configuration for one machine, one gear box on the rear axle.
% ------------
% Sources: More information regarding the implementation of the LDS functions is available at:
%          [2] K. Moller, „Validierung einer MATLAB Längsdynamiksimulation für die Auslegung von Elektrofahrzeugen,“ Bachelor thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2020. 
%          [3] K. Moller, „Antriebsstrangmodellierung zur Optimierung autonomer Elektrofahrzeuge,“Semester thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2021.
% ------------
% Input: v: The vehicle structure -> Stores the calculated component volumes and masses
%        Par: The Parameters structure -> Stores the constant values and regressions for volume and mass models
% ------------
% Output: The vehicle struct with filled MOTOR and GEARBOX fields, according to the choosen topology
% ------------
%% Implementation:
%initialize empty gearbox struct
v.LDS.GEARBOX = cell(1,2); 

%initialize empty motor struct
v.LDS.MOTOR   = cell(1,2); 

%define drive of topology
v.LDS.settings.drive   = 'rear_wheel';

%define variable for recently loaded characteristic
v.LDS.MOTOR{2}.last_characteristic.T = NaN;
v.LDS.MOTOR{2}.last_characteristic.n = NaN;

%define number of gears
v.LDS.GEARBOX{2}.num_gears = length(v.Input.i_gearbox_r);


%% ---------------------------------------------------    Rear axle    --------------------------------------------------- 
% Gearbox rear only single gear. Include also the gear ratio of the differential
v.LDS.GEARBOX{2}.i_gearbox       = v.Input.i_gearbox_r*Par.LDS.i_differential;  % gear ratio

%Assign efficiency of gearbox (including efficiency of differential)
if ~isempty(v.Input.eta_gearbox_r) && all(~isnan(v.Input.eta_gearbox_r))
    v.LDS.GEARBOX{2}.eta                = v.Input.eta_gearbox_r*Par.LDS.eta_differential;              
else
    v.LDS.GEARBOX{2}.eta                = Par.LDS.eta_gearbox*Par.LDS.eta_differential ;                 
end     

%check if required inputs are given
if length(v.LDS.GEARBOX{2}.i_gearbox) ~= length(v.LDS.GEARBOX{2}.eta)
    v.LDS = errorlog(v.LDS,'Rear axle: gearbox ratio vector and gearbox efficiency vector do not have the same length. eta from gear 1 was assumed for all gears');
    v.LDS.GEARBOX{2}.eta = repmat(v.LDS.GEARBOX{2}.eta(1),1,length(v.LDS.GEARBOX{2}.i_gearbox));
    
end

% Motor rear 
v.LDS.MOTOR{2}.type              = v.Input.machine_type_r;                              % ASM or PSM
v.LDS.MOTOR{2}.T_max             = v.Input.T_max_Mot_r;                                 % machine torque in Nm
v.LDS.MOTOR{2}.quantity          = 1;                                                   % quantity of motors on rear axis
v.LDS.MOTOR{2}.J_M               = Par.LDS.inertia.(v.LDS.MOTOR{2}.type);               % inertia of motor and gearbox in kg m^2
v.LDS.MOTOR{2}.overload_factor   = Par.LDS.overload_factor.(v.LDS.MOTOR{2}.type);       % overload factor motor
v.LDS.MOTOR{2}.overload_duration = Par.LDS.overload_duration.(v.LDS.MOTOR{2}.type);     %overload duration motor
end