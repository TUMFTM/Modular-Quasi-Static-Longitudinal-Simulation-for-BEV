function [v] = initialize_powertrain_2G2M_X(v, Par)
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich), Korbinian Moller
%-------------
% Created on: 06.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function initializes a powertrain configuration for 2 machines and 2 gearboxes on the front axle.
%              The function assumes that the 2 machines on the same axle have are of the same type and
%              have the same power. The same assumption is made also for the gearboxes
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
v.LDS.settings.drive   = 'front_wheel';

%define variable for recently loaded characteristic
v.LDS.MOTOR{1}.last_characteristic.T = NaN;
v.LDS.MOTOR{1}.last_characteristic.n = NaN;

%define number of gears
v.LDS.GEARBOX{1}.num_gears = length(v.Input.i_gearbox_f);


%% ---------------------------------------------------    Front axle    --------------------------------------------------- 
% Gearbox front only single gear. No differential, as there is one motor per wheel
v.LDS.GEARBOX{1}.i_gearbox       = v.Input.i_gearbox_f;                         % gear ratio 

%Assign efficiency of gearbox (efficiency of differential not included, as there is one machine per wheel)
if ~isempty(v.Input.eta_gearbox_f) && all(~isnan(v.Input.eta_gearbox_f))
    v.LDS.GEARBOX{1}.eta                = v.Input.eta_gearbox_f;                % efficiency of gearbox
else
    v.LDS.GEARBOX{1}.eta                = Par.LDS.eta_gearbox;                  % efficiency of gearbox
end

%check if required inputs are given
if length(v.LDS.GEARBOX{1}.i_gearbox) ~= length(v.LDS.GEARBOX{1}.eta)
    v.LDS = errorlog(v.LDS,'Front axle: gearbox ratio vector and gearbox efficiency vector do not have the same length. eta from gear 1 was assumed for all gears');
    v.LDS.GEARBOX{1}.eta = repmat(v.LDS.GEARBOX{1}.eta(1),1,length(v.LDS.GEARBOX{1}.i_gearbox));
    
end

% Motor front 
v.LDS.MOTOR{1}.type              = v.Input.machine_type_f;                              % ASM or PSM
v.LDS.MOTOR{1}.T_max             = v.Input.T_max_Mot_f;                                 % machine torque in Nm
v.LDS.MOTOR{1}.quantity          = 2;                                                   % quantity of motors on front axis
v.LDS.MOTOR{1}.J_M               = Par.LDS.inertia.(v.LDS.MOTOR{1}.type);               % inertia of motor in kg m^2
v.LDS.MOTOR{1}.overload_factor   = Par.LDS.overload_factor.(v.LDS.MOTOR{1}.type);       % overload factor motor
v.LDS.MOTOR{1}.overload_duration = Par.LDS.overload_duration.(v.LDS.MOTOR{1}.type);     %overload duration motor
end