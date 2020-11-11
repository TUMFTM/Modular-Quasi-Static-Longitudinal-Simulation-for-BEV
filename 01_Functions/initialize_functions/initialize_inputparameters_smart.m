function [vehicle,Parameters] = initialize_inputparameters_smart(vehicle)
%% Description: Smart fortwo electric
%This function initializes the vehicle struct. This is needed to avoid errors in the next steps.

%Author:    Korbinian Moller, FTM, TUM 
%date:      21.10.2020
%% Outputs:
%veicle struct, filled with the Inputs
%parameters struct, filled with the constant values
%% Sources
%[1] Vieweg Handbuch Kraftfahrzeugtechnik - Pischinger und Steiffert
%[2] M. Mitschke und H. Wallentowitz, Dynamik der Kraftfahrzeuge
%[3] Malkic - Aufbau eines Tools zur Berechnung von möglichen Motor-Getriebe-Kombinationen aus Konzeptanforderungen, Bachelorthesis, München, Lehrstuhl für Fahrzeugtechnik, TUM
%[4] Haken - Grundlagen der Kraftfahrzeugtechnik
%[5] Hoppert - Analytische und experimentelle Untersuchungen zum Wirkungsgradverhalten von Achsgetrieben
%[6] Tschöke - Die Elektrifizierung des Antriebsstrangs
%[7] Fuchs, J - Analyse der Wechselwirkungen und Entwicklungspotentiale in der Auslegung elektrifizierter Fahrzeugkonzepte. Technische Universität,München, Fakultät für Maschinenwesen, Lehrstuhl für Fahrzeugtechnik, TUM, Dissertation, 2014
%[8] Vaillant - Design Space Exploration zur multikriteriellen Optimierung elektrischer Sportwagenantriebsstränge: Variation von Topologie und Komponenteneigenschaften zur Steigerung
%[9] Matz - Beschreibung der Modellierungsart sowie der Modellierungsparameter von Elektrofahrzeugen in der Konzeptphase
%[10] Hefele - Hefele - Implementierung einer MATLAB Längsdynamiksimulation für Elektrofahrzeuge, Technische Universität,München, Fakultät für Maschinenwesen, Lehrstuhl für Fahrzeugtechnik, TUM, Semester thesis
%[11] https://www.gesetze-bayern.de/Content/Document/BayGaV-3
%[12] Kirchner - Leistungsübertragung in Fahrzeuggetrieben
%[13] Wissenschaftlicher Grundlagenbericht: Umweltbilanzen Elektromobilität
%% Implementation:
% 1) Section where the user can assign the Inputparameters
% 2) Section where the user can change the fixed values

%% 1) Section where the user can assign the Inputparameters
%%----------------------------------------------------Vehicle Inputs----------------------------------------------------
vehicle.Input.topology                  =   'X_GM';        %vehicle topology. See functions initialize powertrain
vehicle.Input.weight_extra_equipment    =   0;              %weight of the extra equipment
vehicle.Input.vehicle_payload           =   0;              %payload of vehicle in kg
vehicle.Input.number_passengers         =   2;              %Number of passengers
vehicle.Input.vehicle_empty_weight      =   995-75;        %empty weight netto in kg (no driver)
vehicle.Input.vehicle_sim_cons_weight   =   1049;            %vehicle weight for energy consumption sim, optional
vehicle.Input.r_tire                    =   291;            %static tire radius in mm 
vehicle.Input.w_tire                    =   165;            %tire width in mm, (Optional, needed for e_i calculation)
vehicle.Input.vehicle_width             =   1559;           %vehicle width in mm
vehicle.Input.vehicle_height            =   1565;           %vehicle height in mm
vehicle.Input.wheelbase                 =   1867;           %wheelbase in mm
vehicle.Input.ground_clearance          =   'flatfloor';    %ground clearance (more than 200mm ==> highfloor, otherwise flatfloor)                 
vehicle.Input.driving_cycle             =   'NEFZ';         %driving cycle (cycle are in folder 04_Drive_Cycle)
vehicle.Input.power_auxiliaries         =    0.2;               %power of auxiliary users in kW. (optional, otherwise NaN)

%Obligatory Inputs: Max Velocity, Machine Type and Rotational Level (Latter defines the motor characteristics, which will be loaded for futher scaling)
vehicle.Input.max_speed                 =   125;            %maximum velocity in km/h
vehicle.Input.machine_type_f            =   'PSM';          %machine type front 'PSM' or 'ASM'
vehicle.Input.machine_type_r            =   'PSM';          %machine type rear 'PSM' or 'ASM'

%%-----------------------------------------------------Motor Inputs-----------------------------------------------------
%At least n_max or i_gearbox have to be assigned!! The non assigned values can be set to NaN

vehicle.Input.n_max_Mot_f               =   NaN;      %max rotational speed of front machine in 1/min, optional
vehicle.Input.n_max_Mot_r               =   NaN;      %max rotational speed of front machine in 1/min, optional

vehicle.Input.i_gearbox_f               =   NaN;          %gear ratio front
vehicle.Input.i_gearbox_r               =   9.34;         %gear ratio rear

vehicle.Input.T_max_Mot_f               =  NaN;         %max torque of front machine in Nm
vehicle.Input.T_max_Mot_r               =  130;         %max torque of rear machine in Nm

%Optional: If the given torque is not sufficient to reach this acceleration time, the code will scale the torque
%In the case the user does not want to give a required acceleration time value, it can be set as NaN (see line underneath)
vehicle.Input.acceleration_time_req     =   11.5;    %required acceleration time in s

%Motor characteristic inputs
vehicle.Input.characteristic_forced = {NaN,NaN};  %select forced characteristic per axle, otherwise NaN, f.e. characteristic_forced = {'PSM_M270_n12000_0.2.mat',NaN};
vehicle.Input.ratio_req = [0.3,0.3];              %select required angular speed ratio per axle
vehicle.Input.weight_T = [1,1];                   %select Torque weight factor per axle
vehicle.Input.weight_n = [1,1];                   %select speed weight factor per axle
vehicle.Input.weight_ratio = [1,1];               %select angular speed ratio weight factor per axle

%%-------------------------------------------------Resistance (Optional-------------------------------------------------
%If not assigned will be calculated or modeled with constant values (taken from Parameter struct)
vehicle.Input.e_i                       =   NaN;            %e_i value (optional)
vehicle.Input.c_r                       =   NaN;            %roll resistance coefficient, (optional)
vehicle.Input.c_d                       =   0.33;            %air resistance coefficient, (optional)
vehicle.Input.eta_gearbox_r             =   NaN;            %efficiency of rear gearbox, optional
vehicle.Input.eta_gearbox_f             =   NaN;            %efficiency of front gearbox, optional

%%----------------------------------------------------------------------------------------------------------------------

%Initialize other parameters which are needed for further calculations:
vehicle.masses.vehicle_empty_weight_EU=vehicle.Input.vehicle_empty_weight +75;  %vehicle weight with driver
vehicle.masses.vehicle_sim_cons_weight = vehicle.Input.vehicle_sim_cons_weight;
vehicle.masses.vehicle_max_weight=vehicle.masses.vehicle_empty_weight_EU+vehicle.Input.vehicle_payload +vehicle.Input.weight_extra_equipment+(vehicle.Input.number_passengers-1)*(75); %weight of the heaviest vehicle variant
vehicle.dimensions.CX=[];

%% 2) Section where the user can change the fixed values

Parameters=struct;

%%----------------------------------------------------Characteristic Path----------------------------------------------------
%looks like: '02_Characteristics_Diagrams'
Parameters.LDS.char_path = '02_Characteristics_Diagrams'; %Path where engine characteristics are stored within the folder /lds_paumani/03_LDS_Hefele/XXXXX


%%--------------------------------Parameters for calculation of vehicle resistance force--------------------------------
Parameters.LDS.c_d                      =   0.258;      %drag coefficient in [-]. Source [1]
Parameters.LDS.c_r                      =   0.009;      %roll resistance coefficient in [-]. Source [2]
Parameters.LDS.e_i                      =   1.08;       %fix e_i value (used only if there are no tire values) in [-]. Source [3]
Parameters.LDS.g                        =   9.81;       %gravitational acceleration in m/s^2
Parameters.LDS.rho_L                    =   1.20;       %air density in kg/m^3
Parameters.LDS.mue_max                  =   1.15;       %driving traction coefficient (wheel-asphalt) in [-]. Source [3]
Parameters.LDS.correction_area          =   0.81;       %correction factor of cross sectional area (for calculation of air resistance) in [-]. Source [4] %0.81

%%-------------------------------Parameters for calculation of gearbox and gearbox losses-------------------------------
Parameters.LDS.i_differential           =   1;          %gear ratio of differential in [-]. Source [3]
Parameters.LDS.eta_differential         =   0.97;      %efficiency of differential in [-]. Source [5]
Parameters.LDS.eta_gearbox              =   0.97;       %efficiency of gearbox in [-]. Source [6]
Parameters.LDS.correction_gearbox       =   0.9142;       %correction factor for gear ratio (for electric vehicles) in [-]. Source [3]

%-------------------------------------------Parameters for calculation of e_i-------------------------------------------
Parameters.LDS.m_wheel_P9               =   538.02;                         %factor P9 for calculating mass of wheels in kg m^2. Source [7]
Parameters.LDS.m_wheel_P10              =   1.87;                           %factor P10 for calculating mass of wheels in kg. Source [7]
Parameters.LDS.inertia.ASM              =   0.0555;                         %inertia ASM + gearbox in kg m^2. Source [3]
Parameters.LDS.inertia.PSM              =   0.0342;                         %inertia PSM + gearbox in kg m^2. Source [3]

%----------------------------Parameters for efficiency calculation foe E-machine and battery----------------------------
Parameters.LDS.eta_battery              =   0.92;                              %fix efficiency of battery in [-]. Source [8] 0.92
Parameters.LDS.eta_power_electronics    =   0.95;                           %efficiency of power electronics in [-]. Source [6]
Parameters.LDS.overload_factor.ASM      =   3;                              %overload factor ASM in [-]. Source [9]
Parameters.LDS.overload_factor.PSM      =   2;                              %overload factor PSM in [-]. Source [9]
Parameters.LDS.overload_duration.ASM    =   30;                             %overload duration ASM in s. Source [9]
Parameters.LDS.overload_duration.PSM    =   30;                             %overload duration PSM in s. Source [9]

%%---------------------------------Parameters for calculation of the slope requirements---------------------------------
Parameters.LDS.trailer_factor.highfloor =   0.98;                           %trailer factor for highfloor vehicles. Source [10]
Parameters.LDS.trailer_factor.flatfloor =   0.83;                           %trailer factor for flatfloor vehicles. Source [10]
Parameters.LDS.slope_1.flatfloor        =   0;                              %slope 1 for flatfloor vehicles for max weight. Additional slot for simulation of other slopes
Parameters.LDS.slope_1.highfloor        =   0.25;                           %slope 1 for highfloor vehicles for max weight. Additional slot for simulation of other slopes
Parameters.LDS.slope_2.flatfloor        =   0.15;                           %slope 2 for flatfloor vehicles for max weight. Source [11]
Parameters.LDS.slope_2.highfloor        =   0.15;                           %slope 2 for highfloor vehicles for max weight. Source [11]
Parameters.LDS.slope_3.flatfloor        =   0.12;                           %slope 3 for flatfloor vehicles for max weight + trailer mass. Source [12]
Parameters.LDS.slope_3.highfloor        =   0.12;                           %slope 3 for highfloor vehicles for max weight + trailer mass. Source [12]

%%----------------------------------------Parameters for the simulation settings----------------------------------------
Parameters.LDS.max_diff_to_acc          =   0.05;                            %tolerance of acceleration time in s
Parameters.LDS.max_diff_to_max_speed    =   2;                              %tolerance of actual speed to max speed in km/h
Parameters.LDS.t_sim                    =   25;                             %maximum acceleration simulation time in s
Parameters.LDS.v_max_sim                =   100;                            %max speed in acceleration simulation in km/h
Parameters.LDS.delta_t                  =   0.1;                            %step size in acceleration simulation in s
Parameters.LDS.delta_t_con              =   0.1;                            %step size in consumption simulation in s
Parameters.LDS.axle_load_front          =   0.5;                                %axis load distribution on front axis 
Parameters.LDS.axle_load_rear           =   1-Parameters.LDS.axle_load_front;   %axis load distribution on rear axis 
Parameters.LDS.slope_angle_sim_acc      =   0;                              %Slope angle for acceleration simulation in °
Parameters.LDS.torque_optimizer         =   NaN;                            %Torque optimizer if only one engine is overloaded - on/off (1/NaN)

end


