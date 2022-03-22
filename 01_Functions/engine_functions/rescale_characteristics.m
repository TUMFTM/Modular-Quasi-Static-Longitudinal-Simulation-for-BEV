function [vehLDS] = rescale_characteristics(vehLDS,n_max,n_factor,T_max,T_factor,axles) 
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich), Korbinian Moller
%-------------
% Created on: 06.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: Adapts machine operating map to fulfill v_max requirement if no axles are given, all available axles will be scaled
% ------------
% Sources: More information regarding the implementation of the LDS functions is available at:
%          [1] K. Moller, „Validierung einer MATLAB Längsdynamiksimulation für die Auslegung von Elektrofahrzeugen,“ Bachelor thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2020. 
%          [2] K. Moller, „Antriebsstrangmodellierung zur Optimierung autonomer Elektrofahrzeuge,“Semester thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2021.
% ------------
% Input: vehLDS: The vehicle structure -> Stores the calculated component volumes and masses
%        nmax:   There is a given (input or calculated) maximal machine speed
%        Tmax:   There is a given (input or calculated) maximal machine torque
%        n_factor: There is a given scaling factor for the machine rotational speed
%        T_factor: There is a given scaling factor for the machine torque
% ------------
% Output: Rescaled machine characteristics
% ------------
%% Implementation:
% 1) Load local variables
% 2) Recalculate T_max and n_max 
% 3) Start load_engine to find new matching characteristic 

%% 1) Load variables
%Find which axles are filled: [front_axle, rear_axle]
if isnan(axles)
    axles = vehLDS.settings.filled_axles;
end
    
%% 2) Recalculate T_max and n_max 
for axle_id = find(axles)

if ~isnan(n_max) && n_max~= 0 %n rescale with given n_max 
    
    vehLDS.MOTOR{axle_id}.n_max = n_max;
        
elseif ~isnan(n_factor) && n_factor~= 0 %n rescale with rescale factor
    
    if ~isnan(vehLDS.MOTOR{axle_id}.n_max)
        vehLDS.MOTOR{axle_id}.n_max = vehLDS.MOTOR{axle_id}.n_max * n_factor;
    else
        vehLDS.MOTOR{axle_id}.n_max = max(vehLDS.MOTOR{axle_id}.characteristic(:,2))*n_factor;
    end        
end
    
if ~isnan(T_max) && T_max~= 0 %Torque rescale with given T_max 
    
    vehLDS.MOTOR{axle_id}.T_max = T_max;
        
elseif ~isnan(T_factor) && T_factor~= 0 %Torque rescale with given rescale factor
    
    if ~isnan(vehLDS.MOTOR{axle_id}.T_max) %T_max already available
        vehLDS.MOTOR{axle_id}.T_max = vehLDS.MOTOR{axle_id}.T_max * T_factor;
    else % no T_max available so far
        vehLDS.MOTOR{axle_id}.T_max = max(vehLDS.MOTOR{axle_id}.characteristic(:,1))*T_factor;
    end        
end
end

%% 3) Start load_engine to find new matching characteristic 
vehLDS = load_engine(vehLDS);
end
