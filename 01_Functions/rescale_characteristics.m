function [vehLDS] = rescale_characteristics(vehLDS,n_max,n_factor,T_max,T_factor,axles) 
%% Description
% adapts machine operating map to fulfill v_max requirement
% if no axles are given, all available axles will be scaled

%author:    Lorenzo Nicoletti, Korbinian Moller, FTM, TUM
%date:      29.05.2020
%% Input: 
%   vehLDS: struct with the vehicle LDS parameters
% 
%% Output:
%   vehLDS: updated characteristics
%% Sources: 
%   --
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
