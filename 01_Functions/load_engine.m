function [vehLDS] = load_engine(vehLDS)
%% Description
% finds matching characteristic, scales it properly

%author:    Lorenzo Nicoletti, Korbinian Moller, FTM, TUM
%date:      27.05.2020
%% Input: 
%   v:    struct with the vehicle parameters
%   Par:  struct with fixed parameters
%% Output:
%   v:    updated struct with the vehicle parameters
%% Sources: 
%   --
%% Implementation:

filled_axles = vehLDS.settings.filled_axles;

for i=find(filled_axles)
    vehLDS = check_n_max_mot(vehLDS,i); %required n_max for maximum speed
end

%% Load the motor characteristics:
%vehLDS = load_engine_characteristics_old(vehLDS);
vehLDS = load_engine_characteristic(vehLDS);

%% Scale the motor characteristics:
vehLDS = scale_engine_characteristics(vehLDS);  

end