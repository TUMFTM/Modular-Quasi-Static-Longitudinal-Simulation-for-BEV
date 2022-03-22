function [vehLDS] = load_engine(vehLDS)
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich), Korbinian Moller
%-------------
% Created on: 06.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: Finds matching machine characteristic and scales it properly
% ------------
% Sources: More information regarding the implementation of the LDS functions is available at:
%          [1] K. Moller, „Validierung einer MATLAB Längsdynamiksimulation für die Auslegung von Elektrofahrzeugen,“ Bachelor thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2020. 
%          [2] K. Moller, „Antriebsstrangmodellierung zur Optimierung autonomer Elektrofahrzeuge,“Semester thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2021.
% ------------
% Input: veh: The vehicle structure -> Stores the calculated component volumes and masses
%        Par: The Parameters structure -> Stores the constant values and regressions for volume and mass models
% ------------
% Output: vehicle struct with machine characteristics
% ------------
%% Implementation:
filled_axles = vehLDS.settings.filled_axles;

%Check if the n_max of the given machines is sufficient to reach the maximum speed
for i=find(filled_axles)
    vehLDS = check_n_max_mot(vehLDS,i); %required n_max for maximum speed
end

% Load the motor characteristics:
vehLDS = load_engine_characteristic(vehLDS);

% Scale the motor characteristics:
vehLDS = scale_engine_characteristics(vehLDS);  
end