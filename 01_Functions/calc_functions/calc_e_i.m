function veh=calc_e_i(veh,ParLDS)
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich), Korbinian Moller
%-------------
% Created on: 06.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: If the e_i is a User-Input, then the Input will be assigned for further
%              calculations. It is also possible to exactly calculate the
%              e_i, this however requires knowledge about the dimensions
%              and mass of rims, tires, gearbox components and electric
%              machines.
% ------------
% Sources: More information regarding the modeling of e_i is available here:
%          M. Tschochner, Comparative Assessment of Vehicle Powertrain Concepts in the Early Development Phase, Ph.D. Thesis, Institute of Automotive Technology, Technical University of Munich, Munich, Shaker, 2019, ISBN: 978-3-8440-6461-2.
%          
% ------------
% Input: vehicle: The vehicle structure -> Stores the calculated component volumes and masses
%        ParLDS: The Parameters structure -> Stores the constant values and regressions for volume and mass models
% ------------
% Output: The mass inertia factor reduced at the wheel center, e_i 
% ------------
%% Implementation: 
if ~isnan(veh.Input.e_i) %The User has assigned an e_i which should be used

    % repeat given e_i value - needed for multispeed transmission
    veh.LDS.parameters.e_i = veh.Input.e_i; 
    
else %The User did not assign an Input e_i
    
    % Retrieve e_i value from Parameters - needed for multispeed transmission
    veh.LDS.parameters.e_i = ParLDS.e_i; % Realistic value for BEVs
end
end