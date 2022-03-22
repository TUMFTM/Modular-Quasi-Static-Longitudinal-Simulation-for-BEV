function vehicle=calc_c_r(vehicle,Par)
% Designed by: Korbinian Moller
%-------------
% Created on: 06.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function calculates the c_r value from Inputparameters
% ------------
% Sources: More information regarding the implementation of the LDS functions is available at:
%          [1] K. Moller, „Antriebsstrangmodellierung zur Optimierung autonomer Elektrofahrzeuge,“Semester thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2021.
% ------------
% Input: veh: The vehicle structure -> Stores the calculated component volumes and masses
%        Par: The Parameters structure -> Stores the constant values and regressions for volume and mass models
% ------------
% Output: vehicle struct with the c_r
% ------------
%% Implementation
if ~isempty(vehicle.Input.c_r) && ~isnan(vehicle.Input.c_r) %User has given a c_r -> Use the User Input
    
    vehicle.LDS.parameters.c_r = vehicle.Input.c_r;

elseif ~isempty(vehicle.LDS.wheel.r_tire) && ~isempty(vehicle.LDS.wheel.w_tire) %The tire are calculated and there is no user input -> Estimate c_r from tire size

    width  = vehicle.LDS.wheel.w_tire;
    radius = vehicle.LDS.wheel.r_tire;
    
    if width > 270
        width = 270;
        vehicle = errorlog(vehicle,'Tire Width is out of boundary and is set to 270 mm (upper bound) for rolling resistance coefficient calculation. Results could be distorted!');
        
    elseif width < 125
        width = 125;
        vehicle = errorlog(vehicle,'Tire Width is out of boundary and is set to 125 mm (lower bound) for rolling resistance coefficient calculation. Results could be distorted!');
    end
    
    if radius > 420
        radius = 420;
        vehicle = errorlog(vehicle,'Tire Radius is out of boundary and is set to 420 mm (upper bound) for rolling resistance coefficient calculation. Results could be distorted!');
        
    elseif radius < 250
        radius = 250;
        vehicle = errorlog(vehicle,'Tire Radius is out of boundary and is set to 250 mm (lower bound) for rolling resistance coefficient calculation. Results could be distorted!');
    end
    
    %Equation taken from [1] in kg/ton
    vehicle.LDS.parameters.c_r = 0.034565063636242926+(width)*(1.72623336e-04)+(radius)*(-2.25929942e-04)+(width^2)*(-3.21249605e-07)+(radius^2)*(2.59714227e-07);
   
else   %There is no user Input but the tire have not been calculated -> Use constant value (realstic value)
    
    vehicle.LDS.parameters.c_r = Par.LDS.c_r; 
end
end