function v=check_LDS_inputs(v)
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich), Korbinian Moller
%-------------
% Created on: 01.03.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function checks the values
% ------------
% Input: v: The vehicle structure -> Stores the calculated component volumes and masses
% ------------
% Output: %The vehicle updated with the errors occurring during the simulation
% ------------
%% Implementation:
if v.LDS.First_Run==1
    %Check the value of the drag coefficient (if assigned)
    if v.Input.c_d > 0.4 || v.Input.c_d < 0.15 
        v=errorlog(v,'THE ASSIGNED DRAG COEFFICIENT IS UNREALISIC! VALUES SHOULD BE BETWEEN 0.2-0.35',1);
    end

    %Check the value of the rolling coefficient (if assigned)
    if v.Input.c_r > 0.015 && v.Input.c_r<0.004 
        v=errorlog(v,'THE ASSIGNED ROLLING COEFFICIENT IS UNREALISIC. VALUES SHOULD BE BETWEEN 0.006-0.0013',1);
    end

    %Check the value of the rolling coefficient (if assigned)
    if v.Input.e_i < 1 && v.Input.e_i > 1.3 
        v=errorlog(v,'THE ASSIGNED e_i FACTOR IS UNREALISTIC: VALUES SHOULD BE BETWEEN 1.02-1.2',1);
    end
end
end