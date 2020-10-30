function v=calc_c_d(v,Par)
%% Description:
%This function choses the drag coefficient value from Inputparameters or Fixparameters
%Author:    Ruben Hefele, FTM, TUM
%Date:      01.09.2019

%% Input:
%vehicle.Input
%Parameters
%% Output:
%vehicle.LDS
%% Sources: No sources for this function
%% Implementation

%if c_d is an input, this value will be chosen
if ~isnan(v.Input.c_d) && ~isempty(v.Input.c_d)
    
    v.LDS.parameters.c_d=v.Input.c_d;

%if c_d is no input, the value from Fixparameters will be chosen
else
    
    v.LDS.parameters.c_d=Par.LDS.c_d;
    
end

end