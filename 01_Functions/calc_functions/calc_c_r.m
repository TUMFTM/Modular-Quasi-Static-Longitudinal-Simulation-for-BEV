function v=calc_c_r(v,Par)
%% Description:
%This function choses the c_r value from Inputparameters or Fixparameters
%Author:    Ruben Hefele, FTM, TUM
%Date:      01.09.2019

%% Input:
%vehicle.Input
%Parameters
%% Output:
%vehicle.LDS
%% Sources: No sources for this function
%% Implementation

%if there's an input for c_r value, this will be taken
if ~isempty(v.Input.c_r) && ~isnan(v.Input.c_r)
    
    v.LDS.parameters.c_r=v.Input.c_r;
  
% the c_r value of FixParameter will be chosen    
else
    
    v.LDS.parameters.c_r=Par.LDS.c_r;
    
end

end
