function [veh] = load_cycle(veh,Par)
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich), Korbinian Moller
%-------------
% Created on: 06.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: Load the selected cycle
% ------------
% Sources: More information regarding the implementation of the LDS functions is available at:
%          [2] K. Moller, „Validierung einer MATLAB Längsdynamiksimulation für die Auslegung von Elektrofahrzeugen,“ Bachelor thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2020. 
%          [3] K. Moller, „Antriebsstrangmodellierung zur Optimierung autonomer Elektrofahrzeuge,“Semester thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2021.
% ------------
% Input: veh: The vehicle structure -> Stores the calculated component volumes and masses
%        Par: The Parameters structure -> Stores the constant values and regressions for volume and mass models
% ------------
% Output: The vehicle structure updated with the required acceleration time
% ------------
%% Implementation:
%1) Load cycle
%2) Assign outputs

%% 1) Load cycle
%Load cycle variable:
dc = eval(['load(''',veh.Input.driving_cycle,'.mat''',')']);

%if no Slope is available create empty vector, needed for further checks
if ~isfield(dc,'S')
    dc.S = zeros(size(dc.v));
end

while dc.t(1) < 0 %timesteps <0 are cut off
    dc.t(1,:) = [];
    dc.v(1,:) = [];
    dc.S(1,:) = [];
end   

if dc.t(1) ~= 0 %cycle has to start at timestep 0                              
    dc.t = [0; dc.t];
    dc.v = [0;dc.v];
    dc.S = [0;dc.S];
end
    
%Assign the timestep in s
if isnan(Par.LDS.delta_t_con) 
    delta_t = [dc.t(2)-dc.t(1); diff(dc.t)]; %no delta_t input available
else
    delta_t = Par.LDS.delta_t_con;
end

%create t vector
t = (0:delta_t:dc.t(end))';

%Assign the cycle speed in m/s
v = dc.v/3.6;

%create Interpolation for speed
F_v = griddedInterpolant(dc.t,v,'linear','none');

%Assign interpolated cycle speed in m/s
v = F_v(t);
                    
%Assign the cycle acceleration in  m/s²
a = [0; diff(v)]./delta_t;  

%Assign slope S. Check if a vector S containing the elevation in % for each timestep is assigned
if isfield(dc,'S')
    alpha=dc.S;
    F_S = griddedInterpolant(dc.t,alpha,'linear','none');
    alpha = F_S(t);
else
    %No vector S in MATLAB cycle variable, the elevation will be taken as 0% for all steps
    alpha   = zeros(size(v));                                       
end  

%% 2) Assign output
%As the cycle has already been loaded, the values are saved, so, that it does not have to be loaded again
veh.LDS.sim_cons.t          = t;                                            %time vector in s
veh.LDS.sim_cons.delta_t    = delta_t;                                      %time step in s
veh.LDS.sim_cons.v          = v;                                            %speed vector in m/s
veh.LDS.sim_cons.a          = a;                                            %acceleration vector in m/s^2 
veh.LDS.sim_cons.alpha      = alpha;                                        %slope vector in %     
end