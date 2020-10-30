function distr = gear_distr(gear,num_gears)
%% Description:
% This function calculates the relative use of each gear
% Author:   Korbinian Moller, TUM
% Date:     18.08.2020
%% Inputs
% gear vector
% number of gears at axle
%% Outputs
%  gear distribution
%
% first row: absolute amount of timesteps in every gear - sum of timesteps
% second row: relative amount of timesteps in every gear - 1
%
%% Sources
%% Implementation:

%gear distribution
distr_abs = zeros(1,num_gears);
for i = 1:num_gears
    distr_abs(1,i) = sum((gear == i));
end
distr_rel = distr_abs./length(gear);
distr = [distr_abs , sum(distr_abs);distr_rel , sum(distr_rel)];


end