function [gear_f, gear_r, distr_f, distr_r, distr_trq_f, distr_trq_r, distr_trq_rel] = eval_gear_all(num_gears_f,num_gears_r,num_comb,I_mot)
%% Description:
% This function "translates" the I_mot vector into gear and torque distribution
% Author:   Korbinian Moller, TUM
% Date:     19.08.2020
%% Inputs
% number of gears at front axle
% number of gears at rear axle
% number of possible combinations
% I_mot vector with column index of P_el_min
%% Outputs
% used gears at front axle
% used gears at rear axle
% gear distribution front
% gear distribution rear
% distribution of torque at front and rear axle
% table with 
%% Sources
%% Implementation:

%gear front
thrshld_f = 1:num_gears_r * 101:(num_comb+1) * 101;
gear_f = zeros(numel(I_mot),1);
for i = 1:(numel(thrshld_f)-1)
    gear_f(thrshld_f(i+1) > I_mot & I_mot >= thrshld_f(i)) = i;
end

%gear distribution front
distr_f = gear_distr(gear_f,num_gears_f);

%gear rear
thrshld_r = 1:101:(num_comb+1) * 101;
gear_r = zeros(numel(I_mot),1);
for i = 1:(numel(thrshld_r)-1)
        j = mod(i,num_gears_r);
        if j == 0
            j = num_gears_r;
        end
        gear_r(thrshld_r(i+1) > I_mot & I_mot >= thrshld_r(i)) = j;
end
%gear distribution rear
distr_r = gear_distr(gear_r,num_gears_r);

%distribution torque front - rear
%calculate distribution between front and rear axle
distr_trq_f = mod(I_mot,101);

%"translate" vector to vector with values from 0-100 percent
distr_trq_f(distr_trq_f == 0) = 101;
distr_trq_f = distr_trq_f - 1;

%create distribution vector for rear axle
distr_trq_r = 100-distr_trq_f;

% calculate number of total timesteps * 100% and calculate relative motor usage 
sum_step = sum(distr_trq_f + distr_trq_r); 
sum_f = sum(distr_trq_f);
sum_r = sum(distr_trq_r);

%relative usage of front/rear motor
distr_trq_rel = table;
distr_trq_rel.rel_front = sum_f/sum_step;
distr_trq_rel.rel_rear = sum_r/sum_step;
distr_trq_rel.rel_all = sum_step/sum_step;


end