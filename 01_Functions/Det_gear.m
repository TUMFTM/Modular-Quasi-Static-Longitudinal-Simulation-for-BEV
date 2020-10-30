function [T_opt, n_opt, eta_opt, P_el_opt,gear, distr] = Det_gear(T,n,eta,gear_forced,num_gears)
%% Description:
%This function finds the gear with the lowest energy consumption
% Author:   Korbinian Moller, TUM
% Date:     14.09.2020
%% Inputs
%  engine torque: T
%  engine rotational speed: n
%  engine eta: eta
%  forced gear
%% Outputs
%  best gear combination with corresponding T_opt,n_opt,eta_opt,P_el_opt,gear and relative gear distribution 
%% Sources
%  Alexander Koch, FTM, TUM
%% Implementation:

%calculate P_el 
P_el = T.*n*2*pi./(60 * (eta).^sign(T));

%if no gear is forced select best gear
if isnan(gear_forced)
    %find index (equal to gear) of best combination
    [~,gear]=min(P_el,[],2);
    %convert "gear" index to linear index    
    idx = sub2ind(size(P_el), (1:size(P_el,1))', gear);
    %create output
    P_el_opt = P_el(idx);
    T_opt = T(idx);
    n_opt = n(idx);
    eta_opt = eta(idx);
else
    %gear is fixed for all time steps
    gear = gear_forced * ones(length(P_el),1);
    
    %create output for required gear
    P_el_opt = P_el(:,gear_forced);
    T_opt = T(:,gear_forced);
    n_opt = n(:,gear_forced);
    eta_opt = eta(:,gear_forced);    
end

%gear distribution
distr = gear_distr(gear,num_gears);



end