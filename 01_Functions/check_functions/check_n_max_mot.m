function vehLDS = check_n_max_mot(vehLDS,axle_id)
%% Description:
%If n_max is an input, this means that it will not be calculated from the
%maximal speed of the vehicle. This function checks, that the given n_max
%is sufficient to reach the maximum speed and the cycle maximum speed.
%This check is needed only, when n_max is an input.

%author:    Nicoletti Lorenzo
%date:      29.12.2019
%% Inputs:
% vehicle struct
% axle id: specifies the considered machine axle
%% Outputs:
% vehicle struct
%% Implementation

%Check cycle max speed in m/s:
v_sim_cons=max(vehLDS.sim_cons.v);

%Check input max speed in m/s
v_sim_acc =vehLDS.sim_acc.max_speed/3.6;

if v_sim_acc>v_sim_cons %The maximum speed is taken as reference for n_max, as it is much bigger than v_cycle
    
    %input max speed used for n_max_calculation: the motor has to be able to reach the maximum vehicle speed.
    n_wheels_max=(v_sim_acc/(vehLDS.wheel.r_dyn/1000))*30/pi; %max speed at the wheels in 1/min
    
    %Required motor rotational speed in 1/min
    n_required_max = n_wheels_max*vehLDS.GEARBOX{axle_id}.i_gearbox(1);
    
else %The max speed input was smaller than cycle max speed. The function check_max_speed has already overwritten it with the cycle maximum speed.
    
    %cycle max speed used for n_max_calculation: the motor has to be able to reach the maximum cycle speed.
    n_wheels_max  = v_sim_cons/(vehLDS.wheel.r_dyn/1000)*60/2/pi; %max speed at the wheels

    %Required motor rotational speed in 1/min
    n_required_max = n_wheels_max*vehLDS.GEARBOX{axle_id}.i_gearbox(1);
    
end

%If the required rotational speed is higher than the motor rotational speed, latter will be overwritten.
if round(n_required_max)>round(vehLDS.MOTOR{axle_id}.n_max)
    
    vehLDS.MOTOR{axle_id}.n_max=n_required_max;
    
    if axle_id==1
        
        vehLDS=errorlog(vehLDS, 'The given n_max for the motor/motors at the front axle is too low wether for the given max speed or the choosen cycle max speed. n_max will be scaled');
    
    else
        
       vehLDS=errorlog(vehLDS, 'The given n_max for the motor/motors at the rear axle is too low wether for the given max speed or the choosen cycle max speed. n_max will be scaled');

    end
    
end

end

