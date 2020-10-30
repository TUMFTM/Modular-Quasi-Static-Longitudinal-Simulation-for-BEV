function  [veh]=check_overload(veh,Par,T_mot,n_mot,ax_id)
%% Description:
%This function identifies the timesteps, where the motor is in overload.
%This function also calculates the maximum time, where the motor is in
%overload and compares it with the motor overload duration.

%Authors: Lorenzo Nicoletti
%date:   31.12.2019

%% Inputs:
%veh: vehicle structure
%Par: parameter structure
%T_mot: Torque at the motor (in Nm)
%n_mot: rotational speeed (in 1/min) at the motor
%ax_id: Axle identifier (1 for front and 2 for rear)
%% Outputs:
%vehicle struct with assigned errors, in case overload time is exceeded.
%% Implementation
%1) Calculate the highest overload duration
%2) Create the vector to plot the  overload duration
%3) Assign Output

%% 1) Calculate the highest overload duration 
%Assign overload factor and duration from Par
overload_factor     = Par.LDS.overload_factor.(veh.LDS.MOTOR{ax_id}.type); 
overload_duration     = Par.LDS.overload_duration.(veh.LDS.MOTOR{ax_id}.type);

%Max amount of timestep where the motor can be in overload
max_timesteps    = overload_duration/veh.LDS.sim_cons.delta_t; 

%Interpolate to find the characteristic line of the motor when not overloaded
F = griddedInterpolant(veh.LDS.MOTOR{ax_id}.characteristic(:,2),veh.LDS.MOTOR{ax_id}.characteristic(:,1)/overload_factor,'linear','none');

%Calculate maximum torque in the no-overload zone
T_max=F(n_mot);

%If the required torque is bigger than T_max, the motor is in overload
Isoverload=T_mot > T_max;

%Vector which resume how many zeros and ones are contained and in which sequence
overload_seq = diff([0; find(diff(Isoverload)); numel(Isoverload)]);

%Check if the first timestep is an overload or non-overload point and initialize the overload vector with ones or zeros
if Isoverload(1)==0

     overload_id=2:2:length(overload_seq); 
     overload_vec=zeros(overload_seq(1),1);

else

     overload_id=1:2:length(overload_seq);
     overload_vec=ones(overload_seq(1),1);

end

%Find maximum amount of timesteps where the motor is continuosly in overload
max_time=max(overload_seq(overload_id));


if max_time>max_timesteps %Check if the motor does not exceed the allowed verload duration
    
    %The motor has been in overload for an amount of time, which is higher than the overload duration
    if ax_id == 1
    veh.LDS=errorlog(veh.LDS,'The motor at the front axle is in overload for an amount of time, which is higher than the permitted overload duration');
    else
    veh.LDS=errorlog(veh.LDS,'The motor at the rear axle is in overload for an amount of time, which is higher than the permitted overload duration');    
    end
    
else %Check more precisely that the motor does not exceed overload point, see explanation section 2)

%% 2) Create the vector to plot the  overload duration
%While the motor is in overload, it gradually gets warmer. For this reason,
%if the motor stays for 30s in overload (30s being the max overload duration) and
%then one second in non overload, it does not mean that then it can stay
%other 30s in overload, as that one second is not sufficient to cool the motor down.
%Here the authors suppose that every second of
%"heating", i.e. overload, can be compensated by one second of cooling,
%i.e. non overload. This vector is needed to evaluate this characteristic:
%At every overload point a +1 is assigned, at every non overload point a -1 is
%assigned, then the cumulative sum is created, while the total sum cannot go
%under 0.

for ii=overload_id

    overload_vec=[overload_vec;ones(overload_seq(ii),1)]; 
    
    cum_overload=cumsum(overload_vec);
    
    overload_is=cum_overload(end);
    
    if overload_is-overload_seq(ii+1)>0

        overload_vec=[overload_vec;ones(overload_seq(ii+1),1)*(-1)];

    else

        overload_vec=[overload_vec;(-1)*ones(overload_is,1);zeros(overload_seq(ii+1)-overload_is,1)];

    end
    
end

%Cumsum of the overload vec
overload_vec=cumsum(overload_vec);

%Maximum time, taking into account the required cooling time!
max_time=max(overload_vec);

if max_time>max_timesteps %Check if the motor does not exceed the allowed verload duration
    
    %The motor has been in overload for an amount of time, which is higher than the overload duration
    if ax_id == 1
    veh.LDS=errorlog(veh.LDS,'The motor at the front axle is in overload for an amount of time, which is higher than the permitted overload duration');
    else
    veh.LDS=errorlog(veh.LDS,'The motor at the rear axle is in overload for an amount of time, which is higher than the permitted overload duration');    
    end

end

%% 3) Assign Outputs:

if ax_id==1
    str_time='max_overload_time_in_cycle_mot_f';
    str_vec ='overload_vector_mot_f';
else
    str_time='max_overload_time_in_cycle_mot_r';
    str_vec ='overload_vector_mot_r';
end

veh.LDS.sim_cons.(str_time)=max_time;       %in s
veh.LDS.sim_cons.(str_vec)=overload_vec;    %in [-]
   
end

