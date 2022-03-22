function [check_torque_output,veh] = check_torque_sim_cons(veh)
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich), Korbinian Moller
%-------------
% Created on: 06.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: For the sim_acc the torque is automatically scaled according to the
%              required acceleration time or the max simulation time. In the case of the
%              consumption simulation it has to be checked wheter the cycle is doable for
%              the motor or not. If not, the motor torque has to be scaled.
%              Regarding the rotational speed, the fuction check_n_max makes sure, that
%              the motor has the required rotational speed for the cycle, i.e. only the
%              torque has to be checked in the fucntion check_torque_sim_cons.
% ------------
% Sources: More information regarding the implementation of the LDS functions is available at:
%          [1] K. Moller, „Validierung einer MATLAB Längsdynamiksimulation für die Auslegung von Elektrofahrzeugen,“ Bachelor thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2020. 
%          [2] K. Moller, „Antriebsstrangmodellierung zur Optimierung autonomer Elektrofahrzeuge,“Semester thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2021.
% ------------
% Input: vehicle: The vehicle structure -> Stores the calculated component volumes and masses
%        Par: The Parameters structure -> Stores the constant values and regressions for volume and mass models
% ------------
% Output: vehicle struct with scaled torque (if a scaling is required)
% ------------
%% Implementation:
%Find which axles are filled: [front_axle, rear_axle]
filled_axles = veh.LDS.settings.filled_axles;

%Torque optimization on or/off (1/NaN) - only for AWD
torque_optimizer = veh.LDS.settings.torque_optimizer;

for i=find(filled_axles)  
    if i==1
        %torque and speed of a single motor (calculated in calc_power)
        n_mot=veh.LDS.sim_cons.n_mot_f;
        T_mot=veh.LDS.sim_cons.T_mot_f;
    else
        
        %torque and speed of a single motor (calculated in calc_power)
        n_mot=veh.LDS.sim_cons.n_mot_r;
        T_mot=veh.LDS.sim_cons.T_mot_r;
    end
    
    %Mechanical Power needed for the motor:
    P_mech_required=abs((n_mot*pi/30).*T_mot*10^-3);

    %Positions where mech required power is above motor power:
    [I]=find(P_mech_required > veh.LDS.MOTOR{i}.P_max_mech);
    
    if ~isempty(I) %There are some points, where the required mechanical power is higher than the motor power

        %Interpolate the motor characteristics (will be needed later):
        F = griddedInterpolant(veh.LDS.MOTOR{i}.characteristic(:,2),veh.LDS.MOTOR{i}.characteristic(:,1),'linear','none');

        %Power at the points where the cycle power is above the motor power (critical points)
        P_mech_critic=P_mech_required(I);

        %Torque required at the critical points:
        T_required=(P_mech_critic*10^3)./(n_mot(I)*pi/30);

        %Maximum torque of the motor at the rotational speed of the critical points:
        T_motor_is=F(n_mot(I));

        %Find the point with the highest torque difference, this point will be used for scaling:
        [~,II]= max( T_required-T_motor_is);
        T_scaling=T_required(II);

        %Calculate the needed scaling factor:
        scale_factor=T_scaling/T_motor_is(II);
        
        %create scale axle vector if optimization is turned on
        if ~isnan(torque_optimizer)
            axle  = zeros(1,2);
            axle(i) = 1;
            axle = logical(axle);
        else
            axle = filled_axles;
        end
        
        %Scale motor diagramm and characteristics, according to the new T_max motor
        veh.LDS = rescale_characteristics(veh.LDS,NaN,NaN,NaN,scale_factor,axle); %(vehLDS,n_max,n_factor,T_max,T_factor)
        
        %Motor has been scaled, check in variable check_torque
        check_torque(i)=0;
        
    else %The motor has enough power for the cycle
        
        %Motor has not been scaled, check in the variable check_torque
        check_torque(i)=1;
        
    end
end

if all(check_torque(find(filled_axles)))
    
    %None of the motor has been scaled, no further simulation is required
    check_torque_output=1;  
else
    
    %At least one motor has been scaled, the energy consumption and the
    %acceleration simulation have to be evaluated again
    check_torque_output=0;   
end
end