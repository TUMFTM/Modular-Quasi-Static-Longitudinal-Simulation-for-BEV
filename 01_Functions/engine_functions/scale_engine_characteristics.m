function [vehLDS] = scale_engine_characteristics(vehLDS)
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich), Korbinian Moller
%-------------
% Created on: 06.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This fucntion scales motor diagram and motor characteristics according to T_max an n_max
% ------------
% Sources: More information regarding the implementation of the LDS functions is available at:
%          [1] K. Moller, „Validierung einer MATLAB Längsdynamiksimulation für die Auslegung von Elektrofahrzeugen,“ Bachelor thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2020. 
%          [2] K. Moller, „Antriebsstrangmodellierung zur Optimierung autonomer Elektrofahrzeuge,“Semester thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2021.
% ------------
% Input: vehLDS: The vehicle structure -> Stores the calculated component volumes and masses
% ------------
% Output: -characteristic torque over rotational speed
%         -characteristic diagram with efficiencies
% ------------
%% Implementation (For each motor): 
%1) Load the motor characteristic and scale it with the known value
%2) Load motor efficiency map and select only the first quadrant

%Find which axles are filled: [front_axle, rear_axle]
filled_axles = vehLDS.settings.filled_axles;

for i=find(filled_axles)
    %% 1) Load the motor characteristic and scale it with the known value
    %Scale torque if T_max is an input
    if ~isnan(vehLDS.MOTOR{i}.T_max)
        
        % huge scale factor
        if  abs(vehLDS.MOTOR{i}.T_max - max(vehLDS.MOTOR{i}.characteristic(:,1))) > 50
            
            if i == 1 %front axle
                vehLDS = errorlog(vehLDS,'Front Motor: Torque is scaled more than 50Nm!');
            elseif i == 2 %rear axle
                vehLDS = errorlog(vehLDS,'Rear Motor: Torque is scaled more than 50Nm!');    
            end
            
        end
        
        vehLDS.MOTOR{i}.characteristic(:,1)=vehLDS.MOTOR{i}.T_max/max(vehLDS.MOTOR{i}.characteristic(:,1))*vehLDS.MOTOR{i}.characteristic(:,1);     %torque in Nm

    end
    
    
    %At this point, independently from the case (1-6) the motor max speed is known. It has to be compared with the cycle max speed!
    %vehLDS = check_n_max_mot(vehLDS,i);
    
    %check how much rotational speed is scaled
    if abs(vehLDS.MOTOR{i}.n_max - max(vehLDS.MOTOR{i}.characteristic(:,2))) > 1500
        if i == 1 %front axle
            vehLDS = errorlog(vehLDS,'Front motor: Rotational speed is scaled more than 1500 1/min!');
        elseif i == 2 %rear axle
            vehLDS = errorlog(vehLDS,'Rear motor: Rotational speed is scaled more than 1500 1/min!');
        end
    end
    
    %Scale n_max (in 1/min) accordingly:
    vehLDS.MOTOR{i}.characteristic(:,2)=vehLDS.MOTOR{i}.n_max/max(vehLDS.MOTOR{i}.characteristic(:,2))*vehLDS.MOTOR{i}.characteristic(:,2);
    
    %Calculate resulting mechanical motor power in kW (if the torque is unkonwn the power will be calculated in calc_torque)
    vehLDS.MOTOR{i}.P_max_mech = max(vehLDS.MOTOR{i}.characteristic(:,2).* vehLDS.MOTOR{i}.characteristic(:,1)*pi/30)*(10^-3);

    %% 2) Load motor efficiency map and select only the first quadrant
    %Scale rotational speed of map in 1/min
    vehLDS.MOTOR{i}.diagram.n_scaled = vehLDS.MOTOR{i}.diagram.n_unscaled*vehLDS.MOTOR{i}.n_max/max(vehLDS.MOTOR{i}.diagram.n_unscaled);    

    %If the T_max is given (configuration 1,3 and 5) it is already possible to scale the vectors: 
    vehLDS.MOTOR{i}.diagram.T_scaled = vehLDS.MOTOR{i}.diagram.T_unscaled*vehLDS.MOTOR{i}.T_max/max(vehLDS.MOTOR{i}.diagram.T_unscaled);     
    %If the T_max is not given the vector T_scaled will be overwritten in further steps
end
end