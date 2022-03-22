function veh=calc_e_i(veh,ParLDS)
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich), Korbinian Moller
%-------------
% Created on: 06.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: If the e_i is a User-Input, then the Input will be assigned for further
%              calculations, if not the Input will be calculated by using the reduced inertia from motor and inertia of wheels
% ------------
% Sources: More information regarding the implementation of the LDS functions is available at:
%          [1] K. Moller, „Validierung einer MATLAB Längsdynamiksimulation für die Auslegung von Elektrofahrzeugen,“ Bachelor thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2020. 
%          [2] K. Moller, „Antriebsstrangmodellierung zur Optimierung autonomer Elektrofahrzeuge,“Semester thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2021.
% ------------
% Input: vehicle: The vehicle structure -> Stores the calculated component volumes and masses
%        ParLDS: The Parameters structure -> Stores the constant values and regressions for volume and mass models
% ------------
% Output: The mass inertia factor reduced at the wheel center, e_i 
% ------------
%% Implementation: 
if ~isnan(veh.Input.e_i) %The User has assigned an e_i which should be used

    % repeat given e_i value - needed for multispeed transmission
    veh.LDS.parameters.e_i = veh.Input.e_i; 
    
else %The User did not assign an Input e_i
    
    %Here the are two options:
    %1) To exactly calculate the e_i, all the masses of the rotating
        %components need to be known. Therefore, the script must already have
        %finished an entire loop (including the mass calculation)
    
    %2) If this is the first loop of the script, the mass calculation has
        %not been completed yet. Therefore, the e_i cannot be calculated yet
        %and we have to use a typical value as starting point. 

    if isfield(veh.masses,'chassis') %Option 1) The mass of all rotating components is known

        %Reduced inertia front motors in kg m^2 
        if ~isempty(veh.LDS.MOTOR{1}) 
            J_red_f=veh.LDS.MOTOR{1}.J_M*veh.LDS.MOTOR{1}.quantity*veh.LDS.GEARBOX{1}.i_gearbox.^2;                     
        else
            J_red_f=0;  
        end

        %Reduced inertia rear motors in kg m^2
        if ~isempty(veh.LDS.MOTOR{2})
            J_red_r=veh.LDS.MOTOR{2}.J_M*veh.LDS.MOTOR{2}.quantity*veh.LDS.GEARBOX{2}.i_gearbox.^2;                      
        else
            J_red_r = 0;
        end

        J_gearbox=zeros(2,1);
        
        % singlespeed transmission
        for axle = 1:2
            if ~isempty(veh.gearbox{axle})
            gearbox = veh.gearbox{axle};

                if (strcmp(gearbox.Input.type, 'lay-shaft'))

                    gearbox = calc_J(gearbox);
                elseif (strcmp(gearbox.Input.type, 'planetary'))
                    % Calculation of moment of inertia in kg*mm^2
                    gearbox = calc_J_plan(gearbox);
                end
                
                J_gearbox(axle) = gearbox.results.J_tot_out*10^-6;
                
                %Check for errors
                if J_gearbox(axle)<0
                    veh = errorlog(veh,'Error in the calculation of the gearbox inertia. The values are not realistic and will be overwritten with typical values!');
                    J_gearbox(axle) = 0.168;%Typical value for a gearbox (calculated from reference vehicle BMW i3)
                end
            end
        end
        
        %mass of 4 wheels (calculated from the wheel modeling)
        m_wheels=veh.masses.chassis.tires_weight;               %All four tires in kg
        m_rims=veh.masses.chassis.rims_weight;                  %All four rims kg
        m_brakes=veh.masses.chassis.wheel_brakes_weight;        %All four brakes in kg
        r_wheel=veh.dimensions.CX.wheel_f_diameter/2000;        %in m
        r_rim=veh.dimensions.CX.rim_diameter_f_inch*25.4/2000;  %in m
        r_brake=veh.dimensions.CX.brake_disc_f_diameter/2000;   %in m

        %inertia of 4 tires ans 4 rims in kg m^2
        J_tires=0.5*m_wheels*(r_wheel^2+r_rim^2);  %Simplified as hollow cylinder with Ri=r_rim and Ro=R_tire
        J_rims=0.5*m_rims*(r_rim^2);               %Simplified as filled cylinder with R=Rrim
        J_brakes=0.5*m_brakes*(r_brake^2);         %Simplified as filled cylinder with R=rRbrake
        
        %addition of inertia wheels + front motor + rear motor to get total reduced inertia in kg m^2        
        J_red = J_red_f + J_red_r + J_tires + J_rims + J_brakes + sum(J_gearbox);  
        
        %rotating mass factor [-]
        eps = J_red/(veh.LDS.weights.vehicle_empty_weight_EU*(veh.LDS.wheel.r_dyn/1000)^2);

        %e_i value [-]
        veh.LDS.parameters.e_i=eps+1;                                                                              

        %Assign Calculated Outputs:
        veh.LDS.wheel.J_tires=J_tires;
        veh.LDS.wheel.J_rims=J_rims;
        veh.LDS.wheel.J_brakes=J_brakes;
        veh.LDS.wheel.m_wheels=m_wheels;

    else %if the tire's width is not given, a fixed e_i value from the fixparameters is assigned for every gear   

        % repeat given e_i value from Parameters - needed for multispeed transmission
        veh.LDS.parameters.e_i = ParLDS.e_i; 
    end
end
end