function veh=calc_e_i(veh,ParLDS)
%% Description:
%If the e_i is a User-Input, then the Input will be assigned for further
%calculations, if not the Input will be calculated by using the reduced inertia from motor and inertia of wheels
%Multi gear:
%if more than one gear is available, e_i for all posssible gear
%combinations will be calculated
%for example:  # gear front:   2 gears - ratio [9.34, 7]
%              # gear rear:    3 gears - ratio [9.73, 5, 3]
%               
%              notation: gear_f,gear_r --> 1,2 means gear 1 at front axle, gear 2 at rear axle
%              e_i is a vector with e_i from gear combination [1,1 ; 1,2 ; 1,3 ; 2,1 ; 2,2 ; 2,3]
%
%author:    Lorenzo Nicoletti, Ruben Hefele, Korbinian Moller, FTM, TUM 
%date:      18.08.2020
%% Inputs:
%inertia rear motor (if used)
%inertia front motor (if used)
%inertia of wheels
%% Outputs:
% e_i 
%% Sources: 
%formula for reduced inertia and eps out of Haken - Grundlagen der
%Kraftfahrzeugtechnik
%formula for mass and inertia of tires: Fuchs - Gewichts- und Kostenmodell
%% Implementation: 

%if there's no input for  the e_i_value it will be calculated. This is
%only possible when width of tire is given
if ~isempty(veh.LDS.GEARBOX{1})
    num_gears_f = veh.LDS.GEARBOX{1}.num_gears;
else 
    num_gears_f = 1;
end
if ~isempty(veh.LDS.GEARBOX{2})
    num_gears_r = veh.LDS.GEARBOX{2}.num_gears;
else
    num_gears_r = 1;
end

if ~isnan(veh.Input.e_i)

    %Use given Input
    veh.LDS.parameters.e_i=veh.Input.e_i;
    if ~isempty(veh.LDS.MOTOR{1})
        veh.LDS.parameters.e_i = repmat(veh.LDS.parameters.e_i,1,veh.LDS.GEARBOX{1}.num_gears);
    else
        veh.LDS.parameters.e_i = repmat(veh.LDS.parameters.e_i,1,veh.LDS.GEARBOX{2}.num_gears);
    end
    
else

    if ~isnan(veh.LDS.wheel.w_tire)

        %reduced inertia front motors in kg m^2 
        if ~isempty(veh.LDS.MOTOR{1}) 
            J_red_f=veh.LDS.MOTOR{1}.J_M*veh.LDS.MOTOR{1}.quantity*veh.LDS.GEARBOX{1}.i_gearbox.^2;                     
        else
            J_red_f=0;  
        end

        %reduced inertia rear motors in kg m^2
        if ~isempty(veh.LDS.MOTOR{2})
            J_red_r=veh.LDS.MOTOR{2}.J_M*veh.LDS.MOTOR{2}.quantity*veh.LDS.GEARBOX{2}.i_gearbox.^2;                      
        else
            J_red_r=0;
        end

        %mass of 4 wheels from Fuchs - Gewichts- und Kostenmodell in kg
        m_wheel= ParLDS.m_wheel_P9*veh.LDS.wheel.r_tire*2/1000*veh.LDS.wheel.w_tire/1000+ParLDS.m_wheel_P10;        

        %inertia of 4 wheels in kg m^2
        J_wheels=m_wheel*(veh.LDS.wheel.r_tire/1000)^2*(0.8+0.1);                                     

        %addition of inertia wheels + front motor + rear motor to get total reduced inertia in kg m^2
        J_red = zeros(1,num_gears_f * num_gears_r);
        k = 1;
        for i = 1:num_gears_f
            for j = 1:num_gears_r
                J_red(k) = (J_wheels)+J_red_f(i)+J_red_r(j);  
                k = k + 1;
            end
        end

        %rotating mass factor [-]
        eps = J_red/(veh.LDS.weights.vehicle_empty_weight_EU*(veh.LDS.wheel.r_dyn/1000)^2);

        %e_i value [-]
        veh.LDS.parameters.e_i=eps+1;                                                                              

        %Assign Calculated Outputs:
        veh.LDS.wheel.J_wheels=J_wheels;
        veh.LDS.wheel.m_wheel=m_wheel;

    else %if the tire's width is not given, a fixed e_i value from the fixparameters is assigned for every gear   

        veh.LDS.parameters.e_i = ones(1,num_gears_f * num_gears_r) * ParLDS.e_i;                                                                       

    end
    
end

end

