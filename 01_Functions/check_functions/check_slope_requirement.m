function[v]=check_slope_requirement(v,Par)
%% Description:
%this function checks if the vehcle meets the slope requirement; if not there will be an error in the ErrorLog struct 
%slope requirement is calculated while driving uphill with or without trailer, slope value is given in Fixparameter

%Author:    Lorenzo Nicoletti, Ruben Hefele, FTM, TUM
%date:      18.12.19
%% Inputs:
%vehicle struct
%Parameters struct
%% Outputs:
%ErrorLog
%% Sources:
% § 42 -StVZO
% derivations for traction limit in 'Implementation of a MATLAB
% longitudinal dynamics simulation for electric vehicles' - Hefele, chapter 3.6

%% Implementation:
%1) Calculate vehicle mass
%2) Calculate needed traction considering only slope and wheel resistance:
%3) Calculate traction limit and check traction limit
%4) Check if vehicle has enough Torque for the uphill scenarios
%5) Assign Outputs

%% 1) Calculate possible trailer mass
%calculate trailer mass trailer factor is defined from ADAC-database, see appendix in thesis Hefele

%Assign local inputs to make the equation easier to visualize
max_weight_without_trailer=v.masses.vehicle_max_weight;                                            %max vehicle weight in kg
trailer_factor=Par.LDS.trailer_factor.(v.Input.ground_clearance);                                  %Trailer factor [-]
weight_trailer   = max_weight_without_trailer *trailer_factor;                                     %Trailer max in kg   
c_r=v.LDS.parameters.c_r;                                                                          %c_R value of the wheels [-]
h_COG=v.LDS.parameters.height_COG;                                                                 %height COG in mm
wb=v.LDS.parameters.wheelbase;                                                                     %wheelbase in mm
g=Par.LDS.g;                                                                                       %gravitational acceleration in m/s^2
mu=Par.LDS.mue_max;                                                                                %driving traction coefficient

%check if maximum authorised trailer mass for passenger cars is surpassed
if weight_trailer > 3500 

    %maximum authorised trailer mass for passenger cars in kg (from StVZO)
    weight_trailer=3500;  
    
end


%% 2) Calculate needed traction considering only slope and wheel resistance:

%air resistance is set to zero for this test 
F_L=0; 

%slopes are if not changed sl_1: 25%, sl_2: 0%, sl_3: 12%; sl_2 is reserved for other slopes, which can be given in fixparameters
sl_1=Par.LDS.slope_1.(v.Input.ground_clearance);
sl_2=Par.LDS.slope_2.(v.Input.ground_clearance);
sl_3=Par.LDS.slope_3.(v.Input.ground_clearance);

%calculate rebound force according to slope cases
F_reb_sl_1 = (c_r*abs(cos(atan(sl_1))) + (abs(sin(atan(sl_1)))))*max_weight_without_trailer*g;                            %slope: sl_1, no trailer, authorised vehicle mass
F_reb_sl_2 = (c_r*abs(cos(atan(sl_2)))+ (abs(sin(atan(sl_2)))))*max_weight_without_trailer*g;                             %slope: sl_2, no trailer, authorised vehicle mass
F_reb_sl_3 = (c_r*abs(cos(atan(sl_3)))+ (abs(sin(atan(sl_3)))))*(max_weight_without_trailer+weight_trailer)*g;           %slope: sl_3, max. trailer

%% 3) Calculate traction limit and check traction limit

%A negative traction limit means that the vehicle has no traction, and is therefore unable to pass the ramp
switch v.LDS.settings.drive
    case 'front_wheel'
        
        %Calculate max Moment (in Nm) at the wheels (simplified). Will be needed later
        T_wheels=v.LDS.MOTOR{1}.T_max.*v.LDS.GEARBOX{1}.i_gearbox(1).*v.LDS.GEARBOX{1}.eta(1).*v.LDS.MOTOR{1}.quantity;

        %a_max for sl_1 in m/s^2
        a_max_sl_1 = (g*(mu*(Par.LDS.axle_load_rear*cos(tan(atan(sl_1)))-(h_COG/wb)*sin(tan(atan(sl_1))))-c_r*(Par.LDS.axle_load_front*cos(tan(atan(sl_1)))+(h_COG/wb)*sin(tan(atan(sl_1))))-sin(tan(atan(sl_1))))-(F_L/max_weight_without_trailer))/1+(h_COG/wb)*(mu+c_r);

        %a_max for sl_2 in m/s^2
        a_max_sl_2 = (g*(mu*(Par.LDS.axle_load_rear*cos(tan(atan(sl_2)))-(h_COG/wb)*sin(tan(atan(sl_2))))-c_r*(Par.LDS.axle_load_front*cos(tan(atan(sl_2)))+(h_COG/wb)*sin(tan(atan(sl_2))))-sin(tan(atan(sl_2))))-(F_L/max_weight_without_trailer))/1+(h_COG/wb)*(mu+c_r); 

        %a_max for sl_3 in m/s^2
        %x and y are auxiliary variables for shortening term and making code clearer
        x=Par.LDS.axle_load_rear*cos(atan(sl_3))-(h_COG/wb)*sin(atan(sl_3));
        y=Par.LDS.axle_load_front*cos(atan(sl_3))+(h_COG/wb)*sin(atan(sl_3));

        %ratios are defined for making code clearer
        m_ratio_auth  = max_weight_without_trailer/(max_weight_without_trailer+weight_trailer);
        m_ratio_trail = weight_trailer/(max_weight_without_trailer+weight_trailer);

        %a_max for sl_3 in m/s^2
        a_max_sl_3 = (g*(m_ratio_auth*(mu*x-c_r*y)-m_ratio_trail*cos(atan(sl_3))-sin(atan(sl_3))))/(1+m_ratio_auth*(h_COG/wb)*(mu+c_r));

    case 'rear_wheel'
        
        %Calculate max Moment (in Nm) at the wheels (simplified). Will be needed later
        T_wheels=v.LDS.MOTOR{2}.T_max.*v.LDS.GEARBOX{2}.i_gearbox(1).*v.LDS.GEARBOX{2}.eta(1).*v.LDS.MOTOR{2}.quantity;
        
        %a_max for sl_1 in m/s^2
        a_max_sl_1 = (g*(mu*(Par.LDS.axle_load_front*cos(atan(sl_1))+(h_COG/wb)*sin(atan(sl_1)))-c_r*(Par.LDS.axle_load_rear*cos(atan(sl_1))-(h_COG/wb)*sin(atan(sl_1)))-sin(atan(sl_1)))-(F_L/max_weight_without_trailer))/1-(h_COG/wb)*(mu+c_r); 

        %a_max for sl_2 in m/s^2
        a_max_sl_2 = (g*(mu*(Par.LDS.axle_load_front*cos(atan(sl_2))+(h_COG/wb)*sin(atan(sl_2)))-c_r*(Par.LDS.axle_load_rear*cos(atan(sl_2))-(h_COG/wb)*sin(atan(sl_2)))-sin(atan(sl_2)))-(F_L/max_weight_without_trailer))/1-(h_COG/wb)*(mu+c_r); 

        %x and y are auxiliary variables for shortening term and making code clearer
        x=Par.LDS.axle_load_rear*cos(atan(sl_3))-(h_COG/wb)*sin(atan(sl_3));
        y=Par.LDS.axle_load_front*cos(atan(sl_3))+(h_COG/wb)*sin(atan(sl_3));

        %ratios are defined for making code clearer
        m_ratio_auth  = max_weight_without_trailer/(max_weight_without_trailer+weight_trailer);
        m_ratio_trail = weight_trailer/(max_weight_without_trailer+weight_trailer);

        %a_max for sl_3 in m/s^2
        a_max_sl_3 = (g*(m_ratio_auth*(mu*y-c_r*x)-m_ratio_trail*cos(atan(sl_3))-sin(atan(sl_3))))/(1-m_ratio_auth*(h_COG/wb)*(mu+c_r)); 

    case 'all_wheel'
        
        %Calculate max Moment (in Nm) at the wheels (simplified). Will be needed later
        T_wheels_f=v.LDS.MOTOR{1}.T_max.*v.LDS.GEARBOX{1}.i_gearbox(1).*v.LDS.GEARBOX{1}.eta(1).*v.LDS.MOTOR{1}.quantity;
        T_wheels_r=v.LDS.MOTOR{2}.T_max.*v.LDS.GEARBOX{2}.i_gearbox(1).*v.LDS.GEARBOX{2}.eta(1).*v.LDS.MOTOR{2}.quantity;
        T_wheels=T_wheels_f+T_wheels_r;

        %a_max for sl_1 in m/s^2
        a_max_sl_1 = g*(mu*cos(atan(sl_1))-sin(atan(sl_1)))-(F_L/max_weight_without_trailer); 

        %a_max for sl_2 in m/s^2
        a_max_sl_2 = g*(mu*cos(atan(sl_2))-sin(atan(sl_2)))-(F_L/max_weight_without_trailer); 

        %a_max for sl_3 in m/s^2
        a_max_sl_3 = g*(mu*(max_weight_without_trailer/(max_weight_without_trailer+weight_trailer))-(weight_trailer/(max_weight_without_trailer+weight_trailer))*cos(atan(sl_3))*c_r-sin(atan(sl_3))); 
end

%% 4) Check if vehicle has enough Torque for the uphill scenarios

%Acceleration needed to reach 3.5 km/h (Start WLTP).
a=1;

%If a is above the traction limit, then the traction limit value is used

if a_max_sl_1 <= 0 %Traction limit negative, vehicle has not enough traction
    
    v.LDS = errorlog(v.LDS,'The vehicle does not have enough traction for the uphill scenario 1');
    
else %Test if the vehicle has enough torque
    
    Ftot_sl_1=F_reb_sl_1 + max_weight_without_trailer * v.LDS.parameters.e_i  *min(a,a_max_sl_1); 
    T_sl_1=Ftot_sl_1*v.LDS.wheel.r_dyn/1000;

    if T_sl_1>T_wheels
        v.LDS = errorlog(v.LDS,['vehicle unable to start by slope ', num2str(sl_1),' % and start acceleration ', num2str(min(a,a_max_sl_1)),'m/s^2']);
    end
    
end
    
if a_max_sl_2 <= 0 %Traction limit negative, vehicle has not enough traction
    
    v.LDS = errorlog(v.LDS,'The vehicle does not have enough traction for the uphill scenario 2');
    
else %Test if the vehicle has enough torque

    Ftot_sl_2=F_reb_sl_2 + max_weight_without_trailer * v.LDS.parameters.e_i  *min(a,a_max_sl_2); 
    T_sl_2=Ftot_sl_2*v.LDS.wheel.r_dyn/1000;

    if T_sl_2>T_wheels
        v.LDS = errorlog(v.LDS,['vehicle unable to start by slope ', num2str(sl_2),' % and start acceleration ', num2str(min(a,a_max_sl_1)),'m/s^2']);
    end
    
end

%Third test only for highfloor (SUV) vehicles
if a_max_sl_3 <= 0 && strcmp(v.Input.ground_clearance,'highfloor') %Traction limit negative, vehicle has not enough traction
    
    v.LDS = errorlog(v.LDS,'The vehicle does not have enough traction for the uphill scenario 3');
    
else %Test if the vehicle has enough torque
    
    Ftot_sl_3=F_reb_sl_3 + (max_weight_without_trailer+weight_trailer) * v.LDS.parameters.e_i  *min(a,a_max_sl_3); 
    T_sl_3=Ftot_sl_3*v.LDS.wheel.r_dyn/1000;

    if T_sl_3>T_wheels
        v.LDS = errorlog(v.LDS,['vehicle unable to start by slope ', num2str(sl_3),' % and start acceleration ', num2str(min(a,a_max_sl_1)),'m/s^2']);
    end

end

%% 5) Assign Outputs:
v.LDS.weights.trailer_weight=weight_trailer;
v.LDS.weights.vehicle_max_weight_no_trailer=max_weight_without_trailer;
    


end