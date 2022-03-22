function [vehicle]=clear_struct(vehicle)
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich), Korbinian Moller
%-------------
% Created on: 06.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function removes all unnecessary information of the vehicle struct
% ------------
% Sources: More information regarding the implementation of the LDS functions is available at:
%          [1] K. Moller, „Antriebsstrangmodellierung zur Optimierung autonomer Elektrofahrzeuge,“Semester thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2021.
% ------------
% Input: vehicle: The vehicle structure -> Stores the calculated component volumes and masses
%------------
% Output: cleaned vehicle struct
% ------------
%% Implementation:
clear_struct = true;

if clear_struct == true
    
    fields_standard = {'t','delta_t','v','a','alpha','resistance','T_wheels','n_wheels','P_wheels','P_batt','traction_limit','E_bat_step','E_bat_cum','dist_vec'};

    if strcmp(vehicle.LDS.settings.drive,'front_wheel')
        fields_front = {'overload_vector_mot_f','T_mot_f','n_mot_f','eta_mot_f','P_el_mot_f','P_mech_mot_f','max_overload_time_in_cycle_mot_f'};
        fields = [fields_standard, fields_front];
    elseif strcmp(vehicle.LDS.settings.drive,'rear_wheel')
        fields_rear = {'overload_vector_mot_r','T_mot_r','n_mot_r','eta_mot_r','P_el_mot_r','P_mech_mot_r','max_overload_time_in_cycle_mot_r'};
        fields = [fields_standard, fields_rear];
    else
        fields_awd = {'max_overload_time_in_cycle_mot_f', 'overload_vector_mot_f','max_overload_time_in_cycle_mot_r', 'overload_vector_mot_r','eta_mot_f','eta_mot_r','T_mot_f','T_mot_r','n_mot_f','n_mot_r','torque_distribution','torque_distribution_cycle','P_mech_mot_f','P_mech_mot_r','P_el_mot_f','P_el_mot_r'};
        fields = [fields_standard, fields_awd];
    end

    vehicle.LDS.sim_cons = rmfield(vehicle.LDS.sim_cons, fields);
end