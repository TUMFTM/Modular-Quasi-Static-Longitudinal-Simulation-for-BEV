function plot_result_LDS(veh)
% Designed by: Lorenzo Nicoletti (FTM, Technical University of Munich), Korbinian Moller
%-------------
% Created on: 07.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function can be called to plot the results of the LDS once the lds is done calculating
% ------------
% Input: v: The vehicle structure -> Stores the calculated component volumes and masses
% ------------
% Output: None
% ------------
%% Implementation
% 0) Initialize inputs
% 1) Plot speed for the acceleration simulation
% 2) Plot resistance forces during acceleration simulation
% 3) Plot speed profile of the selected cycle
% 4) Plot energy consumption of the selected cycle
% 5) Plot the machines and their operating points

%% 0) Initialize inputs
sim_acc  = veh.LDS.sim_acc;     %Structure containing the results of the acceleration simulation
sim_cons = veh.LDS.sim_cons;    %Structure containing the results of the consumption simulation

%It could be that the LDS structure has been clared to save space. In this case it is not possible to plot the results
if ~isfield(sim_cons,'v')
    disp('The option clear struct for the LDS is activated. Therefore it is not possible to plot the results. To plot the result uncomment the line vehicle=clear_struct(vehicle) in the function calc_longitudinal_simulation and then simulate again')
    return
end

%Create figure to plot the results
fig=figure('Name','Results LDS'); fig.Units='Centimeters';
fig.Position(2)=2; fig.Position(3)=20; fig.Position(4)=18;

%% 1) Plot speed for the acceleration simulation
subplot(3,2,1);             
plot(sim_acc.t,sim_acc.v);

%Set Axis Labels
ax=gca; ax.XLabel.String='Time in s'; ax.YLabel.String='Speed in km/h'; ax.Title.String='Acc. simulation: speed profile';

%Set Axis limits
ax.YLim=[0,(veh.LDS.settings.v_max_sim)*1.05]; ax.XLim=[0,sim_acc.acc_time_is*1.05];

%% 2) Plot resistance forces during acceleration simulation
subplot(3,2,2);
plot(sim_acc.v,sim_acc.resistance.F_a)
hold on
plot(sim_acc.v,sim_acc.resistance.F_r)
plot(sim_acc.v,sim_acc.resistance.F_g)

legend('Drag resitance','Roll resistance','Slope resistance','Location','northwest')

%Set Axis Labels
ax=gca; ax.XLabel.String='Speed in km/h'; ax.YLabel.String='Resistance in N'; ax.Title.String='Acc. simulation: Resitance forces';

%Set Axis limits
ax.XLim=[0,veh.LDS.settings.v_max_sim*1.05];

%% 3) Plot speed profile of the selected cycle
subplot(3,2,3);
plot(sim_cons.t,sim_cons.v);

%Set Axis Labels
ax=gca; ax.XLabel.String='Time in s'; ax.YLabel.String='Speed in m/s'; ax.Title.String=[veh.Input.driving_cycle '-Cycle simulation: speed profile']; ax.Title.Interpreter = 'none';

%Set Axis limits
ax.XLim=[0,sim_cons.t(end)*1.05];

%% 4) Plot energy consumption of the selected cycle
subplot(3,2,4);
plot(sim_cons.t,sim_cons.E_bat_cum);

%Set Axis Labels
ax=gca; ax.XLabel.String='Time in cycle s'; ax.YLabel.String='Consumption in kWh'; ax.Title.String=[veh.Input.driving_cycle '-Cycle simulation: battery consumption']; ax.Title.Interpreter = 'none';

%Set Axis limits
ax.YLim=[0,max(sim_cons.E_bat_cum)*1.05];

%% 5) Plot the machines and their operating points
mot=veh.LDS.MOTOR;
filled_axles=[~isempty(veh.LDS.MOTOR{1}),~isempty(veh.LDS.MOTOR{2})];
for i=find(filled_axles)
    
    subplot(3,2,4+i);
    
    n_mot=mot{i}.diagram.n_scaled; % in 1/min
    T_mot=mot{i}.diagram.T_scaled; % in Nm 
    eta=mot{i}.diagram.etages;     % in percent
    
    [C,h]=contourf(n_mot,T_mot,eta,[0.3,0.7,0.9,0.93,0.95,0.96,0.965,0.97,0.9725,0.975,0.977,0.978,0.979]);
    hold on
    [C,h]=contourf(n_mot,-T_mot,eta,[0.3,0.7,0.9,0.93,0.95,0.96,0.965,0.97,0.9725,0.975,0.977,0.978,0.979]);
    if i==1
        scatter(sim_cons.n_mot_f,sim_cons.T_mot_f,15,'r','filled')
        str='Front motor';
    else
        scatter(sim_cons.n_mot_r,sim_cons.T_mot_r,15,'r','filled')
        str='Rear motor';
    end
    
    %Set Axis Labels
    ax=gca; ax.XLabel.String='Machine speed in 1/min'; ax.YLabel.String='Machine torque in Nm'; ax.Title.String=str;
end
end