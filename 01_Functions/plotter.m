function plotter(veh)
%% Description:
% This function plots the results of the longitudinal dynamics simulation.
%
% date:      21.05.2020
%% Plot:

figure

sim_acc=veh.LDS.sim_acc;

%Plot speed for the acceleration simulation
subplot(3,2,1);
v_acc=sim_acc.v;                   
t_acc=sim_acc.t;                
plot(t_acc,v_acc);
ax=gca;
ax.XLabel.String='time in s';
ax.YLabel.String='speed in km/h';
ax.Title.String='Acc. simulation: speed profile';
ax.YLim=[0,(veh.LDS.settings.v_max_sim)*1.05];
ax.XLim=[0,sim_acc.acc_time_is*1.05];
%-----------------------------------------

%Plot resistance for the acceleration simulation
subplot(3,2,2);
plot(v_acc,sim_acc.resistance.F_a)
hold on
plot(v_acc,sim_acc.resistance.F_r)
plot(v_acc,sim_acc.resistance.F_g)
legend('drag resitance','roll resistance','slope resistance','Location','northwest')
ax=gca;
ax.XLabel.String='km/h';
ax.YLabel.String='Resistance in N';
ax.Title.String='Acc. simulation: Resitance forces';
ax.XLim=[0,veh.LDS.settings.v_max_sim*1.05];

%-----------------------------------------

%Plot cycle of energy consumption simulation
subplot(3,2,3);
sim_cons=veh.LDS.sim_cons;
v_cons=sim_cons.v;
t_cons=sim_cons.t;
plot(t_cons,v_cons);
ax=gca;
ax.XLabel.String='time in s';
ax.YLabel.String='speed in m/s';
ax.Title.String=[veh.Input.driving_cycle '-Cycle simulation: speed profile'];
ax.Title.Interpreter = 'none';
ax.XLim=[0,t_cons(end)*1.05];

%-----------------------------------------

%Plot energy consumption in cycle
subplot(3,2,4);
plot(t_cons,sim_cons.E_bat_cum);
ax=gca;
ax.XLabel.String='time in cycle s';
ax.YLabel.String='consumption in kWh';
ax.Title.String=[veh.Input.driving_cycle '-Cycle simulation: battery consumption'];
ax.Title.Interpreter = 'none';
ax.YLim=[0,max(sim_cons.E_bat_cum)*1.05];
%-----------------------------------------

%Plot motor and motor points
mot=veh.LDS.MOTOR;
filled_axles=[~isempty(veh.LDS.MOTOR{1}),~isempty(veh.LDS.MOTOR{2})];
for i=find(filled_axles)
    
    subplot(3,2,4+i);
    
    n_mot=mot{i}.diagram.n_scaled;
    T_mot=mot{i}.diagram.T_scaled;
    eta=mot{i}.diagram.etages;
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
    
    ax=gca;
    ax.XLabel.String='motor speed in 1/min';
    ax.YLabel.String='motor torque in Nm';
    ax.Title.String=str;
    
end

set(gcf, 'PaperUnits', 'centimeters')
set(gcf, 'Units', 'centimeters')
set(gcf,'OuterPosition', [12.7000    8.8265   20   22]);


end

