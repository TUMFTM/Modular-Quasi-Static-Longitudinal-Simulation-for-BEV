function plot_dynamic(vehicle)
%% Description:
% This function plots the results  of the longitudinal dynamics simulation dynamically.
% Author: Korbinian Moller, TUM
% source: Alexander Koch, FTM, TUM
% date:      10.08.2020
%% dynamic Plot:
figure('units','normalized','outerposition',[0 0 1 1])

%drive cycle
plotdc = subplot(2,2,[1,2]);
hold on;
plot(vehicle.LDS.sim_cons.t,vehicle.LDS.sim_cons.v);
ax=gca;
ax.XLabel.String='t in s';
ax.YLabel.String='v in m/s';
ax.Title.String='Drive cycle';

%assign local variables
filled_axles = vehicle.LDS.settings.filled_axles;
mot=vehicle.LDS.MOTOR;
sim_cons = vehicle.LDS.sim_cons;
pause_time = vehicle.LDS.sim_cons.delta_t/100;


%% 2) create figure with subplot background
for i=find(filled_axles)
    
    subplot(2,2,2+i);
    
    n_mot=mot{i}.diagram.n_scaled;
    T_mot=mot{i}.diagram.T_scaled;
    eta=mot{i}.diagram.etages;
    [C,h]=contourf(n_mot,T_mot,eta,[0.3,0.7,0.9,0.93,0.95,0.96,0.965,0.97,0.9725,0.975,0.977,0.978,0.979]);
    hold on
    [C,h]=contourf(n_mot,-T_mot,eta,[0.3,0.7,0.9,0.93,0.95,0.96,0.965,0.97,0.9725,0.975,0.977,0.978,0.979]);
    
    if i == 1
        str = 'Motor front';
        
    else
        str = 'Motor rear';
    end
    ax=gca;
    ax.XLabel.String='motor speed in 1/min';
    ax.YLabel.String='motor torque in Nm';
    ax.Title.String=str;
    
end

%% 3) scatter data points

%reduce data points
step = 10;
sim_cons.t = sim_cons.t(1:step:end);
sim_cons.v = sim_cons.v(1:step:end);

%all wheel drive
if all(filled_axles)
    %reduce data points
    sim_cons.n_mot_f = sim_cons.n_mot_f(1:step:end);
    sim_cons.T_mot_f = sim_cons.T_mot_f(1:step:end);
    sim_cons.n_mot_r = sim_cons.n_mot_r(1:step:end);
    sim_cons.T_mot_r = sim_cons.T_mot_r(1:step:end);
    %assign subplot
    plotf = subplot(2,2,3);
    plotr = subplot(2,2,4);

    for i = 1:length(vehicle.LDS.sim_cons.t)
        scatter(plotdc,sim_cons.t(i),sim_cons.v(i),15,'r','filled');
        scatter(plotf,sim_cons.n_mot_f(i),sim_cons.T_mot_f(i),15,'r','filled');
        scatter(plotr,sim_cons.n_mot_r(i),sim_cons.T_mot_r(i),15,'r','filled');
        
        pause(pause_time);
    end

%front wheel drive
elseif filled_axles(1)
    %reduce data points
    sim_cons.n_mot_f = sim_cons.n_mot_f(1:step:end);
    sim_cons.T_mot_f = sim_cons.T_mot_f(1:step:end);
    %assign subplot
    plotf = subplot(2,2,3);
    
    for i = 1:length(vehicle.LDS.sim_cons.t)
        scatter(plotdc,sim_cons.t(i),sim_cons.v(i),15,'r','filled');
        scatter(plotf,sim_cons.n_mot_f(i),sim_cons.T_mot_f(i),15,'r','filled');
        
        pause(pause_time);
    end

%rear wheel drive
elseif filled_axles(2)
    %reduce data points
    sim_cons.n_mot_r = sim_cons.n_mot_r(1:step:end);
    sim_cons.T_mot_r = sim_cons.T_mot_r(1:step:end);
    %assign subplot
    plotr = subplot(2,2,4);
    
    for i = 1:length(vehicle.LDS.sim_cons.t)
        scatter(plotdc,sim_cons.t(i),sim_cons.v(i),15,'r','filled');
        scatter(plotr,sim_cons.n_mot_r(i),sim_cons.T_mot_r(i),15,'r','filled');
        
        pause(pause_time);
    end

end


end