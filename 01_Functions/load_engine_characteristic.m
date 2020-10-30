function [vehLDS] = load_engine_characteristic(vehLDS) 
%% Description
% finds and loads a matching engine characteristic
%author:    Lorenzo Nicoletti, Korbinian Moller, FTM, TUM
%date:      07.06.2020
%% Input: 
%   vehLDS: struct with the vehicle LDS parameters
% 
%% Output:
%   vehLDS: struct with the vehicle LDS parameters and motor characteristics and diagram
%% Sources: 
%   --
%% Implementation: Load matching front motor characteristic

%% load local variables
characteristic_forced = vehLDS.settings.characteristic_forced;  %forced characteristic per axle, otherwise NaN
ratio_req = vehLDS.settings.ratio_req;                          %required angular speed ratio per axle
weight_T = vehLDS.settings.weight_T;                            %Torque weight factor per axle
weight_n = vehLDS.settings.weight_n;                            %speed weight factor per axle
weight_ratio = vehLDS.settings.weight_ratio;                    %angular speed ratio weight factor per axle

%Find which axles are filled: [front_axle, rear_axle]
filled_axles = vehLDS.settings.filled_axles;

%% find best combination

for i=find(filled_axles) 
    
    %no forced characteristic - find best 
    if all(isnan(cell2mat(characteristic_forced(i))))
            
        %The available motor characteristic list will be loaded only if it hasn't be loaded before
        if ~isfield(vehLDS.MOTOR{i},'available_configurations')
            vehLDS.MOTOR{i}.available_configurations = scan_characteristics(vehLDS.MOTOR{i}.type,vehLDS.settings.path); %Matrix with columns [M,n,ratio]
            vehLDS.MOTOR{i}.last_characteristic.counter = 0;
        end
    
        %load variable
        available_configurations = vehLDS.MOTOR{i}.available_configurations;
    

        
        %find matching Torque
        if isnan(vehLDS.MOTOR{i}.T_max) %no T_max given, use median of available torque
        
            vehLDS.MOTOR{i}.T_max = median(vehLDS.MOTOR{i}.available_configurations(:,1));
        
        end 
        
        %create required combination vector
        char_req = [vehLDS.MOTOR{i}.T_max, vehLDS.MOTOR{i}.n_max, ratio_req(i)];
        %compare reqired vector to availability matrix
        eval = abs(char_req - available_configurations);
        
        %avoid NaN in sum vectors
        % sum T vector
        if sum(eval(:,1)) == 0
            sum_T = 1;
        else
            sum_T = sum(eval(:,1));
        end
        % sum n vector
        if sum(eval(:,2)) == 0
            sum_n = 1;
        else
            sum_n = sum(eval(:,2));
        end
        % sum ratio vector
        if sum(eval(:,3)) == 0
            sum_ratio = 1;
        else
            sum_ratio = sum(eval(:,3));
        end
        
        %standardise eval vector and multiply by weight factors
        eval_weighted = [(eval(:,1) ./ sum_T) .* weight_T(i), (eval(:,2) ./ sum_n) .* weight_n(i), (eval(:,3) ./ sum_ratio) .* weight_ratio(i)];
        %calculate norm of each row
        eval_weighted_sum = sqrt((eval_weighted(:,1).^2 + eval_weighted(:,2).^2 + eval_weighted(:,3).^2));
        %find best combination, i.e. shortest norm
        [~,idx] = sort(eval_weighted_sum);
        
        %load T,n,ratio from avaiability matrix
        T = available_configurations(idx(1),1);
        n = available_configurations(idx(1),2);
        ratio = available_configurations(idx(1),3);
        
        %create filename
        filename = [vehLDS.MOTOR{i}.type,'_M',num2str(T),'_n',num2str(n),'_',num2str(ratio),'.mat'];
        
        
    else
        %if characteristic_forced is available, filename = characteristic_forced
        filename = cell2mat(characteristic_forced(i));
        vehLDS.MOTOR{i}.last_characteristic.counter = 0;
    end   
    
    %load local counter
    counter = vehLDS.MOTOR{i}.last_characteristic.counter;
    
    %determine wether to load a new diagram or scale the already loaded
    if all(~isnan(cell2mat(characteristic_forced(i)))) || vehLDS.MOTOR{i}.last_characteristic.T(end) ~= T || vehLDS.MOTOR{i}.last_characteristic.n(end) ~= n || vehLDS.MOTOR{i}.last_characteristic.ratio(end) ~= ratio       
        
        %compare number of duplicates to number of single entries (needs improvement)
        if length(vehLDS.MOTOR{i}.last_characteristic.T) <= 2 * length(unique(vehLDS.MOTOR{i}.last_characteristic.T))
            load_marker = 1;
        else
            load_marker = 0;
        end
    else 
        load_marker = 0;
    end
    
        %load recommended characteristic 
    if  load_marker == 1
            
        %assign characteristic to Mot struct
        load(filename);
        load_marker = 0;
        
        %adaption to new characteristics
        if ~exist('LDS_Lorenzo','var')
            LDS_Lorenzo = LDS_Diagramm;
        end
        vehLDS.MOTOR{i}.characteristic = [LDS_Lorenzo.Mot.Kennlinie.trq, LDS_Lorenzo.Mot.Kennlinie.rpm];
        
        %output which characteristic has been loaded
        fprintf('Axle %i characteristic %s loaded \n',i,filename);
        
        
        %store loaded characteristic parameters
        if all(isnan(cell2mat(characteristic_forced(i))))
        counter = counter + 1;
        vehLDS.MOTOR{i}.last_characteristic.T(counter) = T;
        vehLDS.MOTOR{i}.last_characteristic.n(counter) = n;
        vehLDS.MOTOR{i}.last_characteristic.ratio(counter) = ratio;
        vehLDS.MOTOR{i}.last_characteristic.counter = counter;
        end
    
        %Load and Flip n, T and eta
        n=flipud(LDS_Lorenzo.n);                                                          %rotational speed in 1/min                                                
        T=flipud(LDS_Lorenzo.M);                                                          %torque in Nm
        eta=LDS_Lorenzo.eta;                                                              %efficiency (already flipped)

        %Only the first quadrant (positive n and T) is considered
        %Find columns with allnegative or allzero elements for n matrix
        n_ind=find(nansum(n,1) <= 0);

        %Find rows with allnegative or allzero element for T Matrix
        T_ind=find(nansum(T,2) <= 0);

        %clear columns and rows with negative or zero rotational speed, torque and negative efficiency
        eta = eta(T_ind+1:end,n_ind+1:end);

        %clear contents smaller zero in torque and rotational speed vector
        T=linspace(0,max(T(:,1)),length(eta));
        n=linspace(0,max(n(1,:)),length(eta));

        %save the vectors of the diagramm
        vehLDS.MOTOR{i}.diagram.n_unscaled = n;                               %rotational speed in 1/min
        vehLDS.MOTOR{i}.diagram.T_unscaled = T;                               %torque in Nm
        vehLDS.MOTOR{i}.diagram.etages = eta;                                 %efficiencies
        
    end
    
        
end   

end

