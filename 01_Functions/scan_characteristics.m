function [available_conf_list] = scan_characteristics(type,path)
%% Description
% scans characteristic folder for available configurations
%author:    Korbinian Moller, FTM, TUM
%date:      04.06.2020
%% Input: 
%   Motor type
%   path where characteristic diagrams are stored
% 
%% Output:
%   available characteristics list
%% Sources: 
%   --
%% Implementation: 

%check path
folder = fileparts(which('path_characteristic')); %path where path_characteristic.m is stored
path = [folder,'/',path,'/',type]; %creates path to characteristic diagrams

if ~isfolder(path)
    fprintf(2,'\n \n \nFatal error! Characteristics for %s are not available. \nCheck whether path "%s" is correct and try again. \nPath to characteristics within project can be edited in initialize_inputparameters. \n \n \n \n ',type,path);
    return
end

%load characteristic folder
files = dir([path,'/*.mat']);

%declare local variables
size = length(files);
M = zeros(size,1);
n = zeros(size,1);
ratio = zeros(size,1);

%Analyse every characteristic file for torque and rotational speed
for i = 1:size
%     [~,remain] = strtok(files(i).name,'_');
%     [M_str,remain] = strtok(remain,'_');
%     [n_str] = strtok(remain,'_');
    cont = split(files(i).name,["_","."]);
    M_str = cell2mat(cont(2));
    n_str = cell2mat(cont(3));
    if numel(cont) >= 6
        ratio_str = cell2mat([cont(4) '.' cont(5)]);
    else
        ratio_str = '0';
    end
    M(i) = str2double(M_str(2:end));
    n(i) = str2double(n_str(2:end));
    ratio(i) = str2double(ratio_str);
    
end

%export available characteristics list
available_conf_list = [M,n,ratio];


end