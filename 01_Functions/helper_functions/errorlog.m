function [ vehicle ] = errorlog( vehicle, errormsg, varargin)
% Designed by: Lorenzo Nicoletti
%-------------
% Created on: 06.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function keeps a track of the error occurring during
%              the calculation and stores them in the structure ErrorLog
% ------------
% Input: vehicle: struct with the vehicle parameters
%        errormsg: error message to log
%        varargin: Its only a marker. If it is not empty it means that the Warning is so important that it has to be displayed regardeless the suppress warning option
%------------
% Output: vehicle struct with ErrorLog
% ------------
%% Implementation
% check if errorlog exists yet
if isfield(vehicle,'ErrorLog')
    % get number of existing entries
    k = size(vehicle.ErrorLog,1);
else
    k = 0;
end
    
% log message and calling function
vehicle.ErrorLog{k+1,1} = errormsg;

%The user did not supress the warnings -> show them in the command window!
if isfield(vehicle.settings,'suppress_LDS_warnings') %The structure which was passed to ErrorLog is the vehicle.LDS struct
    if vehicle.settings.suppress_LDS_warnings==0 || ~isempty(varargin)
        disp(errormsg)
    end
else %The structure which was passed to the ErrorLog is the vehicle struct
    if vehicle.settings.suppress_warnings==0 || ~isempty(varargin)
        disp(errormsg)
    end
end
end