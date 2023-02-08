% ==============
% Electric Motor
% ==============



% #####################################################################################################################

% Global variables
% ----------------
    global h                            % Stepsize [s] from block "Driving Cycle"
    global eta_EM_map                   % Efficiency map                [-]
    global w_EM_row                     % Motor speed range             [rad/s]
    global T_EM_col                     % Motor torque range            [Nm]
    global w_EM_max                     % Maximum motor speed           [rad/s]
    global T_EM_max                     % Maximum motor torque          [Nm]
        
% #####################################################################################################################

% Load data
% ---------
    load EM;

    w_EM_upper = max(w_EM_max);         % Upper limit motor speed       [rad/s]
    
% #####################################################################################################################


