
% ============================================
% Combustion Engine (based on consumption map)
% ============================================



% #####################################################################################################################

% Global variables
% ----------------
    global h                                    % Stepsize [s] from block "Driving Cycle"
    global V_CE_map                             % Consumption map                           [kg/s]
    global w_CE_row                             % Engine speed range                        [rad/s]
    global T_CE_col                             % Engine torque range                       [Nm]
    global w_CE_max                             % Maximum engine speed                      [rad/s]
    global T_CE_max                             % Maximum engine torque                     [Nm]
    global H_u                                  % Fuel upper heating value                  [J/kg]
    global rho_f                                % Fuel volumetric mass density              [kg/l]
    
% #####################################################################################################################

% Load data
% ---------
    load OM_622                                 % Data for Mercedes-Benz OM 622             [from ADVISOR, FC_CI60_emis.m]

    w_CE_upper = max(w_CE_max);                 % Upper limit engine speed                  [rad/s]                 
    T_CE_idle  = P_CE_idle / w_CE_idle;         % Torque at idle                            [Nm]   
    
% #####################################################################################################################