% =======
% Battery
% =======



% #####################################################################################################################

% Global variables
% ----------------
    global h                                    % Stepsize [s] from block "Driving Cycle"
    global Q_BT_IC                              % Initial charge [C] from block "Battery"

% #####################################################################################################################

% Battery parameters
% ------------------
    c_BT_E1     =  39;                          % Minimum battery voltage for discharging (13 cells, 3 V/cell, seminar)                 [V]
    c_BT_E2     =  0.013;					    % Minimum battery resistance for discharging (13 cells, 1 mOhm/cell, seminar)           [Ohm]
    c_BT_E3     =  15.6;					    % Maximum battery voltage variation for discharging (13 cells, 1.2 V/cell, seminar)     [V]
    c_BT_E4     =  0;							% Minimum battery resistance variation for discharging (13 cells, 0 mOhm/cell, seminar) [Ohm]
    c_BT_L1     =  c_BT_E1;                     % c_BT_L1 = c_BT_EL1 for continuity reasons
    c_BT_L2     =  0.013;					    % Minimum battery resistance for charging (13 cells, 1 mOhm/cell, seminar)              [V]
    c_BT_L3     =  c_BT_E3;                     % c_BT_L3 = c_BT_EL3 for continuity reasons
    c_BT_L4     =  0.00;                        % Minimum battery resistance variation for charging (13 cells, 0 mOhm/cell, seminar)    [Ohm]

% #####################################################################################################################

% Prepare data
% ------------
    Q_BT_0      = I_0 * 3600;					% Discharge current in 1 h              [C]
    U_BT_0      = c_BT_E1+c_BT_E3;              % Mean battery voltage                  [V]
    I_BT_max    = (60/t_ch)*I_0;				% Maximum charge/discharge current      [A]
    Q_BT_IC     = (Q_BT_IC_rel/100)*3600*I_0;   % Initial battery charge                [C]

% #####################################################################################################################
