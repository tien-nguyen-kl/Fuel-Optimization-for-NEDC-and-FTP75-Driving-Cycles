function u = final_controller(input)

w_MGB  = input(1);            % get flywheel angular velocity
dw_MGB = input(2);           % get flywheel angular acceleration
T_MGB  = input(3);            % get flywheel torque
i      = input(4);             %get gear number
v      = input(5);              %get velocity
Q_BT   = input(6);              %get SOC
clock  = input(7);              %get clock

global w_EM_max;             % define maximum motor angular velocity (global) 
global T_EM_max;             % define maximum motor torque (global)
theta_EM = 0.1;              % define motor inertia

load v_FTP75.mat;

Q = 36000;
T_MGB_th_NEDC = 60;              % define torque threshold for NEDC (cf. Slide 3-8)
T_MGB_th_FTP75 = 39.5;           % define torque threshold for FTP-75 (cf. Slide 3-8)
epsilon = 0.01;                  % define epsilon (cf. Slide 3-8 and 3-10)
u_LPS_max = 0.3;                 % define maximum torque-split factor for LPS (cf. Slide 3-8)

mode_detection = (abs(v-v_FTP75(clock+1))<1);   % mode_detection = 1: FTP75
                                                % mode_detection = 0: NEDC


switch mode_detection
    case 1
%FTP75

% Engine Start/Stop:
% Deceleration, SOC must > 30%, speed vehicle < 5m/s and demanded torque < 15Nm -> Engine OFF
    if (Q_BT>0.3*Q)  && (v<5)&& (dw_MGB<=0) && (v<5) && (T_MGB<15)
            u(1)=0; 
            u(2) = 1;
         
    else    
 % E-Drive:
 %Demanded torque <52% of EM maximum torque and SOC >29% SOC_max
        if ((T_MGB>0)  && (T_MGB < 0.52*interp1(w_EM_max,T_EM_max,w_MGB)) && (Q_BT>0.29*Q))
                u(2) = 1;  %E-drive
                u(1) = 0;   %Engine OFF
                
                 
  % LPS Motor:
  % Demanded torque > threshold torque of FTP75 and SOC > 62% SOC_max
        elseif  (T_MGB>T_MGB_th_FTP75) && (Q_BT>0.62*Q) 
            u(2) = min((interp1(w_EM_max,T_EM_max,w_MGB)-abs(theta_EM*dw_MGB)-epsilon)/T_MGB,u_LPS_max);  %LPS Motor
            u(1) = 1;   %Engine ON
           
    
   % LPS_Gen:
   % Positive demanded torque <14Nm and SOC < 70% SOC_max 
        elseif ((T_MGB>0) && (T_MGB<14) && (Q_BT < 0.7*Q)) 
            u(2) = -min((interp1(w_EM_max,T_EM_max,w_MGB)-abs(theta_EM*dw_MGB)-epsilon)/T_MGB,0.3);  %LPS Gen
            u(1) = 1;   %Engine ON
       
    
    % Regeneration:
    %Negative demanded torque and SOC < 70% SOC_max
        elseif (Q_BT<0.7*Q) && (T_MGB<0) 
            u(2) = min((interp1(w_EM_max,-T_EM_max,w_MGB)+abs(theta_EM*dw_MGB)+epsilon)/T_MGB,1);
            u(1)=0;     %Engine OFF
          
    
     % ICE only:
     %Engine only and charge battery to get sustained SOC at the end of
     %drive cycle
        else 
            u(2) = -0.12;  %ICE
            u(1) = 1;
         
        
        end;
    end;

    case 0
%NEDC

% Deceleration, speed vehicle < certain speed (choose 10km/h) -> Engine OFF
if (Q_BT>0.2*Q)  && (v<5)&& (dw_MGB<=0) && (T_MGB<15)
        u(1)=0; 
        u(2) = 1;
       
else    

 % E-Drive:
 %Demanded torque <52% of EM maximum torque and SOC >29% SOC_max
    if ((T_MGB>0)  && (T_MGB < 0.45*interp1(w_EM_max,T_EM_max,w_MGB)) && (Q_BT>0.25*Q))
            u(2) = 1;  %E-drive
            u(1) = 0;   %Engine OFF
        
  % LPS Motor:
  % Demanded torque > threshold torque of NEDC and SOC > 34% SOC_max
    elseif  (T_MGB>T_MGB_th_NEDC) && (Q_BT>0.34*Q) 
        u(2) = min((interp1(w_EM_max,T_EM_max,w_MGB)-abs(theta_EM*dw_MGB)-epsilon)/T_MGB,u_LPS_max);  %LPS Motor
        u(1) = 1;   %Engine ON
       

   % LPS_Gen:
   % Positive demanded torque <60Nm and SOC < 60% SOC_max 
    elseif ((T_MGB>0) && (T_MGB<60) && (Q_BT < 0.6*Q)) 
        u(2) = -min((interp1(w_EM_max,T_EM_max,w_MGB)-abs(theta_EM*dw_MGB)-epsilon)/T_MGB,0.3);  %LPS Gen
        u(1) = 1;   %Engine ON
        
    % Regeneration:
    %Negative demanded torque and SOC < 70% SOC_max
    elseif (Q_BT<0.7*Q) && (T_MGB<0) 
        u(2) = min((interp1(w_EM_max,-T_EM_max,w_MGB)+abs(theta_EM*dw_MGB)+epsilon)/T_MGB,1);
        u(1)=0;     %Engine OFF
        

     % ICE only:
     %Engine only and charge battery to get sustained SOC at the end of
     %drive cycle
    else 
        u(2) = -0.15;  %ICE
        u(1) = 1;
       
    
    end;
end;
end;