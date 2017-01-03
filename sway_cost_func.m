%%   Revised 2016 by Vahid to make it work with new Matlab 2015
function [sys,x0,str,ts] = sway_cost_func(t,x,u,flag, ...
                                           w1_n, w1_d, w2_n, w2_d, C_p, C_i, C_d) % Tord: The order of the variables is highly important! It must be the same as listed in the dialog box in the simulink evironment S-func block. 
global sway_u_cost
global delta  %delta (sampling time) was initialized & declared global in masked subsystem 'PID Controller' initialization command

%initialize
sys=[];x0=[];str=[];ts=[];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    >>>  Initialization routine  <<<   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag == 0
   
   [sway_u_cost.K, sway_u_cost.total_number] = form_K_set(C_p, C_i, C_d); % initial number of elements of the candidate controllers
   
   [~, w1_states_number] = size(w1_d); % Tord: Husk at w1_d er kun en vektor. Den er enda ikke en TF!
   [~, w2_states_number] = size(w2_d); % Tord: Size-1 er da høyeste orden av s i tf-polynomet. 
   
   sizes = simsizes;
   sizes.NumContStates  = 0;
   sizes.NumDiscStates  = 0;   % m*(  # of states in filter W1 + # of states in filter W2 )
   sizes.NumOutputs     = sway_u_cost.total_number;
   sizes.NumInputs      = sway_u_cost.total_number+2;
   sizes.DirFeedthrough = 1;
   sizes.NumSampleTimes = 1;
   sys = simsizes(sizes);
   
   sway_u_cost.J     = zeros(sway_u_cost.total_number,1); %sway_u_cost.total_number is the total number of initially unfalsified controllers
   sway_u_cost.J1    = zeros(sway_u_cost.total_number,1); %sway_u_cost.total_number is the total number of initially unfalsified controllers
   sway_u_cost.J2    = zeros(sway_u_cost.total_number,1); %sway_u_cost.total_number is the total number of initially unfalsified controllers
   sway_u_cost.J3    = zeros(sway_u_cost.total_number,1); %sway_u_cost.total_number is the total number of initially unfalsified controllers
   sway_u_cost.index = [1:sway_u_cost.total_number]';
   sway_u_cost.x_w1  = zeros(sway_u_cost.total_number, w1_states_number-1); % Tord: Initializes the state space model for the filter with the dimension for filter state (Output still SISO)
   sway_u_cost.x_w2  = zeros(sway_u_cost.total_number, w2_states_number-1); % Tord: Initializes the state space model for the filter with the dimension for filter state (Output still SISO)
   sway_u_cost.temp  = zeros(sway_u_cost.total_number,0); % For determining instantaneous cost - Tord: See under consistency_test
   sway_u_cost.temp1 = zeros(sway_u_cost.total_number,0); % For determining instantaneous cost - Tord: See under consistency_test
   sway_u_cost.temp2 = zeros(sway_u_cost.total_number,0); % For determining instantaneous cost - Tord: See under consistency_test
   sway_u_cost.temp3 = zeros(sway_u_cost.total_number,0); % For determining instantaneous cost - Tord: See under consistency_test
   sway_u_cost.myj   = zeros(sway_u_cost.total_number,0); %Tord: See under consistency_test
   sway_u_cost.myj1  = zeros(sway_u_cost.total_number,0); %Tord: See under consistency_test
   sway_u_cost.myj2  = zeros(sway_u_cost.total_number,0); %Tord: See under consistency_test
   sway_u_cost.myj3  = zeros(sway_u_cost.total_number,0); %Tord: See under consistency_test
   sway_u_cost.count = 0;
   
   % Filter 1 realization
   filter1 = c2d(tf(w1_n, w1_d), delta);
   [sway_u_cost.A1,sway_u_cost.B1,sway_u_cost.C1,sway_u_cost.D1] = ssdata(ss(filter1));
   
   % Filter 2 realization
   filter2 = c2d(tf(w2_n, w2_d), delta);
   [sway_u_cost.A2,sway_u_cost.B2,sway_u_cost.C2,sway_u_cost.D2] = ssdata(ss(filter2));
   
   str = [];  ts  = [delta 0];% sample time is delta

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   

elseif flag == 1 % Tord: Calculates the derivatives ofthe continous state variables (not in use)
    sys = [];   
elseif flag == 2 % Tord: Updates descrete states, sample times and major time step requirements 
    sys=[];   
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   >>>  Output routine  <<<   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 3
    sway_u_cost.count = sway_u_cost.count +1;
    for i=1:sway_u_cost.total_number   % total number of unfalsified controller candidates
        sway_u_cost.r(i) = u(2+i); %% Ficticious reference signal
        
         % Calculation of consistency criterion      
        [sway_u_cost.J(i), sway_u_cost.J1(i), sway_u_cost.J2(i), sway_u_cost.J3(i), sway_u_cost.x_w1(i,:), sway_u_cost.x_w2(i,:)] = ...
             sway_consistency_test( sway_u_cost.A1, sway_u_cost.B1, sway_u_cost.C1, sway_u_cost.D1, sway_u_cost.x_w1(i,:)', ...
                               sway_u_cost.A2, sway_u_cost.B2, sway_u_cost.C2, sway_u_cost.D2, sway_u_cost.x_w2(i,:)', ...
                               u(1), u(2), sway_u_cost.r(i),sway_u_cost.J(i),sway_u_cost.J1(i),sway_u_cost.J2(i),sway_u_cost.J3(i), ...
                               delta,i,sway_u_cost.count);
    end
    % out put cost function J
    sys = [sway_u_cost.J];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 4
   sys = [];
elseif flag == 9
   sys = [];  
end























