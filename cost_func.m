%%   Revised 2016 by Vahid to make it work with new Matlab 2015
function [sys,x0,str,ts] = cost_func(t,x,u,flag, ...
                                           w1_n, w1_d, w2_n, w2_d, C_p, C_i, C_d) % Tord: The order of the variables is highly important! It must be the same as listed in the dialog box in the simulink evironment S-func block. 
global u_cost
global delta  %delta (sampling time) was initialized & declared global in masked subsystem 'PID Controller' initialization command

%initialize
sys=[];x0=[];str=[];ts=[];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    >>>  Initialization routine  <<<   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag == 0
   
   [u_cost.K, u_cost.total_number] = form_K_set(C_p, C_i, C_d); % initial number of elements of the candidate controllers
   
   [~, w1_states_number] = size(w1_d); % Tord: Husk at w1_d er kun en vektor. Den er enda ikke en TF!
   [~, w2_states_number] = size(w2_d); % Tord: Size-1 er da høyeste orden av s i tf-polynomet. 
   
   sizes = simsizes;
   sizes.NumContStates  = 0;
   sizes.NumDiscStates  = 0;   % m*(  # of states in filter W1 + # of states in filter W2 )
   sizes.NumOutputs     = u_cost.total_number;
   sizes.NumInputs      = u_cost.total_number+2;
   sizes.DirFeedthrough = 1;
   sizes.NumSampleTimes = 1;
   sys = simsizes(sizes);
   
   u_cost.J     = zeros(u_cost.total_number,1); %u_cost.total_number is the total number of initially unfalsified controllers
   u_cost.J1    = zeros(u_cost.total_number,1); %u_cost.total_number is the total number of initially unfalsified controllers
   u_cost.J2    = zeros(u_cost.total_number,1); %u_cost.total_number is the total number of initially unfalsified controllers
   u_cost.J3    = zeros(u_cost.total_number,1); %u_cost.total_number is the total number of initially unfalsified controllers
   u_cost.index = [1:u_cost.total_number]';
   u_cost.x_w1  = zeros(u_cost.total_number, w1_states_number-1); % Tord: Initializes the state space model for the filter with the dimension for filter state (Output still SISO)
   u_cost.x_w2  = zeros(u_cost.total_number, w2_states_number-1); % Tord: Initializes the state space model for the filter with the dimension for filter state (Output still SISO)
   u_cost.temp  = zeros(u_cost.total_number,0); % For determining instantaneous cost - Tord: See under consistency_test
   u_cost.temp1 = zeros(u_cost.total_number,0); % For determining instantaneous cost - Tord: See under consistency_test
   u_cost.temp2 = zeros(u_cost.total_number,0); % For determining instantaneous cost - Tord: See under consistency_test
   u_cost.temp3 = zeros(u_cost.total_number,0); % For determining instantaneous cost - Tord: See under consistency_test
   u_cost.myj   = zeros(u_cost.total_number,0); %Tord: See under consistency_test
   u_cost.myj1  = zeros(u_cost.total_number,0); %Tord: See under consistency_test
   u_cost.myj2  = zeros(u_cost.total_number,0); %Tord: See under consistency_test
   u_cost.myj3  = zeros(u_cost.total_number,0); %Tord: See under consistency_test
   u_cost.count = 0;
   
   % Filter 1 realization
   filter1 = c2d(tf(w1_n, w1_d), delta);
   [u_cost.A1,u_cost.B1,u_cost.C1,u_cost.D1] = ssdata(ss(filter1));
   
   % Filter 2 realization
   filter2 = c2d(tf(w2_n, w2_d), delta);
   [u_cost.A2,u_cost.B2,u_cost.C2,u_cost.D2] = ssdata(ss(filter2));
   
   str = [];  ts  = [delta 0];% sample time is delta

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   

elseif flag == 1 % Tord: Calculates the derivatives ofthe continous state variables (not in use)
    sys = [];   
elseif flag == 2 % Tord: Updates descrete states, sample times and major time step requirements 
    sys=[];   
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   >>>  Output routine  <<<   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 3
    u_cost.count = u_cost.count +1;
    for i=1:u_cost.total_number   % total number of unfalsified controller candidates
        u_cost.r(i) = u(2+i); %% Ficticious reference signal
        
         % Calculation of consistency criterion      
        [u_cost.J(i), u_cost.J1(i), u_cost.J2(i), u_cost.J3(i), u_cost.x_w1(i,:), u_cost.x_w2(i,:)] = ...
             consistency_test_sway( u_cost.A1, u_cost.B1, u_cost.C1, u_cost.D1, u_cost.x_w1(i,:)', ...
                               u_cost.A2, u_cost.B2, u_cost.C2, u_cost.D2, u_cost.x_w2(i,:)', ...
                               u(1), u(2), u_cost.r(i),u_cost.J(i),u_cost.J1(i),u_cost.J2(i),u_cost.J3(i), ...
                               delta,i,u_cost.count);
    end
    % out put cost function J
    sys = [u_cost.J];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 4
   sys = [];
elseif flag == 9
   sys = [];  
end























