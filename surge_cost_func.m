%%   Revised 2016 by Vahid to make it work with new Matlab 2015
function [sys,x0,str,ts] = surge_cost_func(t,x,u,flag, ...
                                           w1_n, w1_d, w2_n, w2_d, C_p, C_i, C_d) % Tord: The order of the variables is highly important! It must be the same as listed in the dialog box in the simulink evironment S-func block. 
global surge_u_cost
global delta  %delta (sampling time) was initialized & declared global in masked subsystem 'PID Controller' initialization command

%initialize
sys=[];x0=[];str=[];ts=[];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    >>>  Initialization routine  <<<   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag == 0
   
   [surge_u_cost.K, surge_u_cost.total_number] = form_K_set(C_p, C_i, C_d); % initial number of elements of the candidate controllers
   
   [~, w1_states_number] = size(w1_d); % Tord: Husk at w1_d er kun en vektor. Den er enda ikke en TF!
   [~, w2_states_number] = size(w2_d); % Tord: Size-1 er da høyeste orden av s i tf-polynomet. 
   
   sizes = simsizes;
   sizes.NumContStates  = 0;
   sizes.NumDiscStates  = 0;   % m*(  # of states in filter W1 + # of states in filter W2 )
   sizes.NumOutputs     = surge_u_cost.total_number;
   sizes.NumInputs      = surge_u_cost.total_number+2;
   sizes.DirFeedthrough = 1;
   sizes.NumSampleTimes = 1;
   sys = simsizes(sizes);
   
   surge_u_cost.J     = zeros(surge_u_cost.total_number,1); %surge_u_cost.total_number is the total number of initially unfalsified controllers
   surge_u_cost.J1    = zeros(surge_u_cost.total_number,1); %surge_u_cost.total_number is the total number of initially unfalsified controllers
   surge_u_cost.J2    = zeros(surge_u_cost.total_number,1); %surge_u_cost.total_number is the total number of initially unfalsified controllers
   surge_u_cost.J3    = zeros(surge_u_cost.total_number,1); %surge_u_cost.total_number is the total number of initially unfalsified controllers
   surge_u_cost.index = [1:surge_u_cost.total_number]';
   surge_u_cost.x_w1  = zeros(surge_u_cost.total_number, w1_states_number-1); % Tord: Initializes the state space model for the filter with the dimension for filter state (Output still SISO)
   surge_u_cost.x_w2  = zeros(surge_u_cost.total_number, w2_states_number-1); % Tord: Initializes the state space model for the filter with the dimension for filter state (Output still SISO)
   surge_u_cost.temp  = zeros(surge_u_cost.total_number,1); % For determining instantaneous cost - Tord: See under consistency_test
   surge_u_cost.temp1 = zeros(surge_u_cost.total_number,0); % For determining instantaneous cost - Tord: See under consistency_test
   surge_u_cost.temp2 = zeros(surge_u_cost.total_number,0); % For determining instantaneous cost - Tord: See under consistency_test
   surge_u_cost.temp3 = zeros(surge_u_cost.total_number,0); % For determining instantaneous cost - Tord: See under consistency_test
   surge_u_cost.myj   = zeros(surge_u_cost.total_number,0); %Tord: See under consistency_test
   surge_u_cost.myj1  = zeros(surge_u_cost.total_number,0); %Tord: See under consistency_test
   surge_u_cost.myj2  = zeros(surge_u_cost.total_number,0); %Tord: See under consistency_test
   surge_u_cost.myj3  = zeros(surge_u_cost.total_number,0); %Tord: See under consistency_test
   surge_u_cost.count = 0;
   
   % Filter 1 realization
   filter1 = c2d(tf(w1_n, w1_d), delta);
   [surge_u_cost.A1,surge_u_cost.B1,surge_u_cost.C1,surge_u_cost.D1] = ssdata(ss(filter1));
   
   % Filter 2 realization
   filter2 = c2d(tf(w2_n, w2_d), delta);
   [surge_u_cost.A2,surge_u_cost.B2,surge_u_cost.C2,surge_u_cost.D2] = ssdata(ss(filter2));
   
   str = [];  ts  = [delta 0];% sample time is delta

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   

elseif flag == 1 % Tord: Calculates the derivatives ofthe continous state variables (not in use)
    sys = [];   
elseif flag == 2 % Tord: Updates descrete states, sample times and major time step requirements 
    sys=[];   
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   >>>  Output routine  <<<   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 3
    surge_u_cost.count = surge_u_cost.count +1;
    for i=1:surge_u_cost.total_number   % total number of unfalsified controller candidates
        surge_u_cost.r(i) = u(2+i); %% Ficticious reference signal
        
         % Calculation of consistency criterion      
        [surge_u_cost.J(i), surge_u_cost.J1(i), surge_u_cost.J2(i), surge_u_cost.J3(i), surge_u_cost.x_w1(i,:), surge_u_cost.x_w2(i,:)] = ...
             surge_consistency_test( surge_u_cost.A1, surge_u_cost.B1, surge_u_cost.C1, surge_u_cost.D1, surge_u_cost.x_w1(i,:)', ...
                               surge_u_cost.A2, surge_u_cost.B2, surge_u_cost.C2, surge_u_cost.D2, surge_u_cost.x_w2(i,:)', ...
                               u(1), u(2), surge_u_cost.r(i),surge_u_cost.J(i),surge_u_cost.J1(i),surge_u_cost.J2(i),surge_u_cost.J3(i), ...
                               delta,i,surge_u_cost.count);
    end
    % out put cost function J
    sys = [surge_u_cost.J];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 4
   sys = [];
elseif flag == 9
   sys = [];  
end























