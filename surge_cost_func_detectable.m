%%  Revised 2016 by Vahid to make it work with new Matlab 2015
%%  Reworked 2017 by Tord F. Onstein for master thesis at institute of marine technology, Norwegian University of Science and Technology
function [sys,x0,str,ts] = surge_cost_func_detectable(t,x,u,flag, ...
                                                C_p, C_i, C_d) 
global surge_u_cost delta  

%initialize
sys=[];x0=[];str=[];ts=[];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    >>>  Initialization routine  <<<   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag == 0
   
   [surge_u_cost.K, surge_u_cost.total_number] = form_K(C_p, C_i, C_d); % initial number of elements of the candidate controllers
   
   sizes = simsizes;
   sizes.NumContStates  = 0;
   sizes.NumDiscStates  = 0;
   sizes.NumOutputs     = surge_u_cost.total_number;
   sizes.NumInputs      = surge_u_cost.total_number+2;
   sizes.DirFeedthrough = 1;
   sizes.NumSampleTimes = 1;
   sys = simsizes(sizes);
   
   surge_u_cost.window = 50;
   surge_u_cost.fading = 0.05;
   surge_u_cost.root   = 9;
   
   surge_u_cost.J    = zeros(surge_u_cost.total_number,1);
   surge_u_cost.index = (1:surge_u_cost.total_number)';

   surge_u_cost.temp = zeros(surge_u_cost.total_number,0); % For determining instantaneous cost - Tord: See under consistency_test
   surge_u_cost.temp_max = zeros(surge_u_cost.total_number,1); % Necessary vector for epsilon-hysteresis switching
   surge_u_cost.count = 0;
   
   str = [];  ts  = [delta 0];% sample time is delta

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   

elseif flag == 1 
    sys = [];   
elseif flag == 2 
    sys=[];   
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   >>>  Output routine  <<<   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 3
    surge_u_cost.count = surge_u_cost.count +1;
    for i=1:surge_u_cost.total_number  
        surge_u_cost.r(i) = u(2+i); 

        [kp,ki,kd]  = set_K_parameter(surge_u_cost.K,surge_u_cost.index,i);
        K           = [kp,ki,kd];
        [surge_u_cost.J(i)] = consistency_test_detectable_windowing_fading(   u(1), u(2), u(2+i),...
                                                                        K , i , surge_u_cost.count,...
                                                                        surge_u_cost.window , surge_u_cost.fading,...
                                                                        surge_u_cost.root);
    end
    
    sys = [surge_u_cost.J];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

elseif flag == 4
   sys = [];
elseif flag == 9
   sys = [];  
end















