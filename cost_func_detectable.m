%%  Revised 2016 by Vahid to make it work with new Matlab 2015
%%  Reworked 2017 by Tord F. Onstein for master thesis at institute of marine technology, Norwegian University of Science and Technology
function [sys,x0,str,ts] = cost_func_detectable(t,x,u,flag, ...
                                                C_p, C_i, C_d) 
global u_cost delta  

%initialize
sys=[];x0=[];str=[];ts=[];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    >>>  Initialization routine  <<<   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag == 0
   
   [u_cost.K, u_cost.total_number] = form_K(C_p, C_i, C_d); % initial number of elements of the candidate controllers
   
   sizes = simsizes;
   sizes.NumContStates  = 0;
   sizes.NumDiscStates  = 0;
   sizes.NumOutputs     = u_cost.total_number;
   sizes.NumInputs      = u_cost.total_number+2;
   sizes.DirFeedthrough = 1;
   sizes.NumSampleTimes = 1;
   sys = simsizes(sizes);
   
   u_cost.window = 50;
   u_cost.fading = 0.05;
   u_cost.root   = 9;
   
   u_cost.J    = zeros(u_cost.total_number,1);
   u_cost.index = (1:u_cost.total_number)';

   u_cost.temp = zeros(u_cost.total_number,0); % For determining instantaneous cost - Tord: See under consistency_test
   u_cost.temp_max = zeros(u_cost.total_number,1); % Necessary vector for epsilon-hysteresis switching
   u_cost.count = 0;
   
   str = [];  ts  = [delta 0];% sample time is delta

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   

elseif flag == 1 
    sys = [];   
elseif flag == 2 
    sys=[];   
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   >>>  Output routine  <<<   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 3
    u_cost.count = u_cost.count +1;
    for i=1:u_cost.total_number  
        u_cost.r(i) = u(2+i); 

        [kp,ki,kd]  = set_K_parameter(u_cost.K,u_cost.index,i);
        K           = [kp,ki,kd];
        [u_cost.J(i)] = consistency_test_detectable_windowing_fading(   u(1), u(2), u(2+i),...
                                                                        K , i , u_cost.count,...
                                                                        u_cost.window , u_cost.fading,...
                                                                        u_cost.root);
    end
    
    sys = [u_cost.J];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

elseif flag == 4
   sys = [];
elseif flag == 9
   sys = [];  
end















