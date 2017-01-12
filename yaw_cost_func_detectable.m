%%  Revised 2016 by Vahid to make it work with new Matlab 2015
%%  Reworked 2017 by Tord F. Onstein for master thesis at institute of marine technology, Norwegian University of Science and Technology
function [sys,x0,str,ts] = yaw_cost_func_detectable(t,x,u,flag, ...
                                                C_p, C_i, C_d) 
global yaw_u_cost delta  

%initialize
sys=[];x0=[];str=[];ts=[];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    >>>  Initialization routine  <<<   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag == 0
   
   [yaw_u_cost.K, yaw_u_cost.total_number] = form_K(C_p, C_i, C_d); % initial number of elements of the candidate controllers
   
   sizes = simsizes;
   sizes.NumContStates  = 0;
   sizes.NumDiscStates  = 0;
   sizes.NumOutputs     = yaw_u_cost.total_number;
   sizes.NumInputs      = yaw_u_cost.total_number+2;
   sizes.DirFeedthrough = 1;
   sizes.NumSampleTimes = 1;
   sys = simsizes(sizes);
   
   yaw_u_cost.window = 50;
   yaw_u_cost.fading = 0.05;
   yaw_u_cost.root   = 9;
   
   yaw_u_cost.J    = zeros(yaw_u_cost.total_number,1);
   yaw_u_cost.index = (1:yaw_u_cost.total_number)';

   yaw_u_cost.temp = zeros(yaw_u_cost.total_number,0); % For determining instantaneous cost - Tord: See under consistency_test
   yaw_u_cost.temp_max = zeros(yaw_u_cost.total_number,1); % Necessary vector for epsilon-hysteresis switching
   yaw_u_cost.count = 0;
   
   str = [];  ts  = [delta 0];% sample time is delta

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   

elseif flag == 1 
    sys = [];   
elseif flag == 2 
    sys=[];   
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   >>>  Output routine  <<<   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 3
    yaw_u_cost.count = yaw_u_cost.count +1;
    for i=1:yaw_u_cost.total_number  
        yaw_u_cost.r(i) = u(2+i); 

        [kp,ki,kd]  = set_K_parameter(yaw_u_cost.K,yaw_u_cost.index,i);
        K           = [kp,ki,kd];
        [yaw_u_cost.J(i)] = consistency_test_detectable_windowing_fading(   u(1), u(2), u(2+i),...
                                                                        K , i , yaw_u_cost.count,...
                                                                        yaw_u_cost.window , yaw_u_cost.fading,...
                                                                        yaw_u_cost.root);
    end
    
    sys = [yaw_u_cost.J];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

elseif flag == 4
   sys = [];
elseif flag == 9
   sys = [];  
end















