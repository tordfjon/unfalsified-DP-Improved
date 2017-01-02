%%   Revised 2016 by Vahid to make it work with new Matlab 2015
% S function, inside "Adaptive PID Controller" block, for resetting the controller state, when switching to a new controller, 
% in order to reduce unwanted abrupt peaks in control signal u(t). Also generates trigger signal for the same purpose.

function [sys,x0,str,ts] = yaw_state_reset(t,x,u,flag, epsilon, delta)
persistent u_pid

% initialize
sys=[];x0=[];str=[];ts=[];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    >>>  Initialization routine  <<<   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag == 0
%%% Input u(1), u(2), u(3) = Kp, Ki, Kd;  u(4) = r(t);  u(5) = y(t); u(6), u(7) = Old integrator & Derivative state
   
   sizes = simsizes;   
   sizes.NumContStates  = 0;
   sizes.NumDiscStates  = 0;   
   sizes.NumOutputs     = 3;
   sizes.NumInputs      = 7;
   sizes.DirFeedthrough = 1;
   sizes.NumSampleTimes = 1;
   sys = simsizes(sizes);
   
   u_pid.kp=0; u_pid.ki = 0; u_pid.kd = 0;    %%% Initializing
   u_pid.trigger = 1;
   u_pid.delta_x = [0; 0];
   u_pid.r = 0; u_pid.y = 0;
   u_pid.state_i=0;  u_pid.state_d =0;
   u_pid.count = 0;
   
   ts  = [delta 0];  % sample time is delta
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
elseif flag == 1
   sys = [];
elseif flag == 2 
    sys = [];
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    >>>  OUTPUT routine  <<<   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 3
  kp_new = u(1);  ki_new = u(2);  kd_new = u(3);
  u_pid.r = u(4); u_pid.y=u(5);  
  u_pid.state_i = u(6); u_pid.state_d = u(7);
  %%% If there is a change in Controller parameters, then reset the states & the trigger signal
  %%% But do not reset the states at the very first iteration, when parameters changes from zero to the initial values
  if ((u_pid.kp ~= u(1) || u_pid.ki ~= u(2) || u_pid.kd ~= u(3)) && (u_pid.count ~= 0))         
      [u_pid.delta_x, u_pid.trigger] = ...                               
                      new_states(   u_pid.kp, u_pid.ki, u_pid.kd,...
                                    kp_new, ki_new, kd_new, u_pid.y, u_pid.state_i,...
                                    u_pid.state_d, u_pid.r, u_pid.trigger, epsilon, delta);   
  else   
      u_pid.delta_x = [0; 0];   
  end
  u_pid.kp = kp_new; u_pid.ki = ki_new; u_pid.kd = kd_new;  %% update states
  u_pid.count = u_pid.count + 1;
  % out put new integrator state, new differentiator state and trigger
  sys = [u_pid.state_i+u_pid.delta_x(1,1);  u_pid.state_d+u_pid.delta_x(2,1);   u_pid.trigger];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 4
   sys = [];
elseif flag == 9
   sys = [];  
end
