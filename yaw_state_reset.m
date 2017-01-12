%%   Revised 2016 by Vahid to make it work with new Matlab 2015
% S function, inside "Adaptive PID Controller" block, for resetting the controller state, when switching to a new controller, 
% in order to reduce unwanted abrupt peaks in control signal u(t). Also generates trigger signal for the same purpose.

function [sys,x0,str,ts] = yaw_state_reset(t,x,u,flag, epsilon, delta)
persistent yaw_u_pid

% initialize
sys=[];x0=[];str=[];ts=[];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    >>>  Initialization routine  <<<   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag == 0
%%% Input u(1), u(2), u(3) = Kp, Ki, Kd;  u(4) = r(t);  u(5) = y(t); u(6), u(7) = Old integrator & Derivative state
   
   sizes = simsizes;   
   sizes.NumContStates  = 0;
   sizes.NumDiscStates  = 0;   
   sizes.NumOutputs     = 3;
   sizes.NumInputs      = 8;
   sizes.DirFeedthrough = 1;
   sizes.NumSampleTimes = 1;
   sys = simsizes(sizes);
   
   yaw_u_pid.kp = 0; 
   yaw_u_pid.ki = 0; 
   yaw_u_pid.kd = 0;    %%% Initializing
   
   yaw_u_pid.trigger = 1;
   yaw_u_pid.new_slow_state  = 0;
   yaw_u_pid.new_fast_state  = 0; 
   
   yaw_u_pid.r      = 0; 
   yaw_u_pid.y      = 0;
   yaw_u_pid.u_prev = 0;   
   yaw_u_pid.count  = 0;
   
   ts  = [delta 0];  % sample time is delta
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
elseif flag == 1
   sys = [];
elseif flag == 2    
    sys = [];
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    >>>  OUTPUT routine  <<<   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 3
  kp_new = u(1);  ki_new = u(2);  kd_new = u(3);
  yaw_u_pid.r       = u(4); 
  yaw_u_pid.y       = u(5);  
  yaw_u_pid.u_prev  = u(6);
  
  old_fast_state = u(7);
  old_slow_state = u(8);
  
  yaw_u_pid.new_fast_state  = old_fast_state; 
  yaw_u_pid.new_slow_state  = old_slow_state; 

  if (yaw_u_pid.kp ~= kp_new || yaw_u_pid.ki ~= ki_new || yaw_u_pid.kd ~= kd_new)  && (yaw_u_pid.count ~= 0)         
      [yaw_u_pid.new_slow_state , yaw_u_pid.new_fast_state , yaw_u_pid.trigger] = ...                               
                      new_states_2rd(   yaw_u_pid.kp , yaw_u_pid.ki , yaw_u_pid.kd ,...
                                        kp_new , ki_new , kd_new , yaw_u_pid.y , ...
                                        yaw_u_pid.r , yaw_u_pid.u_prev , yaw_u_pid.trigger ,...
                                        epsilon , delta );
  elseif yaw_u_pid.count == 0
      % Initialization of fast state to handle initial conditions
      yaw_u_pid.new_fast_state = epsilon * (u(4)-u(5)); 
  end
  
  yaw_u_pid.kp      = kp_new; 
  yaw_u_pid.ki      = ki_new; 
  yaw_u_pid.kd      = kd_new;  
  yaw_u_pid.count   = yaw_u_pid.count + 1;
  % out put new integrator state, new differentiator state and trigger
  sys = [yaw_u_pid.new_fast_state ; yaw_u_pid.trigger ; yaw_u_pid.new_slow_state];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 4
   sys = [];
elseif flag == 9
   sys = [];  
end
