%%      Revised 2016 by Vahid to make it work with new Matlab 2015
%%      Commented 2016 by Tord: Commented for ease of use for students that are not famliar with S-functions
function [sys,x0,str,ts] = surge_fic_ref(t,x,u,flag, ...
                                           C_p, C_i, C_d)
persistent u_ref
global epsilon  delta 

% initialize the output vector
sys=[];x0=[];str=[];ts=[];
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    >>>  Initialization routine  <<<   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag == 0
   
   [u_ref.K, u_ref.total_number] = form_K(C_p, C_i, C_d);   % initial number of elements of the candidate controllers
   
   sizes = simsizes;
   sizes.NumContStates  = 0;
   sizes.NumDiscStates  = 0;
   sizes.NumOutputs     = u_ref.total_number;
   sizes.NumInputs      = 2;
   sizes.DirFeedthrough = 1;
   sizes.NumSampleTimes = 1;
   sys = simsizes(sizes);
   
   % Definitions of persistent children
   u_ref.Ak     = [];
   u_ref.Bk     = [];
   u_ref.Ck     = [];
   u_ref.Dk     = [];
   u_ref.r      = [];
   u_ref.x_c    = zeros(u_ref.total_number,2);
   u_ref.index  = (1:u_ref.total_number)';
   
   
   for i=1:u_ref.total_number 
      [cp,ci,cd] = set_K_parameter(u_ref.K, u_ref.index, i);
      
      if (cp==0 || ci==0 || cd==0)
          error('This is a relization of PID controller, hence Kp, Ki, Kd of all candidate controllers should be nonzero')

      end
      
      %% Should be reworked! -T
%       controller1 = tf(cp,1) + tf(ci,[1 0]);
%       controller2 = tf([cd 0],[epsilon 1]);
%       cont_ss2 = [0, 1] + controller1 \ [1, ss(controller2)] ;   % inverse of controller. See paper, equation (3).
%       % include isstable() funksjon med error() hvis den feiler! 
%       cont = c2d(cont_ss2, delta);
%       % Evt. samme test på diskret system her
%       [a,b,c,d] = ssdata(ss(cont));
      %%
      
      K_s = tf(cp,1) + tf(ci,[1 0]) + tf([cd 0],[epsilon 1]); % Parameterized PID tf
      frg_tf = [0 1] + K_s\[1 0]; % r = y + K_s\u, y = u(2) u = u(1) from function input
      frg_ss = ss(frg_tf);
      if isstable(frg_ss.a)
          [a,b,c,d] = ssdata(c2d(frg_tf,delta));
      else
          error('FRG is not stable for controller %i . Controller should be minimum phase \nTry changing the parameter set or investigate is stability. \n-Tord',i)
      end
      
      %%
      u_ref.Ak = [u_ref.Ak; a];
      u_ref.Bk = [u_ref.Bk; b];
      u_ref.Ck = [u_ref.Ck; c];
      u_ref.Dk = [u_ref.Dk; d];
  end
  str = []; 
  ts  = [delta, 0];

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 1 % Tord: Calculates the derivatives ofthe continous state variables (not in use)
   sys = [];   
elseif flag == 2    % Tord: Updates descrete states, sample times and major time step requirements (Why not in use?)
    sys=[];         % Not in use because the state variable of the S-function is not in use, x. States are stored in a persistent variable u_ref.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    >>>  Output routine  <<<   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 3
    for i=1:u_ref.total_number   
        Ak = u_ref.Ak((2*u_ref.index(i)-1):(2*u_ref.index(i)),:);
        Bk = u_ref.Bk((2*u_ref.index(i)-1):(2*u_ref.index(i)),:);
        Ck = u_ref.Ck(u_ref.index(i),:);
        Dk = u_ref.Dk(u_ref.index(i),:);
        xi = u_ref.x_c(u_ref.index(i),:)';
        xf = (Ak*xi + Bk*[u(1); u(2)])';    % Tord: Note that the system is discretizised and thus not directly integrated. 
                                            %       xf is calculated 
        r  = Ck*xi + Dk*[u(1); u(2)];       % Tord: State output is calculated, here: reference signal
        u_ref.x_c(u_ref.index(i),:) = xf; %% State of Ficticious Signal Generator
        u_ref.r(i) = r;
    end
    sys = [u_ref.r];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 4
   sys = [];
elseif flag == 9
   sys = [];  
end



















