%%      Revised 2016 by Vahid to make it work with new Matlab 2015
%%      Reworked 2016 by Tord
function [sys,x0,str,ts] = surge_fic_ref(t,x,u,flag, ...
                                           C_p, C_i, C_d)
persistent surge_u_ref
global epsilon  delta 

% initialize the output vector
sys=[];x0=[];str=[];ts=[];
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    >>>  Initialization routine  <<<   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag == 0
   
   [surge_u_ref.K, surge_u_ref.total_number] = form_K(C_p, C_i, C_d);   % initial number of elements of the candidate controllers
   
   sizes = simsizes;
   sizes.NumContStates  = 0;
   sizes.NumDiscStates  = 0;
   sizes.NumOutputs     = surge_u_ref.total_number;
   sizes.NumInputs      = 2;
   sizes.DirFeedthrough = 1;
   sizes.NumSampleTimes = 1;
   sys = simsizes(sizes);
   
   % Definitions of persistent children
   surge_u_ref.Ak     = [];
   surge_u_ref.Bk     = [];
   surge_u_ref.Ck     = [];
   surge_u_ref.Dk     = [];
   surge_u_ref.r      = [];
   surge_u_ref.x_c    = zeros(surge_u_ref.total_number,2);
   surge_u_ref.index  = (1:surge_u_ref.total_number)';
   
   
   for i=1:surge_u_ref.total_number 
      [cp,ci,cd] = set_K_parameter(surge_u_ref.K, surge_u_ref.index, i);
      
      if (cp==0 || ci==0 || cd==0)
          error('This is a relization of PID controller, hence Kp, Ki, Kd of all candidate controllers should be nonzero')
      end
      
      K_s = tf(cp,1) + tf(ci,[1 0]) + tf([cd 0],[epsilon 1]); % Parameterized PID tf
      frg_tf = [0 1] + K_s\[1 0]; % r = y + K_s\u, y = u(2) u = u(1) from function input
      frg_ss = ss(frg_tf);
      if isstable(frg_ss.a)
          [a,b,c,d] = ssdata(c2d(frg_tf,delta));
      else
          error('SURGE:\nFRG is not stable for controller %i . Controller should be minimum phase \nTry changing the parameter set or investigate its stability. \n-Tord',i)
      end
      
      %%
      surge_u_ref.Ak = [surge_u_ref.Ak; a];
      surge_u_ref.Bk = [surge_u_ref.Bk; b];
      surge_u_ref.Ck = [surge_u_ref.Ck; c];
      surge_u_ref.Dk = [surge_u_ref.Dk; d];
  end
  str = []; 
  ts  = [delta, 0];

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 1 
   sys = [];   
elseif flag == 2
    sys=[];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    >>>  Output routine  <<<   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 3
    for i=1:surge_u_ref.total_number   
        Ak = surge_u_ref.Ak((2*surge_u_ref.index(i)-1):(2*surge_u_ref.index(i)),:);
        Bk = surge_u_ref.Bk((2*surge_u_ref.index(i)-1):(2*surge_u_ref.index(i)),:);
        Ck = surge_u_ref.Ck(surge_u_ref.index(i),:);
        Dk = surge_u_ref.Dk(surge_u_ref.index(i),:);
        xi = surge_u_ref.x_c(surge_u_ref.index(i),:)';
        xf = (Ak*xi + Bk*[u(1); u(2)])';    
                                            
        r  = Ck*xi + Dk*[u(1); u(2)];       
        surge_u_ref.x_c(surge_u_ref.index(i),:) = xf; 
        surge_u_ref.r(i) = r;
    end
    sys = [surge_u_ref.r];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 4
   sys = [];
elseif flag == 9
   sys = [];  
end



















