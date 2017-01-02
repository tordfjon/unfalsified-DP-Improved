%%      Revised 2016 by Vahid to make it work with new Matlab 2015
%%      Commented 2016 by Tord: Commented for ease of use for students that are not famliar with S-functions
function [sys,x0,str,ts] = yaw_fic_ref(t,x,u,flag, ...
                                           C_p, C_i, C_d)
persistent u_ref
global epsilon  delta  %%% Epsilon & delta (sampling time) was stored as global variable during 
                        %%% masked subsystem 'PID Cpntroller' initialization commands

% initialize the output vector
sys=[];x0=[];str=[];ts=[];
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    >>>  Initialization routine  <<<   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag == 0
   
   [u_ref.K, u_ref.total_number] = form_K_set(C_p, C_i, C_d);   % initial number of elements of the candidate controllers
   % u_ref.K is the complete candidate controller set formed by the
   % function form_K_set and C_p, C_i and C_d are the vectors holding the
   % values which define 
   
   sizes = simsizes;                % Tord: Nessecary to initialize all parameters associated with the S-function block                                                             
   sizes.NumContStates  = 0;
   sizes.NumDiscStates  = 0;
   sizes.NumOutputs     = u_ref.total_number;   % Tord: The output of the fictitious reference sisgnal generator is a unique signal for every controller in the set, hence, the total number of available controllers
   sizes.NumInputs      = 2;
   sizes.DirFeedthrough = 1;
   sizes.NumSampleTimes = 1;
   sys = simsizes(sizes);           % Tord: This call is necessary to write the initialization of sizes to sys
                                    % Note that this is the output for the
                                    % initializaton flag. The data is
                                    % stored in the simulink memory 
   
   % Definitions of persistent children
   u_ref.Ak = [];                   % Tord: Definition of children of u_ref for usage locally in this function
   u_ref.Bk = [];                   % Tord: Definition of children of u_ref for usage locally in this function
   u_ref.Ck = [];                   % Tord: Definition of children of u_ref for usage locally in this function
   u_ref.Dk = [];                   % Tord: Definition of children of u_ref for usage locally in this function
   u_ref.r  = [];                   % Tord: Definition of children of u_ref for usage locally in this function
   u_ref.x_c = zeros(u_ref.total_number,2);
   u_ref.index = [1:u_ref.total_number]';
   
   
   for i=1:u_ref.total_number  % generating a set of filters which will produce fictitious reference signals
      [cp,ci,cd] = set_K_parameter(u_ref.K, u_ref.index, i);
      
      if (cp==0 || ci==0 || cd==0)
          error('This is a relization of PID controller, hence Kp, Ki, Kd of all candidate controllers should be nonzero')
          % This error message is necessary as the generator uses the
          % inverse of controller transferfunction. For all parameters
          % equal to zero, this turns out real bad :-( 
      end
      
      controller1 = tf(cp,1) + tf(ci,[1 0]);
      controller2 = tf([cd 0],[epsilon 1]);
      cont_ss2 = [0, 1] + controller1 \ [1, ss(controller2)] ;   % inverse of controller. See paper, equation (3).
      cont = c2d(cont_ss2, delta);      % Exact discretization of LTI system
      [a,b,c,d] = ssdata(ss(cont));     % Not necesary to call ss() since ssdata will take care of it. Also, cont is already in ss form so cont.a will access the system matrix
      u_ref.Ak = [u_ref.Ak; a];     % Tord: These are the stack of matrices representing all controllers in a ss format
      u_ref.Bk = [u_ref.Bk; b];     % Tord: The matrices stack vertically and the output is easely found for all controllers through simple matrix calculation of the system output (flag 3)
      u_ref.Ck = [u_ref.Ck; c];
      u_ref.Dk = [u_ref.Dk; d];
  end
  str = []; 
  ts  = [delta, 0];% sample time is delta and the callback starts from t=0

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 1 % Tord: Calculates the derivatives ofthe continous state variables (not in use)
   sys = [];   
elseif flag == 2    % Tord: Updates descrete states, sample times and major time step requirements (Why not in use?)
    sys=[];         % Not in use because the state variable of the S-function is not in use, x. States are stored in a persistent variable u_ref.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    >>>  Output routine  <<<   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 3
    for i=1:u_ref.total_number   % total number of unfalsified controller candidates
        %controller1 = tf(cp,1) + tf(ci,[1 0]);
        %controller2 = tf([cd 0],[epsilon 1]);
        %cont_ss2 = [0, 1] + controller1 \ [1, ss(controller2)] ;   % inverse of controller
        %cont = c2d(cont_ss2, delta);
        %[Ak,Bk,Ck,Dk] = ssdata(ss(cont));
        Ak = u_ref.Ak((2*u_ref.index(i)-1):(2*u_ref.index(i)),:);
        Bk = u_ref.Bk((2*u_ref.index(i)-1):(2*u_ref.index(i)),:);
        Ck = u_ref.Ck(u_ref.index(i),:);
        Dk = u_ref.Dk(u_ref.index(i),:);
        xi = u_ref.x_c(u_ref.index(i),:)';
        xf = (Ak*xi + Bk*[u(1); u(2)])';    % Tord: Note that the system is discretizised and thus not directly integrated. 
                                            %       xf is calculated 
        r  = Ck*xi + Dk*[u(1); u(2)];       % Tord: State output is calculated, here: reference signal
        u_ref.x_c(u_ref.index(i),:) = xf; %% State of Ficticious Signal Generator
        u_ref.r(i) = r;  %%% Ficticious Reference Signal
    end
    sys = [u_ref.r]; % Output Ficticious Reference Signal

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 4
   sys = [];
elseif flag == 9
   sys = [];  
end
