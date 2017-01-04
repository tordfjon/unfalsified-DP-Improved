%%   This S-function selects current controller from the set of unfalsified candidate controllers, based on the cost function
%%   Copyright by Myungsoo Jun and Michael G. Safonov, 1999, Revised on June 2002 - A.Paul
%%   Revised 2016 by Vahid to make it work with new Matlab 2015


function [sys,x0,str,ts] = Supervisor_detectable(t,x,u,flag, ...
                                   C_p, C_i, C_d,rho)  %C_p, C_i, C_d used for determining the initial number of candidate controllers
persistent u_dat
global delta %delta - sampling time


%initialize
sys=[];x0=[];str=[];ts=[];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    >>>  Initialization routine  <<<     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag == 0
   
   [u_dat.K, u_dat.total_number] = form_K_set(C_p, C_i, C_d); % TORD: Output is the entire controller set (\Theta) with PID parameters and m is the number of combinations possible, i.e. u_dat.m = ni*np*nd; 
   u_dat.m = 1;%u_dat.total_number;    % initial number of elements of the candidate controller set K
                                    
   
   sizes = simsizes;   
   sizes.NumContStates  = 0;                        %Number of continuous states
   sizes.NumDiscStates  = 0;                        %Number of discrete states   
   sizes.NumOutputs     = u_dat.total_number + 4;   %Number of outputs
   sizes.NumInputs      = u_dat.total_number;       %Number of inputs
   sizes.DirFeedthrough = 0;                        %Flag for direct feedthrough
   sizes.NumSampleTimes = 1;                        %Number of sample times
   sys = simsizes(sizes);
   
%    switching_eps    = 20;
   u_dat.J     = zeros(u_dat.total_number,1);
   u_dat.index = [1:u_dat.total_number]';

   [u_dat.kp, u_dat.ki, u_dat.kd] = set_K_parameter(u_dat.K, u_dat.index, u_dat.m); % Tord: Starting from behind, with current index of u_dat.m which is the number of controllers
   u_dat.cur_index = u_dat.m;
   u_dat.prev_index = u_dat.cur_index;
   
      %% Variables for data storage
   u_dat.temp = zeros(u_dat.total_number,0); %%% for determining instantaneous cost
   u_dat.myj = zeros(u_dat.total_number,0);
   u_dat.count = 0;
   u_dat.t_switch = 0;
   
   str = []; ts  = [delta 0];% sample time is delta

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 1
   sys = [];

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   >>> state update routine  <<<     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 2 
    u_dat.count = u_dat.count + 1;
    
    u_dat.J = u;                             %% Input u(n) is the cost of n th Candidate Controller
    j=0;                                     %% Initilize the number of Unfalsified Controller
    p = u_dat.J(u_dat.cur_index);            %% Cost of current Controller
    unfalsified_ball_index = [];

    for i=1:u_dat.total_number               % u_dat.m - # of unfalsified controller candidates
        if u_dat.J(i)+rho < p        %% if ith controller is unfalsified
            j = j+1;                         %% j : # of unfalsified Controllers
            unfalsified_ball_index(j) = i;
        else 
            % Do nothing
        end
    end
    
    time_since_last = t-u_dat.t_switch;
    if isempty(unfalsified_ball_index) %|| time_since_last < 1% i.e. no controller has better performance including hysteresis step
        % Algorithm does not change controller
    else % The unfalsified ball index set is non-empty, therfore a new controller is chosen
        [~, next_controller] = min(u_dat.J(unfalsified_ball_index));   % New Controller selection Criterion
        [u_dat.kp, u_dat.ki, u_dat.kd] = set_K_parameter(u_dat.K, unfalsified_ball_index, next_controller);
        u_dat.prev_index = u_dat.cur_index; 
        u_dat.cur_index = unfalsified_ball_index(next_controller);
        u_dat.t_switch = t;
    end
   
   
   
   %%%% IMPORTANT LINE! %%%%
% % %    u_dat.m = j;                  %  # of unfalsified controllers up to now
   %%%% IMPORTANT LINE! %%%%

   sys = [];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   >>>  Output routine  <<<    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 3

%    if (u_dat.count == 50)
%      Test1=45;
%      plot_Contr_index('change') %for simulink scope initialization, changing the attributes of controller index scope lines
%    end

   
   %%Tord: initializing the vector u_dat.controller which holds indexes of all "as of yet" unfalsified controllers. 
   % The array is updated at every output call. Q: Why "inf"? A: Plotting
   % "inf" will appear as nothing and the vector should be of length equal
   % to number of all controllers
%    u_dat.controller=[]; % For online Simulink Plot of unfalsified controller indices
% % %    for i=1:u_dat.total_number
% % %       u_dat.controller=[u_dat.controller inf];
% % %    end
%    u_dat.controller = u_dat.index; %Just to make lines visible at active controller plot

   %Tord: As u_dat.m is updated to range only the, as of now, unfalsified
   %controllers. u_dat.controller is initialized as inf in all indicies and
   %has length equal that of the total number of controllers.
   %u_dat.controller is then updated s.t. the index of those controllers
   %who are unfalsified has numbered values. The rest (falsified
   %controllers) is "inf". Counting only the u_dat.m first indexes of
   %u_dat.index, which is the ones unfalsified by data. 
% % %    for (i=1:u_dat.m)
% % %       u_dat.controller(u_dat.index(i)) = u_dat.index(i);
% % %    end


   
   
   % output kp,ki,kd, Current Controller Index & all Unfalsified Controller Index
    sys = [u_dat.kp; u_dat.ki; u_dat.kd; u_dat.cur_index; u_dat.index];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
elseif flag == 4
   sys = [];
elseif flag == 9
   sys = [];  
   disp('Number of unfalsified controllers at the end = ')
   disp(u_dat.m)
end