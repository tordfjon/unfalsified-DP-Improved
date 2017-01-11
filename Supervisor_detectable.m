%%   This S-function selects current controller from the set of unfalsified candidate controllers, based on the cost function
%%   Copyright by Myungsoo Jun and Michael G. Safonov, 1999, Revised on June 2002 - A.Paul
%%   Revised 2016 by Vahid to make it work with new Matlab 2015


function [sys,x0,str,ts] = Supervisor_detectable(t,x,u,flag, ...
                                   C_p, C_i, C_d, rho, ...
                                   switching_method , eta , deta )  
persistent u_dat
global delta


%initialize
sys=[];x0=[];str=[];ts=[];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    >>>  Initialization routine  <<<     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag == 0
   
   [u_dat.K, u_dat.total_number] = form_K(C_p, C_i, C_d); % TORD: Output is the entire controller set (\Theta) with PID parameters and m is the number of combinations possible, i.e. u_dat.m = ni*np*nd; 
   u_dat.m = 1;%u_dat.total_number;    % initial number of elements of the candidate controller set K
                                    
   
   sizes = simsizes;   
   sizes.NumContStates  = 0;                        %Number of continuous states
   sizes.NumDiscStates  = 0;                        %Number of discrete states   
   sizes.NumOutputs     = u_dat.total_number + 5;   %Number of outputs
   sizes.NumInputs      = u_dat.total_number;       %Number of inputs
   sizes.DirFeedthrough = 0;                        %Flag for direct feedthrough
   sizes.NumSampleTimes = 1;                        %Number of sample times
   sys = simsizes(sizes);
   
   u_dat.switching_method = switching_method;
   u_dat.J     = zeros(u_dat.total_number,1);
   u_dat.index = (1:u_dat.total_number)';

   [u_dat.kp, u_dat.ki, u_dat.kd] = set_K_parameter(u_dat.K, u_dat.index, u_dat.m); % Tord: Starting from behind, with current index of u_dat.m which is the number of controllers
   u_dat.cur_index = u_dat.m;
   u_dat.prev_index = u_dat.cur_index;
   
   u_dat.count = 0;
   u_dat.t_switch = 0;
   
        %% Variables for LICLA
    u_dat.eta   = eta;
    u_dat.deta  = deta;
    u_dat.increase_deta = 'false';
    u_dat.unfalsified_controller_index = u_dat.index;
   
   str = []; ts  = [delta 0];% sample time is delta

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 1
   sys = [];

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   >>> state update routine  <<<     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 2 
    u_dat.count = u_dat.count + 1;
    
    u_dat.J = u;
    
    switch u_dat.switching_method
        case 'epsilon'
            j=0;                                     %% Initilize the number of Unfalsified Controller
            p = u_dat.J(u_dat.cur_index);            %% Current falsification limit
            unfalsified_ball_index = [];
            
            for i=1:u_dat.total_number
                if u_dat.J(i)+rho < p               %% if ith controller is unfalsified
                    j = j+1;                        %% j : # of unfalsified Controllers
                    unfalsified_ball_index(j) = i;
                else
                    % Do nothing
                end
            end
            
            time_since_last = t-u_dat.t_switch;
            if isempty(unfalsified_ball_index) %|| time_since_last < 1
                % Do nothing and continue with current active controller
            else % The unfalsified ball index set is non-empty, therfore a new controller is chosen
                [~, next_controller] = min(u_dat.J(unfalsified_ball_index));   % New Controller selection Criterion
                [u_dat.kp, u_dat.ki, u_dat.kd] = set_K_parameter(u_dat.K, unfalsified_ball_index, next_controller);
                u_dat.prev_index = u_dat.cur_index;
                u_dat.cur_index = unfalsified_ball_index(next_controller);
                u_dat.t_switch = t;
            end
    
        case 'LICLA' %LICLA with only self-falsification
            if u_dat.J(u_dat.cur_index) > u_dat.eta
                u_dat.unfalsified_controller_index = u_dat.unfalsified_controller_index(u_dat.unfalsified_controller_index ~= u_dat.cur_index);
                
                u_dat.eta = u_dat.eta + u_dat.deta;
                
                L = length(u_dat.unfalsified_controller_index); 
                j = 0;
                temp = [];
                for k = 1:L
                    controller_id = u_dat.unfalsified_controller_index(k);
                    if u_dat.J(controller_id) < u_dat.eta
                        j = j+1;
                        temp(j) = controller_id;
                    end
                end
                

                if isempty(temp) % Reinitialize falsified controllers that might performe better with varying system
                    temp = u_dat.index;
                end
                u_dat.unfalsified_controller_index = temp;
                
                [~, next_controller] = min(u_dat.J(u_dat.unfalsified_controller_index));   % New Controller selection Criterion
                [u_dat.kp, u_dat.ki, u_dat.kd] = set_K_parameter(u_dat.K, u_dat.unfalsified_controller_index, next_controller);
                u_dat.prev_index = u_dat.cur_index;
                u_dat.cur_index = u_dat.unfalsified_controller_index(next_controller);
                
            else
                % Do nothing and continue with current active controller
            end
        case 'ICLA-1'
            if u_dat.J(u_dat.cur_index) > u_dat.eta
                temp = u_dat.unfalsified_controller_index(u_dat.unfalsified_controller_index ~= u_dat.cur_index);
                if isempty(temp) % Reinitialize falsified controllers that might performe better with varying system
                    temp = u_dat.index;
                    u_dat.increase_deta = 'true';
                end
                u_dat.unfalsified_controller_index = temp;
                
                [~, next_controller] = min(u_dat.J(u_dat.unfalsified_controller_index));   % New Controller selection Criterion
                [u_dat.kp, u_dat.ki, u_dat.kd] = set_K_parameter(u_dat.K, u_dat.unfalsified_controller_index, next_controller);
                u_dat.prev_index = u_dat.cur_index;
                u_dat.cur_index = u_dat.unfalsified_controller_index(next_controller);
                u_dat.eta = u_dat.eta + u_dat.deta;
                if u_dat.increase_deta
                    u_dat.deta = u_dat.deta*3;
                    u_dat.increase_deta = 'false';
                end
            else
                % Do nothing and continue with current active controller
                u_dat.deta = 0.5;
            end
        case 'ICLA-2'
            if u_dat.J(u_dat.cur_index) > u_dat.eta
                temp = u_dat.unfalsified_controller_index(u_dat.unfalsified_controller_index ~= u_dat.cur_index);
                if isempty(temp) % Reinitialize falsified controllers that might performe better with varying system
                    temp = u_dat.index;
                    u_dat.increase_deta = 'true';
                end
                u_dat.unfalsified_controller_index = temp;
                
                [~, next_controller] = min(u_dat.J(u_dat.unfalsified_controller_index));   % New Controller selection Criterion
                [u_dat.kp, u_dat.ki, u_dat.kd] = set_K_parameter(u_dat.K, u_dat.unfalsified_controller_index, next_controller);
                u_dat.prev_index = u_dat.cur_index;
                u_dat.cur_index = u_dat.unfalsified_controller_index(next_controller);
                u_dat.eta = u_dat.eta + u_dat.deta;
                if u_dat.increase_deta
                    u_dat.deta = u_dat.deta*3;
                    if u_dat.deta > 50
                        u_dat.deta = 50;
                    end
                    u_dat.increase_deta = 'false';
                end
            else
                % Do nothing and continue with current active controller
                u_dat.deta = 0.5;
            end
        otherwise 
            error('Chose an switching algorithm that is available; epsilon, LICLA or ICLA')
    end
   
   


   sys = [];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   >>>  Output routine  <<<    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 3

   
   
   % output kp,ki,kd, Current Controller Index & all Unfalsified Controller Index
    sys = [u_dat.eta ; u_dat.kp; u_dat.ki; u_dat.kd; u_dat.cur_index; u_dat.index];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
elseif flag == 4
   sys = [];
elseif flag == 9
   sys = [];  
   disp('Number of unfalsified controllers at the end = ')
   disp(u_dat.m)
end


















