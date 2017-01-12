%%   This S-function selects current controller from the set of unfalsified candidate controllers, based on the cost function
%%   Copyright by Myungsoo Jun and Michael G. Safonov, 1999, Revised on June 2002 - A.Paul
%%   Revised 2016 by Vahid to make it work with new Matlab 2015


function [sys,x0,str,ts] = surge_Supervisor_detectable(t,x,u,flag, ...
                                   C_p, C_i, C_d, rho, ...
                                   switching_method , eta , deta , deta_lim , deta_factor)  
persistent surge_u_dat
global delta


%initialize
sys=[];x0=[];str=[];ts=[];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    >>>  Initialization routine  <<<     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag == 0
   
   [surge_u_dat.K, surge_u_dat.total_number] = form_K(C_p, C_i, C_d); % TORD: Output is the entire controller set (\Theta) with PID parameters and m is the number of combinations possible, i.e. surge_u_dat.m = ni*np*nd; 
   surge_u_dat.m = 1;%surge_u_dat.total_number;    % initial number of elements of the candidate controller set K
                                    
   
   sizes = simsizes;   
   sizes.NumContStates  = 0;                        %Number of continuous states
   sizes.NumDiscStates  = 0;                        %Number of discrete states   
   sizes.NumOutputs     = surge_u_dat.total_number + 5;   %Number of outputs
   sizes.NumInputs      = surge_u_dat.total_number;       %Number of inputs
   sizes.DirFeedthrough = 0;                        %Flag for direct feedthrough
   sizes.NumSampleTimes = 1;                        %Number of sample times
   sys = simsizes(sizes);
   
   surge_u_dat.switching_method = switching_method;
   surge_u_dat.J     = zeros(surge_u_dat.total_number,1);
   surge_u_dat.index = (1:surge_u_dat.total_number)';

   [surge_u_dat.kp, surge_u_dat.ki, surge_u_dat.kd] = set_K_parameter(surge_u_dat.K, surge_u_dat.index, surge_u_dat.m); % Tord: Starting from behind, with current index of surge_u_dat.m which is the number of controllers
   surge_u_dat.cur_index = surge_u_dat.m;
   surge_u_dat.prev_index = surge_u_dat.cur_index;
   
   surge_u_dat.count = 0;
   surge_u_dat.t_switch = 0;
   
        %% Variables for LICLA
    surge_u_dat.eta   = eta;
    surge_u_dat.deta  = deta;
    surge_u_dat.increase_deta = 'false';
    surge_u_dat.unfalsified_controller_index = surge_u_dat.index;
   
   str = []; ts  = [delta 0];% sample time is delta

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 1
   sys = [];

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   >>> state update routine  <<<     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 2 
    surge_u_dat.count = surge_u_dat.count + 1;
    
    surge_u_dat.J = u;
    
    switch surge_u_dat.switching_method
        case 'epsilon'
            j=0;                                     %% Initilize the number of Unfalsified Controller
            p = surge_u_dat.J(surge_u_dat.cur_index);            %% Current falsification limit
            unfalsified_ball_index = [];
            
            for i=1:surge_u_dat.total_number
                if surge_u_dat.J(i)+rho < p               %% if ith controller is unfalsified
                    j = j+1;                        %% j : # of unfalsified Controllers
                    unfalsified_ball_index(j) = i;
                else
                    % Do nothing
                end
            end
            
            time_since_last = t-surge_u_dat.t_switch;
            if isempty(unfalsified_ball_index) %|| time_since_last < 1
                % Do nothing and continue with current active controller
            else % The unfalsified ball index set is non-empty, therfore a new controller is chosen
                [~, next_controller] = min(surge_u_dat.J(unfalsified_ball_index));   % New Controller selection Criterion
                [surge_u_dat.kp, surge_u_dat.ki, surge_u_dat.kd] = set_K_parameter(surge_u_dat.K, unfalsified_ball_index, next_controller);
                surge_u_dat.prev_index = surge_u_dat.cur_index;
                surge_u_dat.cur_index = unfalsified_ball_index(next_controller);
                surge_u_dat.t_switch = t;
            end
    
        case 'LICLA' %LICLA with only self-falsification
            if surge_u_dat.J(surge_u_dat.cur_index) > surge_u_dat.eta
                surge_u_dat.unfalsified_controller_index = surge_u_dat.unfalsified_controller_index(surge_u_dat.unfalsified_controller_index ~= surge_u_dat.cur_index);
                
                
                L = length(surge_u_dat.unfalsified_controller_index); 
                j = 0;
                temp = [];
                for k = 1:L
                    controller_id = surge_u_dat.unfalsified_controller_index(k);
                    if surge_u_dat.J(controller_id) < surge_u_dat.eta
                        j = j+1;
                        temp(j) = controller_id;
                    end
                end
                

                if ~isempty(temp) % Reinitialize falsified controllers that might performe better with varying system
                    surge_u_dat.unfalsified_controller_index = temp;
                
                    [~, next_controller] = min(surge_u_dat.J(surge_u_dat.unfalsified_controller_index));   % New Controller selection Criterion
                    [surge_u_dat.kp, surge_u_dat.ki, surge_u_dat.kd] = set_K_parameter(surge_u_dat.K, surge_u_dat.unfalsified_controller_index, next_controller);
                    surge_u_dat.prev_index = surge_u_dat.cur_index;
                    surge_u_dat.cur_index = surge_u_dat.unfalsified_controller_index(next_controller);
                    
%                     surge_u_dat.eta = surge_u_dat.eta + surge_u_dat.deta;
                else
                    surge_u_dat.unfalsified_controller_index = surge_u_dat.index;
                    surge_u_dat.eta = surge_u_dat.eta + surge_u_dat.deta;
                end
                
            else
                % Do nothing and continue with current active controller
            end
        case 'ICLA-1'
            if surge_u_dat.J(surge_u_dat.cur_index) > surge_u_dat.eta
                surge_u_dat.unfalsified_controller_index = surge_u_dat.unfalsified_controller_index(surge_u_dat.unfalsified_controller_index ~= surge_u_dat.cur_index);
                
                % Falsification of offline controllers
                L = length(surge_u_dat.unfalsified_controller_index); 
                j = 0;
                temp = [];
                for k = 1:L
                    controller_id = surge_u_dat.unfalsified_controller_index(k);
                    if surge_u_dat.J(controller_id) < surge_u_dat.eta
                        j = j+1;
                        temp(j) = controller_id;
                    end
                end
                
                if ~isempty(temp) % Reinitialize falsified controllers that might performe better with varying system
                    surge_u_dat.unfalsified_controller_index = temp;
                    
                    [~, next_controller] = min(surge_u_dat.J(surge_u_dat.unfalsified_controller_index));   % New Controller selection Criterion
                    [surge_u_dat.kp, surge_u_dat.ki, surge_u_dat.kd] = set_K_parameter(surge_u_dat.K, surge_u_dat.unfalsified_controller_index, next_controller);
                    surge_u_dat.prev_index = surge_u_dat.cur_index;
                    surge_u_dat.cur_index = surge_u_dat.unfalsified_controller_index(next_controller);
                    
                    surge_u_dat.deta = deta;
%                     surge_u_dat.eta = surge_u_dat.eta + surge_u_dat.deta;
                else
                    surge_u_dat.unfalsified_controller_index = surge_u_dat.index;

                    surge_u_dat.eta = surge_u_dat.eta + surge_u_dat.deta;
                    surge_u_dat.deta = surge_u_dat.deta * deta_factor;
                end
            else
                % Do nothing and continue with current active controller
                surge_u_dat.deta = deta;
            end
        case 'ICLA-2'
            if surge_u_dat.J(surge_u_dat.cur_index) > surge_u_dat.eta
                surge_u_dat.unfalsified_controller_index = surge_u_dat.unfalsified_controller_index(surge_u_dat.unfalsified_controller_index ~= surge_u_dat.cur_index);
                
                % Falsification of offline controllers
                L = length(surge_u_dat.unfalsified_controller_index); 
                j = 0;
                temp = [];
                for k = 1:L
                    controller_id = surge_u_dat.unfalsified_controller_index(k);
                    if surge_u_dat.J(controller_id) < surge_u_dat.eta
                        j = j+1;
                        temp(j) = controller_id;
                    end
                end
                
                if ~isempty(temp) % Reinitialize falsified controllers that might performe better with varying system
                    surge_u_dat.unfalsified_controller_index = temp;
                    
                    [~, next_controller] = min(surge_u_dat.J(surge_u_dat.unfalsified_controller_index));   % New Controller selection Criterion
                    [surge_u_dat.kp, surge_u_dat.ki, surge_u_dat.kd] = set_K_parameter(surge_u_dat.K, surge_u_dat.unfalsified_controller_index, next_controller);
                    surge_u_dat.prev_index = surge_u_dat.cur_index;
                    surge_u_dat.cur_index = surge_u_dat.unfalsified_controller_index(next_controller);
                    
                    surge_u_dat.deta = deta;
%                     surge_u_dat.eta = surge_u_dat.eta + surge_u_dat.deta;
                else 
                    surge_u_dat.unfalsified_controller_index = surge_u_dat.index;
                    
                    surge_u_dat.eta = surge_u_dat.eta + surge_u_dat.deta;
                    surge_u_dat.deta = surge_u_dat.deta * deta_factor;
                    if surge_u_dat.deta > deta_lim
                        surge_u_dat.deta = deta_lim;
                    end
                end
            else
                % Do nothing and continue with current active controller
                surge_u_dat.deta = deta;
            end
        otherwise 
            error('Chose an switching algorithm that is available; epsilon, LICLA or ICLA')
    end
   
   


   sys = [];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   >>>  Output routine  <<<    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 3

   
   
   % output kp,ki,kd, Current Controller Index & all Unfalsified Controller Index
    sys = [surge_u_dat.eta ; surge_u_dat.kp; surge_u_dat.ki; surge_u_dat.kd; surge_u_dat.cur_index; surge_u_dat.index];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
elseif flag == 4
   sys = [];
elseif flag == 9
   sys = [];  
   disp('Number of unfalsified controllers at the end = ')
   disp(surge_u_dat.m)
end


















