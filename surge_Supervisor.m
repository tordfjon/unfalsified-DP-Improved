%%   This S-function selects current controller from the set of unfalsified candidate controllers, based on the cost function
%%   Copyright by Myungsoo Jun and Michael G. Safonov, 1999, Revised on June 2002 - A.Paul
%%   Revised 2016 by Vahid to make it work with new Matlab 2015


function [sys,x0,str,ts] = surge_Supervisor(t,x,u,flag, ...
                                   C_p, C_i, C_d, rho1)  %C_p, C_i, C_d used for determining the initial number of candidate controllers
persistent u_dat
global delta rho


%initialize
sys=[];x0=[];str=[];ts=[];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    >>>  Initialization routine  <<<     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag == 0
   
   [u_dat.K, u_dat.m] = form_K_set(C_p, C_i, C_d); % TORD: Output is the entire controller set (\Theta) with PID parameters and m is the number of combinations possible, i.e. u_dat.m = ni*np*nd; 
   u_dat.total_number = u_dat.m;    % initial number of elements of the candidate controller set K
                                    % Tord: Whats the difference?
   
   sizes = simsizes;   
   sizes.NumContStates  = 0;                        %Number of continuous states
   sizes.NumDiscStates  = 0;                        %Number of discrete states   
   sizes.NumOutputs     = u_dat.total_number + 5;   %Number of outputs
   sizes.NumInputs      = u_dat.total_number;       %Number of inputs
   sizes.DirFeedthrough = 0;                        %Flag for direct feedthrough
   sizes.NumSampleTimes = 1;                        %Number of sample times
   sys = simsizes(sizes);
   
   u_dat.J     = zeros(u_dat.m,1);
   u_dat.index = [1:u_dat.m]';

   [u_dat.kp, u_dat.ki, u_dat.kd] = set_K_parameter(u_dat.K, u_dat.index, u_dat.m); % Tord: Starting from behind, with current index of u_dat.m which is the number of controllers
   u_dat.cur_index = u_dat.m;
   
      %% Variables for data storage
   u_dat.temp = zeros(u_dat.m,0); %%% for determining instantaneous cost
   u_dat.myj = zeros(u_dat.m,0);
   u_dat.count = 0;
   rho = rho1;
   
   str = []; ts  = [delta 0];% sample time is delta

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 1
   sys = [];   

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   >>> state update routine  <<<     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 2 
   u_dat.count = u_dat.count + 1;
   
   u_dat.J = u;   %% Input u(n) is the cost of n th Candidate Controller
   j=0;           %% Initilize the number of Unfalsified Controller
   
   for i=1:u_dat.m                      % u_dat.m - # of unfalsified controller candidates
       
       if u_dat.J(u_dat.index(i)) <= rho1    % if ith controller is unfalsified
           j = j+1;                         % j : # of unfalsified Controllers
           u_dat.index(j) = u_dat.index(i); %Tord: Dette er en DÅRLIG algoritme. Det som skjer er at index for en falsifisert kontroller blir erstattet med en som er ufalsifisert.
                                            % Svakheten her er at
                                            % index-vectoren alltid er like
                                            % lang slik at til slutt når
                                            % det kanskje bare er en igjen
                                            % vil denne søkes gjennom m
                                            % ganger! Løsning kan være å
                                            % kutte index vectoren når
                                            % løkken er ferdig: u_dat.index
                                            % = u_dat.index(1:j+1); 
       end
   end
   
   p = u_dat.J(u_dat.cur_index); %% Cost of current Controller
   
   
   %%%% IMPORTANT LINE! %%%%
   u_dat.m = j;                  %%  # of unfalsified controllers up to now
   %%%% IMPORTANT LINE! %%%%
   
   
   if u_dat.m == 0               %% If all controllers has been falsified
         error('All Surge Controllers Has been falsified; try increasing the set of initial candidate controllers');
   elseif (p > rho1) %|| ~mod(t,5)                %%% If cost of current controller > 0, change the controller

         % Tord: This hold no hysteresis switching algorithm! 
         [~, next_controller] = min(u_dat.J(u_dat.index(1:u_dat.m)));   % New Controller selection Criterion
         [u_dat.kp, u_dat.ki, u_dat.kd] = set_K_parameter(u_dat.K, u_dat.index, next_controller);
%          u_dat.kp = u_dat.K(u_dat.index(next_controller), 1);           % The selection is made only from those controllers that are unfalsified
%          u_dat.ki = u_dat.K(u_dat.index(next_controller), 2);           % using u_dat.J(u_dat.index(1:u_dat.m))!
%          u_dat.kd = u_dat.K(u_dat.index(next_controller), 3);           % where u_dat.m is updated on line 78!
         u_dat.cur_index = u_dat.index(next_controller);
   end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   >>>  Output routine  <<<    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif flag == 3


    
% % %    if (u_dat.count == 50)
% % %      Test1=45;
% % %      plot_Contr_index('change') %for simulink scope initialization, changing the attributes of controller index scope lines
% % %    end

   
   %% Tord: initializing the vector u_dat.controller which holds indexes of all as yes unfalsified controllers. 
   % The array is updated at every output call. Q: Why "inf"? A: Plotting
   % "inf" will appear as nothing and the vector should be of length equal
   % to number of all controllers
   u_dat.controller=[]; % For online Simulink Plot of unfalsified controller indices
   for (i=1:u_dat.total_number)
      u_dat.controller=[u_dat.controller inf];
   end
   

   %Tord: As u_dat.m is updated to range only the, as of now, unfalsified
   %controllers. u_dat.controller is initialized as inf in all indicies and
   %has length equal that of the total number of controllers.
   %u_dat.controller is then updated s.t. the index of those controllers
   %who are unfalsified has numbered values. The rest (falsified
   %controllers) is "inf". Counting only the u_dat.m first indexes of
   %u_dat.index, which is the ones unfalsified by data. 
   for (i=1:u_dat.m)
      u_dat.controller(u_dat.index(i)) = u_dat.index(i);
   end
   

   
   
   % out put kp,ki,kd, Current Controller Index & all Unfalsified Controller Index
    sys = [rho1;u_dat.kp; u_dat.ki; u_dat.kd; u_dat.cur_index; u_dat.controller'];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
elseif flag == 4
   sys = [];
elseif flag == 9
   sys = [];  
   disp('Number of unfalsified surge-controllers at the end = ')
   disp(u_dat.m)
end