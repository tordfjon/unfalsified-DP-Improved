%% <<<<<< Definition of controller type >>>>>>
%% TORD: Note that this function is spesific for a PID-controller
%% Revised 2016 by Tord F. Onstein

function [K, m] = form_K_set(Kp_set, Ki_set, Kd_set)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Range of controller parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% C_p = [5 10 25 80 110, 120]';
% C_i = [2 50 100]';
% C_d = [.5 .6]';

ni = size(Ki_set,2);
np = size(Kp_set,2);
nd = size(Kd_set,2); 

m = ni*np*nd; % Tord: Total number of possible combinations in kp, ki and kd, i.e. total number of controller in controller set

%%%%%%%%%%%%%%%%%%%%%%%%%
% Form a controller set K
%%%%%%%%%%%%%%%%%%%%%%%%%

K = []; % Tord: Hard to initialize to K = zeros(m,3) since indexing in a tripple backwards for-loop assosiates problems

for j=ni:-1:1
   for i=np:-1:1
      for k=nd:-1:1
         K = [K; Kp_set(i) Ki_set(j) Kd_set(k)];
      end
   end
end

%% test modification to avoid all intermediate mixed sign controllers
K = [  -10     -0.05    2;
       -1.2    -0.05    2;
        1.2     0.05    2;
        10      0.05    2   ];
m = size(K,1);
end