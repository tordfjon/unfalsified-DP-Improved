%%   Revised 2016 by Vahid to make it work with new Matlab 2015
function [J,J1,J2,J3, xfw1, xfw2] = yaw_consistency_test(aw1, bw1, cw1, dw1, xw1, ...
                                            aw2, bw2, cw2, dw2, xw2, ...
                                            u, y, r, ...
                                            J_prev,J1_prev,J2_prev,J3_prev, delta, index,count)
                                        
error_gain      = 10;
controller_gain = 0.1;
reference_gain  = 1;
global yaw_u_cost
persistent t
xfw1    = (aw1*xw1 + bw1*(y - r))';
j1   = cw1*xw1 + dw1*(y - r); %Tord: Calculation of filter1 with input (y-r) state space output. Equation (9) says (r-y)
% j1      = error_gain*(y-r);
xfw2    = (aw2*xw2 + bw2*u)';
j2      = cw2*xw2 + dw2*u; %Tord: Calculation of filter2 with inout u state space output
% j2 = controller_gain*u;

j3      = r;
% j3      = reference_gain*r;

% For DP with           %% Noise comp. term
temp = j1^2 + j2^2 -j3^2 - (1/300);% - 2.1213^2; % Including std. of the time varying environmental force. Tord: This is the evaluated cost function according to (9) in the paper. See notes in OneNote

%% Writing temporary values to global var
yaw_u_cost.temp(index,count) = temp;
yaw_u_cost.temp1(index,count) = j1;
yaw_u_cost.temp2(index,count) = j2;
yaw_u_cost.temp3(index,count) = j3;



J1 = J1_prev + delta*(j1^2);
J2 = J2_prev + delta*(j2^2);
J3 = J3_prev + delta*(j3^2);


%% Cost with forgetting
forgetting_time = 100; %[s]
forgetting_rate = 0.02;
count_limit = round(forgetting_time/delta);

if isempty(t)
    t = 0:delta:forgetting_time;
end

if count > count_limit
    J = sum(fliplr(yaw_u_cost.temp(index,count-count_limit:count)).*exp(-forgetting_rate*t)).*delta;
else 
    t_hor = (count-1)*delta;
    J = sum(fliplr(yaw_u_cost.temp(index,1:count)).*exp(-forgetting_rate*(0:delta:t_hor))).*delta;
end
% J = J_prev + temp*delta; %Tord: Integration using Eulers method, NOT trapezodial as described in the paper. 


%%% Storage, used to get plot of cost functions of individual controllers.
%%% Both instantaneous values and accumulated cost. 

yaw_u_cost.myj1(index,count) = J1;
yaw_u_cost.myj2(index,count) = J2;
yaw_u_cost.myj3(index,count) = J3;
yaw_u_cost.myj(index,count) = J; %Tord: This is the data storage for the cost function values of all controllers























