%%   Revised 2016 by Vahid to make it work with new Matlab 2015
function [J, xfw1, xfw2] = consistency_test(aw1, bw1, cw1, dw1, xw1, ...
                                            aw2, bw2, cw2, dw2, xw2, ...
                                            u, y, r, ...
                                            J_prev, delta, index,count)
global u_cost
xfw1 = (aw1*xw1 + bw1*(r - y))';
j1   = cw1*xw1 + dw1*(r - y); %Tord: Calculation of filter1 with input (r-y) state space output. Equation (9) says (r-y)

xfw2 = (aw2*xw2 + bw2*u)';
j2   = cw2*xw2 + dw2*u; %Tord: Calculation of filter2 with input u state space output

j3   = r;

temp = j1^2 + j2^2 -j3^2; %Tord: This is the evaluated cost function according to (9) in the paper. See notes in OneNote

J = J_prev + temp*delta; %Tord: Integration using Eulers method, NOT trapezodial as described in the paper. 


%%% Storage, used to get plot of cost functions of individual controllers.
%%% Both instantaneous values and accumulated cost. 
u_cost.myj(index,count) = J; %Tord: This is the data storage for the cost function values of all controllers
u_cost.temp(index,count) = temp; %Tord: Data storage of all instantaneous cost of all controllers. 