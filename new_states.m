%%   Revised 2016 by Vahid to make it work with new Matlab 2015
function [x, new_trigger] = new_states(kp_current, ki_current, kd_current, ...
                                       kp_new, ki_new, kd_new, y, x_integrator, x_derivative, r, ...
                                       trigger, epsilon, delta)
                                    
ak1 = 0;               % current PI controller realization
bk1 = 1;
ck1 = ki_current;
dk1 = kp_current;
      
ak2 = -1/epsilon;      % current D controller realization
bk2 = 1;
ck2 = -kd_current/(epsilon^2);
dk2 = kd_current/epsilon; 

[Ak1, Bk1, Ck1, Dk1] = ssdata(c2d(ss(ak1,bk1,ck1,dk1),delta));
[Ak2, Bk2, Ck2, Dk2] = ssdata(c2d(ss(ak2,bk2,ck2,dk2),delta));
   
   
ak1_new = 0;           % next PI controller realization
bk1_new = 1;
ck1_new = ki_new;
dk1_new = kp_new;
      
ak2_new = -1/epsilon;  % next D controller realization
bk2_new = 1;
ck2_new = -kd_new/(epsilon^2);
dk2_new = kd_new/epsilon;

[Ak1_new, Bk1_new, Ck1_new, Dk1_new] = ssdata(c2d(ss(ak1_new,bk1_new,ck1_new,dk1_new),delta));
[Ak2_new, Bk2_new, Ck2_new, Dk2_new] = ssdata(c2d(ss(ak2_new,bk2_new,ck2_new,dk2_new),delta));
   
        % New integrator state
x1 = ( (Ck1-Ck1_new)*x_integrator + Dk1*(r-y) - Dk1_new*(r-y) ) / Ck1_new;
        % New derivative state
x2 = ( (Ck2-Ck2_new)*x_derivative + (Dk2-Dk2_new)*y ) / Ck2_new;
%-x_derivative;
x = [x1; x2];
      
if trigger > 0
   new_trigger = -1;
else
   new_trigger = 1;
end
