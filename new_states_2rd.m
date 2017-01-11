%%   Revised 2016 by Vahid to make it work with new Matlab 2015
function [new_slow_state , new_fast_state , new_trigger] = new_states_2rd(kp_current, ki_current, kd_current, ...
                                       kp_new, ki_new, kd_new, y, r, ...
                                       u_prev , trigger, epsilon, delta)

xi = 0; % Cjs*xi = 0, i.e. xi in null spcae of Cjs.
                                   
ak1 = -0;               % current, i, SLOW, s, controller realization
bk1 = ki_current;
ck1 = 1;
dk1 = kp_current;
      
ak2 = -1/epsilon;      % current, i, FAST, f, controller realization
bk2 = 1;
ck2 = -kd_current/(epsilon^2);
dk2 = kd_current/epsilon; 

[Ais, Bis, Cis, Dis] = ssdata(c2d(ss(ak1,bk1,ck1,dk1),delta));
[Aif, Bif, Cif, Dif] = ssdata(c2d(ss(ak2,bk2,ck2,dk2),delta));
   
   
ak1_new = 0;           % next, j, SLOW, s, controller realization
bk1_new = ki_new;
ck1_new = 1;
dk1_new = kp_new;
      
ak2_new = -1/epsilon;  % next, j, FAST, f, controller realization
bk2_new = 1/epsilon;
ck2_new = -kd_new/(epsilon);
dk2_new = kd_new/epsilon;

[Ajs, Bjs, Cjs, Djs] = ssdata(c2d(ss(ak1_new,bk1_new,ck1_new,dk1_new),delta));
[Ajf, Bjf, Cjf, Djf] = ssdata(c2d(ss(ak2_new,bk2_new,ck2_new,dk2_new),delta));


%%%% Working! 10.01.2016
Cjs_pi = Cjs'/(Cjs*Cjs'); % Pseudo-inverse for nonsquare C-matrix

new_slow_state = Cjs_pi*(u_prev - Djs*(r-y) + xi) ;
new_fast_state = epsilon*(r-y);

if trigger > 0
   new_trigger = -1;
else
   new_trigger = 1;
end

end













