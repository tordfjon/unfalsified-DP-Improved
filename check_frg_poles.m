syms kp ki kd s e
global epsilon delta
frg_roots = solve((kd+kp*e)*s^2+(kp+ki*e)*s+ki==0,s);
frg_roots = solve((kd+kp*e)*s^2+(kp+ki*e)*s+ki==0,s);

n = size(K,1);
index = (1:n);
frg_poles = zeros(n,2);
for i = 1:n
    [cp,ci,cd] = set_K_parameter(K,index,i);
    e = epsilon; kp = cp; ki = ci; kd = cd;
    frgi_pole = subs(frg_roots);
    frg_poles(i,1:2) = [eval(frgi_pole(1)) eval(frgi_pole(2))];
end

frg_poles