%%   Revised 2016 by Vahid to make it work with new Matlab 2015
function [kp, ki, kd] = set_K_parameter(K,index,m)

kp = K(index(m),1);
ki = K(index(m),2);
kd = K(index(m),3);