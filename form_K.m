function [K,N] = form_K(Kp_set,Ki_set,Kd_set)
% Kp_set = [10,20];
% Ki_set = [1,2];
% Kd_set = [0.1,0.2];

nKp = size(Kp_set,2);
nKi = size(Ki_set,2);
nKd = size(Kd_set,2);

N = nKp*nKi*nKd;

K = zeros(N,3);
for i = 1:nKp
    for j = 1:nKi
        for m = 1:nKd
            index = (i-1)*nKi*nKd + (j-1)*nKd + m;
            K(index,1:3) = [Kp_set(i),Ki_set(j),Kd_set(m)];
        end
    end
end
    