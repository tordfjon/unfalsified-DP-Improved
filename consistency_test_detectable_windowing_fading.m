function out = consistency_test_detectable_windowing_fading(u , y , r , K , i , count, window , fading)
global u_cost delta

alpha = 0.1;
beta = 0;
gamma = 0.0;

time_elapsed    = (count-1)*delta;
window_step     = (window/delta);
% temp = (norm(y,2)+norm(u,2)+norm(r-y,2))/(norm(r,2)+alpha)+beta+gamma*norm(K,2);
temp = (norm(y,2)+norm(u,2))/(norm(r,2)+alpha)+beta+gamma*norm(K,2);%+100*norm(r-y,2);
u_cost.temp2(i,count) = temp;


if time_elapsed < window
    index       = 1:count;
    J_window    = u_cost.temp2(i,index);
    weight      = fliplr(exp(-fading*(0:delta:time_elapsed)));
    cost        = weight.*J_window;
    result      = max(cost);
else 
    index       = count-window_step:count;
    J_window    = u_cost.temp2(i,index);
    weight      = fliplr(exp(-fading*(0:delta:window))); %Problem
    cost        = weight.*J_window;
    result      = max(cost);
end


% % % if temp > u_cost.temp2_max(i)
% % %     result              = temp;
% % %     u_cost.temp2_max(i) = temp;
% % % else 
% % %     result = u_cost.temp2_max(i);
% % % end


out = result;
end