function [out , cost_ind] = consistency_test_detectable_windowing_fading(u , y , r , K , i , count, window , fading , root)
global u_cost delta

alpha = 0.1;
beta = 0;
gamma = 0.0;

time_elapsed    = (count-1)*delta;
window_step     = (window/delta);

% temp = (norm(y,2)+norm(u,2)+norm(r-y,2))/(norm(r,2)+alpha)+beta+gamma*norm(K,2);
temp = (norm(y,2)+norm(u,2))/(norm(r,2)+alpha)+beta+gamma*norm(K,2);%+100*norm(r-y,2);
u_cost.temp2(i,count)   = temp;


if time_elapsed < window
    time        = 0:delta:time_elapsed;
    index       = 1:count;
    J_window    = u_cost.temp2(i,index);
    weight      = fading_weight_root(time,root); % Alternative: fading_weight_exp(time,fading)
    cost        = weight.*J_window;
    [result,j]  = max(cost);
else 
    time        = 0:delta:window; % Note not really time variable, but used for weighing function
    index       = count-window_step:count;
    J_window    = u_cost.temp2(i,index);
    weight      = fading_weight_root(time,root);
    cost        = weight.*J_window;
    [result,j]  = max(cost);
end


% % % if temp > u_cost.temp2_max(i)
% % %     result              = temp;
% % %     u_cost.temp2_max(i) = temp;
% % % else 
% % %     result = u_cost.temp2_max(i);
% % % end

cost_ind    = index(j);
out         = result;
end