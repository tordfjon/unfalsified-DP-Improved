function [out , cost_ind] = consistency_test_detectable_windowing_fading(u , y , r , K , i , count, window , fading , root)
% Persistent variables seen not to get reset when new simulation starts. This is a stupid quick fix

global u_cost delta
persistent weight
memory_function = 1;
if count == 1
    switch memory_function
        case 1
            weight.full     = fading_weight_root(window,root,delta);
        case 2
            weight.full     = fading_weight_exp(window,root,delta); % Function need update. See structure of fading_weight_root();
    end
    weight.short    = 1;
end

alpha = 0.1;
beta = 0;
gamma = 0.0;

time_elapsed    = (count-1)*delta;
window_step     = (window/delta);

% temp = (norm(y,2)+norm(u,2)+norm(r-y,2))/(norm(r,2)+alpha)+beta+gamma*norm(K,2);
temp = (norm(y,2)+norm(u,2))/(norm(r,2)+alpha)+beta+gamma*norm(K,2);%+100*norm(r-y,2);
u_cost.temp(i,count)   = temp;


if time_elapsed < window
%     time        = 0:delta:time_elapsed;
    index       = 1:count;
    J_window    = u_cost.temp(i,index);
    if count==1
        % Use initialized value
    else
        weight.short  = weight.full(end-(count-1):end);
    end
    cost        = weight.short.*J_window;
    [result,j]  = max(cost);
else 
%     time        = 0:delta:window; % Note not really time variable, but used for weighing function
    index       = count-window_step:count;
    J_window    = u_cost.temp(i,index);
    cost        = weight.full.*J_window;
    [result,j]  = max(cost);
end

cost_ind    = index(j);
out         = result;
end
























