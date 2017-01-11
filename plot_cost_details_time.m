%% Need update after latest changes to fading_weight_root(). See changes in consistency_test()
%% -Tord

function plot_cost_details_time(time_elapsed,window,i)
global u_cost

delta = 0.01;
count = time_elapsed/delta+1;
window_step     = (window/delta);
root = 9;

if time_elapsed < window
    time        = 0:delta:time_elapsed;
    index       = 1:count;
    J_window    = u_cost.temp(i,index);
    weight      = fading_weight_root(window,root); % Alternative: fading_weight_exp(time,fading)
    cost        = weight.*J_window;
    [result,j]  = max(cost);
else 
    time        = 0:delta:window; % Note not really time variable, but used for weighing function
    index       = count-window_step:count;
    J_window    = u_cost.temp(i,index);
    weight      = fading_weight_root(window,root);
    cost        = weight.*J_window;
    [result,j]  = max(cost);
end
weight2 = [zeros(1,j-1),fliplr(weight(j:end))];
max_cost = [result.*weight2];
%% ploting result
fg = figure; clf(fg);
p1 = plot(time,J_window,time,cost);
hold on
vline = line([0,0],[0,J_window(j)]);
hline1 = line([0,time(end)],[cost(end),cost(end)]);
hline2 = line([0,time(end)],[result,result]);
p2 = plot(time,max_cost);
end






















