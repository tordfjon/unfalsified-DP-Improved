function result = fading_weight_root(window,root,delta)

time = 0:delta:window;
result = (time./time(end)).^(1/root);


end