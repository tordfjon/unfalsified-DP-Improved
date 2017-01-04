function out = fading_weight_root(time,root)

check = time(end);
if check 
    result = (time./time(end)).^(1/root);

else 
    result = 0;
end

out = result;

end