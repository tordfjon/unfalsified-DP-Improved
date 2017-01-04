function out = fading_weight_exp(time , fading)

result  = fliplr(exp(-fading.*(time)));
out     = result;

end