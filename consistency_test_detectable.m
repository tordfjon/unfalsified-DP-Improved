function out = consistency_test_detectable(u,y,r,K,i,count)
global u_cost

alpha = 0.1;
beta = 0;
gamma = 0.0;

% temp = (norm(y,2)+norm(u,2)+norm(r-y,2))/(norm(r,2)+alpha)+beta+gamma*norm(K,2);
temp = (norm(y,2)+norm(u,2))/(norm(r,2)+alpha)+beta+gamma*norm(K,2);%+100*norm(r-y,2);
if temp > u_cost.temp2_max(i)
    result              = temp;
    u_cost.temp2_max(i) = temp;
else 
    result = u_cost.temp2_max(i);
end
u_cost.temp2(i,count) = result;

out = result;
end