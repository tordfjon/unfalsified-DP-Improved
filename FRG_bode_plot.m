%% Working script that plots bode-plot for the FRG transformation of the falsification algorithm. 
% The script plots all bode-plots in one figure. For individual figures
% change the for-loop parameters. 
%% Written by Tord F. Onstein (tordfjon@gmail.com) Oct. 2016. 

global surge_u_cost sway_u_cost yaw_u_cost delta epsilon

motion = 'surge';
s = [];
switch motion
    case 'surge'
        bode_surge = figure; clf;
        hold on
        for i = 3:3%surge_u_cost.total_number
            [kp,ki,kd] = set_K_parameter(surge_u_cost.K,surge_u_cost.index, i);
            s = [s;sprintf('G%i_u',i);sprintf('G%i_y',i)];
            controller1 = tf(kp,1) + tf(ki,[1 0]);
            controller2 = tf([kd 0],[epsilon 1]);
            cont_ss2 = [0, 1] + controller1 \ [1, ss(controller2)];
            cont = c2d(cont_ss2, delta);
            [a,b,c,d] = ssdata(ss(cont));
            [G1_num,G1_den] = ss2tf(a,b,c,d,1);
            [G2_num,G2_den] = ss2tf(a,b,c,d,2);
            G1 = tf(G1_num,G1_den,delta);
            G2 = tf(G2_num,G2_den,delta);
 
            hold on
            bode(G1,G2)
            grid on
            hold off
            
        end
        legend(s)
    case 'sway'
        bode_sway = figure; clf;
        hold on
        for i = 3:3%sway_u_cost.total_number
            [kp,ki,kd] = set_K_parameter(sway_u_cost.K,sway_u_cost.index, i);
            s = [s;sprintf('G%i_u',i);sprintf('G%i_y',i)];
            controller1 = tf(kp,1) + tf(ki,[1 0]);
            controller2 = tf([kd 0],[epsilon 1]);
            cont_ss2 = [0, 1] + controller1 \ [1, ss(controller2)];
            cont = c2d(cont_ss2, delta);
            [a,b,c,d] = ssdata(ss(cont));
            [G1_num,G1_den] = ss2tf(a,b,c,d,1);
            [G2_num,G2_den] = ss2tf(a,b,c,d,2);
            G1 = tf(G1_num,G1_den,delta);
            G2 = tf(G2_num,G2_den,delta);
 
            hold on
            bode(G1,G2)
            grid on
            hold off
            
        end
        legend(s)
    case 'yaw'
        bode_yaw = figure; clf;
        hold on
        for i = 3:3%sway_u_cost.total_number
            [kp,ki,kd] = set_K_parameter(yaw_u_cost.K,yaw_u_cost.index, i);
            s = [s;sprintf('G%i_u',i);sprintf('G%i_y',i)];
            controller1 = tf(kp,1) + tf(ki,[1 0]);
            controller2 = tf([kd 0],[epsilon 1]);
            cont_ss2 = [0, 1] + controller1 \ [1, ss(controller2)];
            cont = c2d(cont_ss2, delta);
            [a,b,c,d] = ssdata(ss(cont));
            [G1_num,G1_den] = ss2tf(a,b,c,d,1);
            [G2_num,G2_den] = ss2tf(a,b,c,d,2);
            G1 = tf(G1_num,G1_den,delta);
            G2 = tf(G2_num,G2_den,delta);
 
            hold on
            bode(G1,G2)
            grid on
            hold off
            
        end
        legend(s)
    otherwise
        %Do nothing
end