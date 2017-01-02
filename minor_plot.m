global u_cost
% % % kontroller = 7;
% % % figure
% % % time_hor = (u_cost.count-1)*delta;
% % % time = 0:delta:time_hor;
% % % plot(frs.Time,frs.Data(:,kontroller))
% % % grid on
% % % hold on
% % % plot(result.Time,frs.Data(:,kontroller)-result.Data(:,2))
% % % plot(result.Time,result.Data(:,4))
% % % plot(time,u_cost.myj(kontroller,:))
% % % legend('frs','frs-y','u','J')
% % % title(sprintf('Controller: %i, K_p=%3.1f K_i=%3.1f K_d=%3.1f',kontroller,u_cost.K(kontroller,1),u_cost.K(kontroller,2),u_cost.K(kontroller,3)))
% % % 

%% Plot all cost
fg_allcost = figure;
time_hor = (u_cost.count-1)*delta;
time = 0:delta:time_hor;
Leg = [];
hold on
for controller = 1:u_cost.total_number
%     hold on
    plot(time,u_cost.myj(controller,:))
    Leg = [Leg;sprintf('Cont: %3.0i, K_p=%07.1f K_i=%07.1f K_d=%07.1f',controller,u_cost.K(controller,1),u_cost.K(controller,2),u_cost.K(controller,3))];
%     hold off
end

for t = 1:10:u_cost.count
    active_controller = simout.Data(t,1);
    p = plot((t-1)*delta,u_cost.myj(active_controller,t),'ro');
%     set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
end
hold off
grid on

% Formatting the legend
[rad,kol] = size(Leg);
for i=1:rad
    for j=16:19
        if Leg(i,j)=='0'
            Leg(i,j) = ' ';
        else
            break
        end
    end
    for j=28:31
        if Leg(i,j)=='0'
            Leg(i,j) = ' ';
        else
            break
        end
    end
    for j=40:43
        if Leg(i,j)=='0'
            Leg(i,j) = ' ';
        else
            break
        end
    end
end

% legend(Leg,'Location','southoutside')












