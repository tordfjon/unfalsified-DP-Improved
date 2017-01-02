%%   Revised 2016 by Vahid to make it work with new Matlab 2015
global u_cost delta switching_eps

fg3 = figure();clf;
step = round(u_cost.count/10);
hold on
s = [];

% Find time for when controller is changed
controller_index = simout.Data(:,1)';
diff_controller_index = [diff(controller_index),0];
controller_change_logical = diff_controller_index >= 1 | diff_controller_index <= -1;
[~,controller_change_index] = find(controller_change_logical==1);
% Including times after change
[~,col] = size(controller_change_index); 
matr = zeros(col,5);
for i = 1:col
    matr(i,1) = controller_change_index(i);
    matr(i,2) = controller_change_index(i)+1;
    matr(i,3) = controller_change_index(i)+2;
    matr(i,4) = controller_change_index(i)+3;
    matr(i,5) = controller_change_index(i)+5;
end
l = [1,2,reshape(matr',1,col*5)];
L = length(l);
for i = 1:L%i = 1:step:u_cost.count
    % Online cost for each time step
    curr_index = simout.Data(l(i),1);
    plot(u_cost.index,u_cost.myj(:,l(i)));    % Plot cost level-set
    [mi,min_index] = min(u_cost.myj(:,l(i))); % find minimum of level-set
    scat_min = scatter(min_index,mi,'r');
%     p2 = plot(  u_cost.index,u_cost.temp2(curr_index,l(i)).*ones(length(u_cost.index),1)+switching_eps,'-r', ...
%                 'LineWidth',2); % Makes line that mark the index set of which a new controller should be found
%     if l(i)-1
%         prev_index = simout.Data(l(i)-1,1);
%         p4 = plot(prev_index,u_cost.temp2(prev_index,l(i)),'*k'); % Mark previously active controller
%         p3 = plot(  u_cost.index,u_cost.temp2(prev_index,l(i)).*ones(length(u_cost.index),1)-switching_eps,'-b', ...
%                     'LineWidth',2);
%         legend([p2 p3],{sprintf('Delta ball for currently active controller at %5.2fs',simout.Time(l(i)-1,1)), ...
%                         sprintf('Index set for finding new controller at current time: %5.2fs',simout.Time(l(i),1))}, ...
%                                 'Location','best')
%     end
    p = plot(curr_index,u_cost.myj(curr_index,l(i)),'*r'); % Mark currently active controller

    set(get(get(p,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
%     set(get(get(p2,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    set(get(get(scat_min,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    s = [s;num2str(l(i)*delta,'%08.2fs')];
%     legend(s,'Location','best');
    
    pause
    if (l(i)-1) && (i-length(l))
        delete([scat_min,p])
    else
        delete([scat_min,p])
    end
end
% plot(u_cost.index,u_cost.temp2_max,'r');
% s = [s;num2str((u_cost.count-1)*delta,'%08.2fs')];
[rad,kol] = size(s);
for i=1:rad
    for j=1:kol
        if s(i,j)=='0'
            s(i,j) = ' ';
        else
            break
        end
    end
end
% legend(s,'Location','best');
hold off


%% % %% Plot fictitious reference signal
% % % 
% % % 
% % % fg4 = figure();clf;
% % % step = round(u_cost.total_number/5);
% % % hold on
% % % s = [];
% % % 
% % % for i = 1:step:u_cost.total_number
% % %     plot(fictitious_reference_signal.time,fictitious_reference_signal.data(:,step:step:end));
% % %     s = [s;num2str(i,'%04d')];
% % % end
% % % 
% % % [rad,kol] = size(s);
% % % for i=1:rad
% % %     for j=1:kol
% % %         if s(i,j)=='0'
% % %             s(i,j) = ' ';
% % %         else
% % %             break
% % %         end
% % %     end
% % % end
% % % legend(s);
% % % hold off
% % % 
% % % 
% % % clear a c count time x_time

