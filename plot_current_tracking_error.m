function plot_current_tracking_error(ned_pos,ned_ref)
figure()
ax = axes('Position',[0.1,0.1,0.8,0.8]);
psi         = ned_pos(3);
psid        = psi*180/pi;
R           = Rpmm(psi);
Plot_range  = [-5,5];
Grid_tick   = Plot_range(1):1:Plot_range(2);
e_n         = ned_ref - ned_pos;
e_b         = R'*e_n;

x = [1,0,0];
y = [0,1,0];
z = [0,0,1];
l1 = line([ned_pos(2),ned_pos(2)+3],[ned_pos(1),ned_pos(1)],'LineWidth',1,'Color','g');
l2 = line([ned_pos(2),ned_pos(2)],[ned_pos(1),ned_pos(1)+3],'LineWidth',1,'Color','g');
rotate(l1,z,-psid,[ned_pos(2),ned_pos(1),0])
rotate(l2,z,-psid,[ned_pos(2),ned_pos(1),0])
ax.XLim = Plot_range; ax.XTick = Grid_tick; ax.XGrid = 'on';
ax.YLim = Plot_range; ax.YTick = Grid_tick; ax.YGrid = 'on';
ax.PlotBoxAspectRatio = [1,1,1];
hold on
plot(ax,ned_ref(2),ned_ref(1),'or');
% quiver(ax,ned_pos(2),ned_pos(1),e_n(2),e_n(1),0)
le_b = line([ned_pos(2),ned_pos(2)+e_b(2)],[ned_pos(1),ned_pos(1)+e_b(1)]);
rotate(le_b,z,-psid,[ned_pos(2),ned_pos(1),0])
line(Plot_range,[0,0],'LineWidth',1,'Color','k');
line([0,0],Plot_range,'LineWidth',1,'Color','k');
[vessel_x,vessel_y] = graphic_vessel(ned_pos(2),ned_pos(1),psi,1);
plot(vessel_x,vessel_y,'k')
hold off

end