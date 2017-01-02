print_figures = 'no';
filetype = '-depsc2';
folder = 'figures/';
%% Figure 1
fg = figure;
subplot(2,1,1)
plot(surge_result.Time,surge_result.Data(:,2))
hold on
plot(sway_result.Time,sway_result.Data(:,2))
plot(yaw_result.Time,yaw_result.Data(:,2))
grid on
T = title('Response');
X = xlabel('Time - [s]');
Y = ylabel('Displacement - [m] and [rad]');
L = legend('North','East','$\Psi$');
T.Interpreter = 'latex';
X.Interpreter = 'latex';
Y.Interpreter = 'latex';
L.Interpreter = 'latex';


subplot(2,1,2)
plot(surge_result.Time,surge_result.Data(:,4))
hold on
plot(sway_result.Time,sway_result.Data(:,4))
plot(yaw_result.Time,yaw_result.Data(:,4))
grid on

ax = gca;
ax.YLim = [-4,6];
T = title('Control effort');
X = xlabel('Time - [s]');
Y = ylabel('Force - [N]');
L = legend('X','Y','$\psi$');
T.Interpreter = 'latex';
X.Interpreter = 'latex';
Y.Interpreter = 'latex';
L.Interpreter = 'latex';

if print_figures=='y' || strcmp(print_figures,'yes')
    fg.PaperPositionMode = 'auto';
    filename = 'response';
    print(gcf,filetype,strcat(folder,filename));
end


%% Figure 2
fg2 = figure;
plot(Forces.Time,Forces.Data(:,7:9))
grid on
T = title('Environmental forces');
X = xlabel('Time - [s]');
Y = ylabel('Force - [N]');
L = legend('X','Y','N');
T.Interpreter = 'latex';
X.Interpreter = 'latex';
Y.Interpreter = 'latex';
L.Interpreter = 'latex';

if print_figures=='y' || strcmp(print_figures,'yes')
    fg2.PaperPositionMode = 'auto';
    filename = 'enviro';
    print(gcf,filetype,strcat(folder,filename));
end

%% Figure 3
fg3 = figure;
plot(sway_result.Data(:,2),surge_result.Data(:,2))
grid on
ax3 = gca;
ax3.XLim = [-0.2,0.2];
ax3.YLim = [-0.2,0.2];

T = title('Trace plot');
X = xlabel('East - [m]');
Y = ylabel('North - [m]');
T.Interpreter = 'latex';
X.Interpreter = 'latex';
Y.Interpreter = 'latex';

if print_figures=='y' || strcmp(print_figures,'yes')
    fg3.PaperPositionMode = 'auto';
    filename = 'trace';
    print(gcf,filetype,strcat(folder,filename));
end

%% Figure 4
fg4 = figure;
subplot(3,1,1)
plot(surge_gains.Time,surge_gains.Data(:,:))
grid on
T = title('Surge');
X = xlabel('Time - [s]');
L = legend('$K_p$','$K_i$','$K_d$');
T.Interpreter = 'latex';
X.Interpreter = 'latex';
L.Interpreter = 'latex';

subplot(3,1,2)
plot(sway_gains.Time,sway_gains.Data(:,:))
grid on
T = title('Sway');
X = xlabel('Time - [s]');
L = legend('$K_p$','$K_i$','$K_d$');
T.Interpreter = 'latex';
X.Interpreter = 'latex';
L.Interpreter = 'latex';

subplot(3,1,3)
plot(yaw_gains.Time,yaw_gains.Data(:,:))
grid on
T = title('Yaw');
X = xlabel('Time - [s]');
L = legend('$K_p$','$K_i$','$K_d$');
T.Interpreter = 'latex';
X.Interpreter = 'latex';
L.Interpreter = 'latex';

if print_figures=='y' || strcmp(print_figures,'yes')
    fg4.PaperPositionMode = 'auto';
    filename = 'gains';
    print(gcf,filetype,strcat(folder,filename));
end







