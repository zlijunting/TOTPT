close all

veh = veh_params();

% track_name = 'Catalunya';
% Tracks: Austin, BrandsHatch, Budapest, Catalunya, Hockenheim, IMS, 
% Melbourne, MexicoCity, Montreal, Monza, MoscowRaceway, Norisring, 
% Nuerburgring, Oschersleben, Sakhir, SaoPaulo, Sepang, Shanghai, 
% Silverstone, Sochi, Spa, Spielberg, Suzuka, YasMarina, Zandvoort

track = readtable(['tracks_smooth\' track_name '_smooth.csv']);
track_opt = readtable(['tracks_mintime\' track_name '_mintime' '.csv']);

load(['tro_sol\' 'sol_' track_name '.mat'])


%% Default Plot Configurations
set(0, 'DefaultLineLineWidth', 1);
set(groot,'defaultAxesXGrid','on')
set(groot,'defaultAxesYGrid','on')


%% State Variables
s_end = track.s(end);

% f1 = figure('Position', [20, 50, 1800, 300*3]);
f1 = figure;
fig_states = tiledlayout(3,2,'TileSpacing','Compact','Padding','Compact');

nexttile
plot(track.s,sol.x_opt(1,:)), grid on, hold on
ylabel('v  [m/s]')
xlim([0 s_end])

nexttile
plot(track.s,sol.x_opt(2,:)), grid on
ylabel('\beta  [rad]')
xlim([0 s_end])

nexttile
plot(track.s,sol.x_opt(3,:)), grid on
ylabel('\gamma  [rad/s]')
xlim([0 s_end])

nexttile
plot(track.s,sol.x_opt(4,:)), grid on
ylabel('n  [m]')
xlim([0 s_end])

nexttile
plot(track.s,sol.x_opt(5,:)), grid on
ylabel('\xi  [rad]')
xlim([0 s_end])
xlabel('s [m]')

nexttile
plot(track.s,sol.x_opt(6,:),'-'), grid on, hold on
plot(track.s,sol.x_opt(7,:),'--')
plot(track.s,sol.x_opt(8,:),'-.')
plot(track.s,sol.x_opt(9,:),':')
ylabel('[m/s]')
legend('\omega_{fl}','\omega_{fr}','\omega_{rl}','\omega_{rr}')
xlim([0 s_end])
xlabel('s [m]')

sgtitle('State Variables')


%% Control and Auxiliary Variables
f2 = figure;
fig_control = tiledlayout(2,2,'TileSpacing','Compact','Padding','Compact');

nexttile
plot(track.s,sol.u_opt(1,:)), grid on, hold on
plot(track.s,sol.u_opt(2,:))
ylabel('T_t/T_b  [N-m]')
xlim([0 s_end])

nexttile
plot(track.s,sol.u_opt(3,:)), grid on,  
ylabel('\delta  [rad]')
xlim([0 s_end])

nexttile
plot(track.s,sol.z_opt(1,:)), grid on
ylabel('$\bar{a}_{x}\;  [\textrm{ms}^{-2}]$','Interpreter','latex')
xlim([0 s_end])
xlabel('s [m]')

nexttile
plot(track.s,sol.z_opt(2,:)), grid on
ylabel('$\bar{a}_{y}\;  [\textrm{ms}^{-2}]$','Interpreter','latex')
xlim([0 s_end])
xlabel('s [m]')

sgtitle('Control and Auxiliary Variables')


%% Output Variables
f3 = figure;
fig_output = tiledlayout(3,3,'TileSpacing','tight','Padding','tight');

nexttile
plot(track.s,sol.tyre(:,1),'-'), grid on, hold on
plot(track.s,sol.tyre(:,2),'--'),
plot(track.s,sol.tyre(:,3),'-.'),
plot(track.s,sol.tyre(:,4),':')
ylabel('[N]')
xlim([0 s_end])
legend('Fx_{fl}','Fx_{fr}','Fx_{rl}','Fx_{rr}')

nexttile
plot(track.s,sol.tyre(:,5),'-'), grid on, hold on
plot(track.s,sol.tyre(:,6),'--'),
plot(track.s,sol.tyre(:,7),'-.'),
plot(track.s,sol.tyre(:,8),':')
ylabel('[N]')
xlim([0 s_end])
legend('Fy_{fl}','Fy_{fr}','Fy_{rl}','Fy_{rr}')

nexttile
plot(track.s,sol.tyre(:,9),'-'), grid on, hold on
plot(track.s,sol.tyre(:,10),'--'),
plot(track.s,sol.tyre(:,11),'-.'),
plot(track.s,sol.tyre(:,12),':')
ylabel('[N]')
xlim([0 s_end])
legend('Fz_{fl}','Fz_{fr}','Fz_{rl}','Fz_{rr}')

nexttile
plot(track.s,sol.slip(:,1),'-'), grid on, hold on
plot(track.s,sol.slip(:,2),'--'),
plot(track.s,sol.slip(:,3),'-.'),
plot(track.s,sol.slip(:,4),':')
ylabel('[-]')
xlim([0 s_end])
legend('\lambda_{fl}','\lambda_{fr}','\lambda_{rl}','\lambda_{rr}')

nexttile
plot(track.s,sol.slip(:,5),'-'), grid on, hold on
plot(track.s,sol.slip(:,6),'--'),
plot(track.s,sol.slip(:,7),'-.'),
plot(track.s,sol.slip(:,8),':')
ylabel('[rad]')
xlim([0 s_end])
legend('\alpha_{fl}','\alpha_{fr}','\alpha_{rl}','\alpha_{rr}')

nexttile
plot(track.s,sol.acc(:,1)), grid on, hold on
plot(track.s,sol.acc(:,2))
ylabel('[m/s^2]')
xlim([0 s_end])
legend('a_x','a_y')

nexttile
plot(track.s,sol.torque(:,1),'-'), grid on, hold on
plot(track.s,sol.torque(:,2),'--'),
plot(track.s,sol.torque(:,3),'-.'),
plot(track.s,sol.torque(:,4),':')
ylabel('[N-m]')
xlim([0 s_end])
legend('T_{fl}','T_{fr}','T_{rl}','T_{rr}')
xlabel('s [m]')


nexttile
plot(track.s,sol.mu(:,1),'-'), grid on, hold on
plot(track.s,sol.mu(:,2),'--'),
plot(track.s,sol.mu(:,3),'-.'),
plot(track.s,sol.mu(:,4),':')
ylabel('[-]')
xlim([0 s_end])
legend('$\hat{\mu}_{fl}$','$\hat{\mu}_{fr}$','$\hat{\mu}_{rl}$','$\hat{\mu}_{rr}$','interpreter','latex')
xlabel('s [m]')

nexttile
plot(track.s,sol.aero(:,2)), grid on, hold on
plot(track.s,sol.aero(:,1))
ylabel('[N]')
xlim([0 s_end])
legend('F_{drag}','F_{lift}')
xlabel('s [m]')

sgtitle('Output Variables')

%% Tire Workloads
titletxt = {'fl', 'fr', 'rl', 'rr'};
figure
hat_mux = sol.tyre(:,1:4)./sol.tyre(:,9:12);
hat_muy = sol.tyre(:,5:8)./sol.tyre(:,9:12);
for i = 1:4
    [x_ellipse, y_ellipse] = calc_ellipse_coords(0, 0, veh.mux, veh.muy);

    subplot(2,2,i)
    plot(x_ellipse, y_ellipse), hold on, grid on
    scatter(hat_muy(:,i),hat_mux(:,i),10)
    xlabel(['\mu_{y' titletxt{i} '}'])
    ylabel(['\mu_{x' titletxt{i} '}'])
    axis equal
    axis([-1.1 1.1 -1.1 1.1])
    title(titletxt(i))
end

sgtitle('Tire Workloads')


%% Path Tracking Global

n_track = [-sin(track.phi) cos(track.phi)];
trackbnd_l = [track.x track.y] + track.wl.*n_track;
trackbnd_r = [track.x track.y] - track.wr.*n_track;

f4 = figure;
plot(trackbnd_l(:,1),trackbnd_l(:,2),'-k'), grid on, hold on
plot(trackbnd_r(:,1),trackbnd_r(:,2),'-k')

scatter(sol.track_opt(:,1),sol.track_opt(:,2),5,sol.x_opt(1,:)*3.6), hold on

plot(track_opt.x,track_opt.y,'Color',[0.3 0.3 0.3])
cb=colorbar;
ylabel(cb,'V [km/h]','FontSize',12)

axis equal

xlabel('X [m]')
ylabel('Y [m]')
legend('track boundary')

title('Path Tracking Global')


%% Optimal Lateral Deviation
f5 = figure;
plot(track.s,-track.wr,'-k'), grid on, hold on
plot(track.s,track.wl,'-k')
plot(track.s,-track.wr+veh.wt/2+veh.ws,'-.k')
plot(track.s,track.wl-veh.wt/2-veh.ws,'-.k')
plot(track.s,sol.x_opt(4,:),'r')

xlim([0 s_end])
xlabel('s [m]')
ylabel('lateral displacement [m]')
legend('real track boundaries','','safe track boundaries','','n^{*}')
title('Optimal Lateral Deviation')


%% Path Tracking Curvilinear
% reference heading and curvature
[phi, kappa] = calc_head_curv_num([track_opt.x track_opt.y]);
f6 = figure;

subplot(2,1,1)
plot(track.s,phi), grid on
ylabel('\phi_{r} [rad]')
xlim([0 s_end])

subplot(2,1,2)
plot(track.s,kappa), grid on
ylabel('\kappa_r [1/m]')
xlim([0 s_end])
xlabel('s [m]')

sgtitle('Path Tracking Curvilinear')


%% Saving Plots
if ~exist(fullfile(pwd,'figures'), 'dir')
   mkdir('figures')
end
saveas(f1,'figures/states.png')
saveas(f2,'figures/controls.png')
saveas(f3,'figures/outputs.png')
saveas(f4,'figures/tracking_global.png')
saveas(f5,'figures/n_star.png')
saveas(f6,'figures/tracking_curv.png')
