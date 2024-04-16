close all

% track_name = 'Catalunya';
% Tracks: Austin, BrandsHatch, Budapest, Catalunya, Hockenheim, IMS, 
% Melbourne, MexicoCity, Montreal, Monza, MoscowRaceway, Norisring, 
% Nuerburgring, Oschersleben, Sakhir, SaoPaulo, Sepang, Shanghai, 
% Silverstone, Sochi, Spa, Spielberg, Suzuka, YasMarina, Zandvoort

N = 30;
T = 1.5; % time horizon length
Ts = T/N;

out_file = ['simout\' 'out_' track_name];
load(out_file)


%% Specify colors
b_rgb = [0 0.4470 0.7410];
r_rgb = [0.8500 0.3250 0.0980];
gray_rgb = [0.7 0.7 0.7];

set(0, 'DefaultLineLineWidth', 1);
set(groot,'defaultAxesXGrid','on')
set(groot,'defaultAxesYGrid','on')

%% Plot States
s_end = out.s_sim(end);
figure('Units','pixels','Position',[10 50 1800 800])
tiledlayout(2,2,'TileSpacing','compact','Padding','compact')

nexttile
plot(out.s_sim,out.V_ref*3.6,'k:'), hold on
plot(out.s_sim,out.V_sim*3.6,'Color',b_rgb)
ylabel('V [km/h]')
xlim([0 s_end])

nexttile
plot(out.s_sim,out.beta_sim,'Color',b_rgb)
ylabel('\beta [rad]')
xlim([0 s_end])

nexttile
plot(out.s_sim,out.gamma_sim,'Color',b_rgb)
ylabel('\gamma [rad/s]')
xlim([0 s_end])

xlabel('s [m]')

nexttile
plot(out.s_sim,out.ax_sim,'Color',b_rgb), hold on
plot(out.s_sim,out.ay_sim,'Color',r_rgb)
ylabel('acc. [m/s^2]'), legend('a_{x}','a_{y}')
xlim([0 s_end])

xlabel('s [m]')

sgtitle('Vehicle States')

%% Inputs and it's derivative
figure('Units','pixels','Position',[10 50 1800 800])
tiledlayout(2,2,'TileSpacing','compact','Padding','compact','TileIndexing','columnmajor')
size_u = size(out.U_star);
length_u = size_u(3);
u1_dot = reshape(out.U_star(1,1,:),1,length_u);
u2_dot = reshape(out.U_star(1,2,:),1,length_u);
u3_dot = reshape(out.U_star(1,3,:),1,length_u);

nexttile
plot(out.t,out.Tt,'Color',b_rgb), hold on
plot(out.t,out.Tb,'Color',r_rgb)
ylabel('Torque [N-m]')
legend('T_{t}','T_{b}')
xlim([0 out.t(end)])

nexttile
plot(out.t,out.delta*180/pi,'Color',b_rgb)
ylabel('\delta [deg]')
xlim([0 out.t(end)])

nexttile
plot(out.s_sim,u1_dot,'Color',b_rgb), hold on
plot(out.s_sim,u2_dot,'Color',r_rgb)
legend('$\dot{T}_{t}$','$\dot{T}_{b}$','Interpreter','latex')
ylabel('$[\textrm{Nm} \; \textrm{s}^{-1}]$','Interpreter','latex')
xlim([0 s_end])

xlabel('s [m]')


nexttile
plot(out.s_sim,u3_dot,'Color',b_rgb)
ylabel('$\dot{\delta}_{\textrm{cmd}} \; [\textrm{deg} \; \textrm{s}^{-1}]$','Interpreter','latex')
xlim([0 s_end])

xlabel('s [m]')

sgtitle('Driving Inputs')


%% Path Tracking Performance

[track_bnd_r, track_bnd_l] = calc_track_bnd([xr yr wr wl]);

figure('Units','pixels','Position',[10 50 1800 800])
tiledlayout(2,4,'TileSpacing','compact','Padding','compact')
sgtitle('Path Tracking Performance')

nexttile([2 2])
plot(track_bnd_r(:,1),track_bnd_r(:,2),'-','Color', gray_rgb), hold on
plot(track_bnd_l(:,1),track_bnd_l(:,2),'-','Color', gray_rgb)

sz = 4;
scatter(out.X_sim,out.Y_sim,sz,out.Vx_sim*3.6), hold on

% plot(xr,yr,'LineWidth',2.0,'Color', gray_rgb), hold on
plot(xr(1:index_max),yr(1:index_max),'-','LineWidth',0.5,'Color', [0.4 0.4 0.4])

xlabel('X [m]')
ylabel('Y [m]')
xlim([0 s_end])

cb=colorbar;
ylabel(cb,'V [km/h]','FontSize',12)
axis([min(xr)-50 max(xr)+50 min(yr)-50 max(yr)+50])
axis equal


nexttile([1 2])
plot(out.s,out.n), hold on
ed_rms = rms(out.n);
% yline(ed_rms,'LineWidth',1.5)
ylim([min(out.n)-0.1 max(out.n)+0.1])
xlabel('S [m]')
ylabel('$n\;[\textrm{m}]$','Interpreter','latex','FontSize',15)
% legend('Data','RMS')
ylim([-0.2 0.2])
xlim([0 s_end])

nexttile([1 2])
plot(out.s,(out.xi+out.beta_sim)*180/pi), hold on
chi_rms = rms(out.xi+out.beta_sim)*180/pi;
% yline(epsi_rms,'LineWidth',1.5)
% ylim([-4 4])
ylim([min(out.xi+out.beta_sim)*180/pi-0.5 max(out.xi+out.beta_sim)*180/pi+0.5])
xlabel('S [m]')
ylabel('$\chi \;[deg]$','Interpreter','latex','FontSize',15)
% legend('Data','RMS')
xlim([0 s_end])

% figure
% chi = out.beta_sim + out.xi;
% plot(out.s_sim, chi), grid on, hold on
% plot(out.s_sim, out.beta_sim)
% plot(out.s_sim, out.xi)
% legend('\chi','\beta','\xi')


%% ggv diagram

figure('Units','pixels')
g = 9.81;
Ax = out.ax_sim/g;
Ay = out.ay_sim/g;
downsample_n = 10;
Ax = downsample(Ax, downsample_n);
Ay = downsample(Ay, downsample_n);
A = sqrt(Ax.^2+Ay.^2);

Vx_contour = downsample(out.Vx_sim, downsample_n);

% scatter(Ay,Ax,[],A,'filled')
scatter(Ay,Ax,[],Vx_contour)
cb=colorbar;
% cb.Title.String = "A [g]";
% ylabel(cb,'$||\vec{\textbf{a}}|| \; [\rm{g}]$','Interpreter','latex','FontSize',15)
ylabel(cb,'$V_x \; [\rm{m/s}]$','Interpreter','latex','FontSize',15)
axis([-1 1 -1 1])
% axis equal
xlabel('a_y [g]')
ylabel('a_x [g]')


%% Wheel Torques
figure
plot(out.s_sim,out.inputs(:,1:4)), hold on
xlabel('S [m]')
ylabel('T [N-m]')
xlim([0 s_end])
legend('T_{FL}','T_{FR}','T_{RL}','T_{RR}')


%% Tire Ellipse

sz = 4;
downsample_n = 5;
Fx = downsample(out.Fx_sim, downsample_n);
Fy = downsample(out.Fy_sim, downsample_n);
Fz = downsample(out.Fz_sim, downsample_n);

theta_circ = 0:pi/50:2*pi;
r_circ= 1.0;
x_circ = r_circ*cos(theta_circ);
y_circ = r_circ*sin(theta_circ);

xlabel_text = {'\mu_{yfl}','\mu_{yfr}','\mu_{yrl}','\mu_{yrr}'};
ylabel_text = {'\mu_{xfl}','\mu_{xfr}','\mu_{xrl}','\mu_{xrr}'};

figure
for i=1:4
    subplot(2,2,i)
    mu_x = Fx(:,i)./Fz(:,i);
    mu_y = Fy(:,i)./Fz(:,i);
    scatter(mu_y,mu_x,sz,'filled'), hold on
    plot(x_circ,y_circ,'r','LineWidth',0.5)
    xlabel(xlabel_text{i}), ylabel(ylabel_text{i})
    axis([-1.1 1.1 -1.1 1.1])
    axis square
end

sgtitle('Tire Ellipse')




