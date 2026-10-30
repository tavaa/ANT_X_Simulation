%% MPCSimulationLoop_test
% MPC closed loop: MATLAB (plant = ode45) vs drone (plant = Gazebo)
% Reference NLP (PWC), IC = hover sul primo punto della flatness map.
% Il QP del tick k parte da x^(t_{k+1}) e il setpoint si invia al tick k+1.
% Frames: NED / FRD, u = [T; L; M; N]

clear; 
clc; 
close all;

%% PATHS
addpath('models');
addpath('flatness');
addpath('configs');
addpath('trajectories');
addpath('utils');
addpath('deployment');
addpath('control/MPC');

%% SHAPE
SHAPE = 'CircleDeployment';      % 'Circle' | 'Lemniscate' | 'Spiral' | 'Lissajous'


%% SETUP
ns = '/antx';

mp = ModelParameters;
cp = ControlParameters;
tp = TrajectoryParameters;
Ts = cp.Ts;

model   = A02_SimplifiedModel_Deployment(mp);
mpc_sim = A02_MPCController(model);     % MPC per la simulazione MATLAB
mpc_drn = A02_MPCController(model);     % MPC per il drone
ode_opt = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);

u_hover     = [mp.m * mp.g; 0; 0; 0];
max_tilt    = deg2rad(35);
max_pos_err = 1.5;

% Warm-up OSQP (check feasibility) -> dovrebbe dare feasible
x_w = zeros(12, 1);
for m = {mpc_sim, mpc_drn}
    m{1}.init(x_w, zeros(12, cp.p + 2), repmat(u_hover, 1, cp.p + 2));
    m{1}.solve(x_w, zeros(12, cp.p + 1), repmat(u_hover, 1, cp.p));
end


%% 1. GENERA REFERENCE NLP 
[traj_obj, T_end, periodic] = makeShape(SHAPE, tp);
[x_ref, u_ref, t_ref, info_ref] = A02_GeneratePWC_reference(traj_obj, T_end, Ts, ...
    struct('periodic', periodic, 'print_level', 0));

N   = size(x_ref, 2) - 1;
t_k = (0:N) * Ts;

x_hover = x_ref(:, end);
x_hover([4:6 7:8 10:12]) = 0;

% IC: hover sul primo punto della flatness (posizione e yaw a t = 0)
[s0, ds0, dds0, ddds0, dddds0] = traj_obj.get_flat_outputs(0);
x_fl0 = A02_FlatnessMap.map(mp, s0, ds0, dds0, ddds0, dddds0);
p_ic   = x_fl0(1:3);
psi_ic = atan2(sin(x_fl0(9)), cos(x_fl0(9)));
x_ic   = [p_ic; 0; 0; 0; 0; 0; psi_ic; 0; 0; 0]; % IC iniziale sul primo x_ref campionato


%% 2. MATLAB: MPC + plant ode45
X_matlab = nan(12, N+1);
U_matlab = nan(4, N);
ts_matlab = nan(1, N);

X_matlab(:, 1) = x_ic;
mpc_sim.init(x_ic, x_ref(:, 2:end), u_ref(:, 2:end));
u_applied = u_hover;

% loop receding horizon MATLAB
for k = 1:N
    [xw, uw] = refWindow(x_ref, u_ref, k + 1, cp.p, x_hover, u_hover);
    [u_k, x_hat1, ~, ts_matlab(k)] = mpcStep(mpc_sim, model, ode_opt, ... 
        X_matlab(:, k), Ts, u_applied, xw, uw, Ts, tic, Inf);

    X_matlab(:, k+1) = x_hat1;      % plant ideale: stato a t_{k+1}
    U_matlab(:, k)   = u_k;
    u_applied        = u_k;
end


%% 3. DRONE: MPC + plant Gazebo
X_drone  = nan(12, N+1);
t_drone  = nan(1, N+1);
U_drone  = nan(4, N);
ts_drone = nan(1, N);
 
% primo input applicato
u_applied = u_hover;
sp_next   = [p_ic; psi_ic];
 
% Drone setup
Drone.startup_sitl();
drone = Drone(ns);
drone.connect();
drone.arm();
 
% manda il drone sulla IC
drone.takeoff(p_ic(3));
drone.sendPositionSetpoint(p_ic, psi_ic);
 
x0 = measureState(drone);
mpc_drn.init(x0, x_ref(:, 2:end), u_ref(:, 2:end));
 
tic_start = tic;

% receding horizon
for k = 1:N

   maintain_timing(tic_start, t_k(k));
   
   % Invio del setpoint calcolato al tick precedente
   drone.sendPositionSetpoint(sp_next(1:3), sp_next(4));
   
   % Misura
   x_meas = measureState(drone);
   t_meas = toc(tic_start);
   X_drone(:, k) = x_meas;
   t_drone(k)    = t_meas;
   
   % controlla inclinazione ed errore di posizione
   if acos(cos(x_meas(7)) * cos(x_meas(8))) > max_tilt
      error('Abort: tilt > %.0f deg a t = %.2f s', rad2deg(max_tilt), t_k(k));
   end
   if norm(x_meas(1:3) - x_ref(1:3, k)) > max_pos_err
      error('Abort: errore di posizione > %.2f m a t = %.2f s', max_pos_err, t_k(k));
   end

   % Predizione, QP, setpoint successivo
   % ottieni la ref
   [xw, uw] = refWindow(x_ref, u_ref, k + 1, cp.p, x_hover, u_hover);
   % t. di integrazione (t(prossimo_step) - t(misure_ottenute))
   dt = max(t_k(k+1) - t_meas, 0);
   % chiamata al solver MPC
   [u_k, ~, x_hat2, ts_drone(k)] = mpcStep(mpc_drn, model, ode_opt, ...
   x_meas, dt, u_applied, xw, uw, Ts, tic_start, t_k(k+1));
   
   % prossimo setpoint
   sp_next = [x_hat2(1:3); atan2(sin(x_hat2(9)), cos(x_hat2(9)))];
   U_drone(:, k) = u_k;
   u_applied     = u_k;
end

% fine loop
maintain_timing(tic_start, t_k(N+1));
X_drone(:, N+1) = measureState(drone);
t_drone(N+1)    = toc(tic_start);

psi = atan2(sin(x_ref(9, end)), cos(x_ref(9, end)));
drone.sendPositionSetpoint(x_ref(1:3, end), psi);
drone.land();
drone.disarm();
drone.disconnect();
Drone.shutdown();
  
save(sprintf('MPC_test_%s_log.mat', SHAPE), 'SHAPE', 'Ts', 't_k', 'x_ref', 'u_ref', ...
    'X_matlab', 'U_matlab', 'ts_matlab', 'X_drone', 't_drone', 'U_drone', 'ts_drone');

%% 4. PLOT
valid = all(isfinite(X_drone), 1);
wrap  = @(X) [X(1:8, :); atan2(sin(X(9, :)), cos(X(9, :))); X(10:12, :)];
Xr = wrap(x_ref);  Xm = wrap(X_matlab);  Xd = wrap(X_drone);

% Traiettoria XY
figure('Name', ['Tracking XY - ' SHAPE], 'Color', 'w');
plot(Xr(2, :), Xr(1, :), 'k--', 'LineWidth', 1.3); hold on;
plot(Xm(2, :), Xm(1, :), 'b-', 'LineWidth', 1.3);
plot(Xd(2, valid), Xd(1, valid), 'r-', 'LineWidth', 1.3);
plot(p_ic(2), p_ic(1), 'ko', 'MarkerFaceColor', 'k');
axis equal; grid on;
xlabel('East [m]'); ylabel('North [m]');
legend('Reference', 'MATLAB', 'Drone', 'Start', 'Location', 'best');
title(sprintf('%s | Ts = %.2f s', SHAPE, Ts));

% Stati
lab = {'x_N [m]','y_E [m]','z_D [m]','v_N [m/s]','v_E [m/s]','v_D [m/s]', ...
       '\phi [deg]','\theta [deg]','\psi [deg]','p [rad/s]','q [rad/s]','r [rad/s]'};

figure('Name', ['Stati - ' SHAPE], 'Color', 'w', 'Position', [50 50 1400 800]);
for i = 1:12
    f = 1;
    if i >= 7 && i <= 9, f = 180/pi; end
    subplot(3, 4, i);
    plot(t_k, Xr(i, :) * f, 'k--', 'LineWidth', 1.2); hold on;
    plot(t_k, Xm(i, :) * f, 'b-', 'LineWidth', 1.2);
    plot(t_drone(valid), Xd(i, valid) * f, 'r-', 'LineWidth', 1.2);
    grid on; ylabel(lab{i});
    if i > 8, xlabel('t [s]'); end
    if i == 1, legend('Reference', 'MATLAB', 'Drone', 'Location', 'best'); end
end
sgtitle(sprintf('%s', SHAPE));

% Input applicati (u*_k agisce da t_{k+1}) e tempi del solver
inp = {'T [N]','L [N m]','M [N m]','N [N m]'};

figure('Name', ['Input - ' SHAPE], 'Color', 'w', 'Position', [100 100 1300 650]);
for i = 1:4
    subplot(2, 3, i);
    stairs(t_k(2:N+1), u_ref(i, 2:N+1), 'k--', 'LineWidth', 1.2); hold on;
    stairs(t_k(2:N+1), U_matlab(i, :), 'b-', 'LineWidth', 1.2);
    stairs(t_k(2:N+1), U_drone(i, :), 'r-', 'LineWidth', 1.2);
    grid on; ylabel(inp{i}); xlabel('t [s]');
    if i == 1, legend('Reference', 'MATLAB', 'Drone', 'Location', 'best'); end
end

subplot(2, 3, 5:6);
plot(t_k(1:N), ts_matlab * 1e3, 'b-', 'LineWidth', 1.1); hold on;
plot(t_k(1:N), ts_drone  * 1e3, 'r-', 'LineWidth', 1.1);
yline(Ts * 1e3, 'k--', sprintf('T_s = %.0f ms', Ts * 1e3));
grid on; xlabel('t [s]'); ylabel('Tempo QP [ms]');
legend('MATLAB', 'Drone', 'Location', 'best');
title(sprintf('medio %.1f / %.1f ms | max %.1f / %.1f ms (MATLAB / drone)', ...
    mean(ts_matlab) * 1e3, mean(ts_drone) * 1e3, max(ts_matlab) * 1e3, max(ts_drone) * 1e3));


%% FUNZIONI LOCALI

% Funzione di risoluzione MPC controller
% MATLAB -> integra u_opt e torna x_hat
% PLANT -> integra anche per x_hat 2 per capire dove mandare setpoint
function [u_k, x_hat1, x_hat2, ts] = mpcStep(mpc, model, ode_opt, x_meas, dt, u_applied, xw, uw, Ts, t0, t_dead)
    % Predizione a t_{k+1} con u_applied, QP, predizione a t_{k+2} con u*
    x_hat1 = x_meas;
    if dt > 0
        [~, xo] = ode45(@(t, x) model.dynamics(t, x, u_applied), [0 dt], x_meas, ode_opt);
        x_hat1 = xo(end, :)';
    end

    tic_s = tic;
    [u_opt, dbg] = mpc.solve(x_meas, xw, uw, [dt; u_applied]);
    ts = toc(tic_s);

    if strcmp(dbg.status, 'solved') && toc(t0) < t_dead
        u_k = u_opt;
    else
        u_k = u_applied;     % tieni quello vecchio
    end

    [~, xo] = ode45(@(t, x) model.dynamics(t, x, u_k), [0 Ts], x_hat1, ode_opt);
    x_hat2 = xo(end, :)';
end

% Builda la shape
function [traj_obj, T_end, periodic] = makeShape(name, tp)
    switch name
        case 'Circle'
            traj_obj = ShapeCircle(tp.Circle);            T_end = tp.T_duration_circle;      periodic = true;
        case 'CircleDeployment'
            traj_obj = ShapeCircleDeployment(tp.CircleDeployment);            T_end = tp.T_duration_circle_deployment;      periodic = true;
        case 'Lemniscate'
            traj_obj = ShapeLemniscate2(tp.Lemniscate);   T_end = tp.T_duration_lemniscate;  periodic = true;
        case 'Spiral'
            traj_obj = ShapeSpiral(tp.Spiral);            T_end = tp.T_duration_spiral;      periodic = false;
        case 'Lissajous'
            traj_obj = ShapeLissajous3D(tp.Lissajous3D);  T_end = tp.T_duration_lissajous3D; periodic = true;
        otherwise
            error('Shape sconosciuta: %s', name);
    end
end

% misura lo stato del drone (getter)
function x = measureState(drone)
    [pn, pe, pd] = drone.getPosition();
    [vn, ve, vd] = drone.getVelocity();
    [ph, th, ps] = drone.getEuler();
    [p,  q,  r ] = drone.getAngularVelocity();
    x = [pn; pe; pd; vn; ve; vd; ph; th; atan2(sin(ps), cos(ps)); p; q; r];
end

% Ottieni reference lunga p
function [xw, uw] = refWindow(x_ref, u_ref, k0, p, x_hover, u_hover)
    xw = zeros(size(x_ref, 1), p + 1);
    uw = zeros(4, p);
    T  = size(x_ref, 2);
    for j = 0:p
        idx = k0 + j;
        if idx <= T
            xw(:, j+1) = x_ref(:, idx);
            if j < p, uw(:, j+1) = u_ref(:, idx); end
        else
            xw(:, j+1) = x_hover;
            if j < p, uw(:, j+1) = u_hover; end
        end
    end
end

% Funzione per mantenere timing
function maintain_timing(tic_start, t_deadline)
    sleep_time = t_deadline - toc(tic_start);
    if sleep_time > 0
        pause(sleep_time);
    end
end