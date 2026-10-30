%% Prova2_offset - MPC in closed loop su Gazebo, comando differito di un passo
% Plant: drone (PX4 + Gazebo). Comando = setpoint di posizione + yaw ottenuto
% integrando il modello non lineare A02 con u* (non si inviano L,M,N,F).
% Frame NED / FRD, stato x = [p; v; phi theta psi; p q r], u = [T; L; M; N].
%
% Il QP calcolato al tick k parte dallo stato PREDETTO a t_{k+1} e il suo
% risultato viene inviato al tick k+1 (istante deterministico).
%
% Timeline del tick k (t_k = (k-1)*Ts, t_{k+1} = k*Ts):
%   1) ATTESA   fino a t_k
%   2) INVIO    del setpoint calcolato al tick k-1 (u_applied agisce su [t_k, t_{k+1}))
%   3) MISURA   x_k = x(t_k) dal drone
%   4) PREDIZIONE x^(t_{k+1}) = ode45(A02, x_k, u_applied, t_{k+1}-t_meas)
%   5) MPC      QP da x^(t_{k+1}), riferimento dal nodo k+1 -> u*_k su [t_{k+1}, t_{k+2})
%   6) SETPOINT x^(t_{k+2}) = ode45(A02, x^(t_{k+1}), u*_k, Ts), tenuto pronto
%               per l'invio al tick k+1
%   Se il QP fallisce o finisce dopo t_{k+1}: u*_k = u_applied (hold).

clear; clc; close all;

%% PATHS
addpath('models');
addpath('flatness');
addpath('configs');
addpath('control/MPC');
addpath('trajectories');
addpath('utils');
addpath('casadi');
addpath('deployment');


%% SCELTA (una sola shape, una sola IC)
SHAPE = 'Circle';      % 'Circle' | 'Lemniscate' | 'Spiral' | 'Lissajous'
IC    = 'Origin';      % 'Origin' | 'Perturbation'


%% SETUP
ns = '/antx';                 % stesso namespace di Prova1

mp = ModelParameters;
cp = controlParameters;
tp = TrajectoryParameters;
Ts = cp.Ts;

model   = A02_SimplifiedModel_Deployment(mp);   % modello dentro l'MPC e per il setpoint
mpc     = A02_MPCController(model);
ode_opt = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);

u_hover     = [mp.m * mp.g; 0; 0; 0];
pert_xy     = [0.20; -0.20];     % [m] offset IC 'Perturbation'
origin_pos  = [0; 0; -1];        % [m] IC 'Origin'
max_tilt    = deg2rad(35);       % [rad] soglia di abort
max_pos_err = 1.5;               % [m]   soglia di abort
REFERENCE_OFFSET = 1;            % la finestra dell'MPC parte dal nodo k+1

% Warm-up OSQP (problema di hovering, risultato scartato)
x_w = zeros(12, 1);
mpc.init(x_w, zeros(12, cp.p + 2), repmat(u_hover, 1, cp.p + 2));
mpc.solve(x_w, zeros(12, cp.p + 1), repmat(u_hover, 1, cp.p));


%% 1. RIFERIMENTO (NLP, PWC)
[traj_obj, T_end, periodic] = makeShape(SHAPE, tp);
nlp_opts = struct('periodic', periodic, 'print_level', 0);
[x_ref, u_ref, ~, ~] = A02_GeneratePWC_reference(traj_obj, T_end, Ts, nlp_opts);

N   = size(x_ref, 2) - 1;        % numero di tick
t_k = (0:N) * Ts;

% Target di hovering oltre la fine del riferimento
x_hover = x_ref(:, end);
x_hover([4:6 7:8 10:12]) = 0;

% Setpoint iniziale del drone
switch IC
    case 'Perturbation'
        p_ic = x_ref(1:3, 1) + [pert_xy; 0];
    case 'Origin'
        p_ic = origin_pos;
    otherwise
        error('IC sconosciuta: %s', IC);
end
psi_ic = 0;


%% 2. LOG
X_drone = nan(12, N+1);     % stato misurato
t_drone = nan(1, N+1);      % istante di misura [s]
X_mdl   = nan(12, N+1);     % stato predetto dal modello = setpoint inviato
U_mpc   = nan(4, N);        % u* usato a ogni tick
t_solve = nan(1, N);        % tempo del solver [s]
qp_ok   = false(1, N);
late    = false(1, N);      % QP finito dopo t_{k+1}
t_send  = nan(1, N);        % istante reale di invio [s]

u_applied = u_hover;        % input che agisce sul tick corrente
sp_next   = [p_ic; psi_ic]; % setpoint da inviare al prossimo tick


%% 3. DEPLOYMENT
drone = [];

try
    Drone.startup();

    drone = Drone(ns);
    drone.connect();
    drone.arm();
    pause(2);

    drone.takeoff(p_ic(3));
    pause(5);

    drone.sendPositionSetpoint(p_ic, psi_ic);
    pause(5);

    % Stato iniziale misurato -> init del controllore
    x0 = measureState(drone);
    mpc.init(x0, x_ref(:, 1+REFERENCE_OFFSET:end), u_ref(:, 1+REFERENCE_OFFSET:end));

    t0 = tic;

    for k = 1:N
        % 1) ATTESA fino a t_k
        while toc(t0) < t_k(k)
        end

        % 2) INVIO del setpoint calcolato al tick precedente
        drone.sendPositionSetpoint(sp_next(1:3), sp_next(4));
        t_send(k) = toc(t0);

        % 3) MISURA
        x_meas = measureState(drone);
        t_meas = toc(t0);
        X_drone(:, k) = x_meas;
        t_drone(k)    = t_meas;

        % Sicurezza
        if acos(cos(x_meas(7)) * cos(x_meas(8))) > max_tilt
            error('Abort: tilt > %.0f deg a t = %.2f s', rad2deg(max_tilt), t_k(k));
        end
        if norm(x_meas(1:3) - x_ref(1:3, k)) > max_pos_err
            error('Abort: errore di posizione > %.2f m a t = %.2f s', max_pos_err, t_k(k));
        end

        % 4) PREDIZIONE a t_{k+1} con l'input applicato
        dt = max(t_k(k+1) - t_meas, 0);
        x_hat1 = x_meas;
        if dt > 0
            [~, x_ode] = ode45(@(t, x) model.dynamics(t, x, u_applied), [0 dt], x_meas, ode_opt);
            x_hat1 = x_ode(end, :)';
        end
        X_mdl(:, k+1) = x_hat1;

        % 5) MPC: finestra dal nodo k+1; seg = parte residua di u_applied
        [xw, uw] = refWindow(x_ref, u_ref, k + REFERENCE_OFFSET, cp.p, x_hover, u_hover);
        seg = [dt; u_applied];

        ts = tic;
        [u_opt, dbg] = mpc.solve(x_meas, xw, uw, seg);
        t_solve(k) = toc(ts);

        qp_ok(k) = strcmp(dbg.status, 'solved');
        late(k)  = toc(t0) >= t_k(k+1);

        if qp_ok(k) && ~late(k)
            u_k = u_opt;
        else
            u_k = u_applied;     % hold
        end
        U_mpc(:, k) = u_k;

        % 6) SETPOINT per il tick k+1: x^(t_{k+2}) con u_k su [t_{k+1}, t_{k+2})
        [~, x_ode] = ode45(@(t, x) model.dynamics(t, x, u_k), [0 Ts], x_hat1, ode_opt);
        x_hat2  = x_ode(end, :)';
        sp_next = [x_hat2(1:3); atan2(sin(x_hat2(9)), cos(x_hat2(9)))];
        u_applied = u_k;
    end

    % Ultimo campione a t_N
    while toc(t0) < t_k(N+1)
    end
    X_drone(:, N+1) = measureState(drone);
    t_drone(N+1)    = toc(t0);
    X_mdl(:, 1)     = X_drone(:, 1);

    save('Prova2_offset_log.mat', 'SHAPE', 'IC', 'Ts', 't_k', 'x_ref', 'u_ref', ...
        'X_drone', 't_drone', 'X_mdl', 'U_mpc', 't_solve', 'qp_ok', 'late', 't_send');

    % Setpoint finale (fine riferimento) e atterraggio
    psi = atan2(sin(x_ref(9, end)), cos(x_ref(9, end)));
    drone.sendPositionSetpoint(x_ref(1:3, end), psi);
    pause(3);
    drone.land();
    pause(5);
    drone.disarm();
    drone.disconnect();
    Drone.shutdown();

catch ME
    fprintf(2, 'Errore deployment: %s\n', ME.message);

    save('Prova2_offset_log_abort.mat', 'SHAPE', 'IC', 'Ts', 't_k', 'x_ref', 'u_ref', ...
        'X_drone', 't_drone', 'X_mdl', 'U_mpc', 't_solve', 'qp_ok', 'late', 't_send');

    if ~isempty(drone)
        try
            x_now = measureState(drone);
            drone.sendPositionSetpoint(x_now(1:3), x_now(9));
            pause(2);
            drone.land();
            pause(5);
            drone.disarm();
            drone.disconnect();
        catch
        end
    end

    try
        Drone.shutdown();
    catch
    end

    rethrow(ME);
end


%% 4. PLOT: STATI (riferimento vs drone in closed loop)
lab = {'x_N [m]','y_E [m]','z_D [m]','v_N [m/s]','v_E [m/s]','v_D [m/s]', ...
       '\phi [deg]','\theta [deg]','\psi [deg]','p [rad/s]','q [rad/s]','r [rad/s]'};
ang = 7:9;

Xr = x_ref;
Xr(9, :) = atan2(sin(Xr(9, :)), cos(Xr(9, :)));
Xm = X_mdl;
Xm(9, :) = atan2(sin(Xm(9, :)), cos(Xm(9, :)));

figure('Name', ['Stati - ' SHAPE ' ' IC], 'Color', 'w', 'Position', [50 50 1400 800]);
for i = 1:12
    f = 1;
    if ismember(i, ang), f = 180/pi; end

    subplot(3, 4, i);
    plot(t_k, Xr(i, :) * f, 'k--', 'LineWidth', 1.2);
    hold on;
    plot(t_k, Xm(i, :) * f, 'b.-', 'LineWidth', 0.8);
    plot(t_drone, X_drone(i, :) * f, 'r-', 'LineWidth', 1.3);
    grid on;
    ylabel(lab{i});
    if i > 8, xlabel('t [s]'); end
    if i == 1, legend('Riferimento', 'Predizione modello', 'Drone', 'Location', 'best'); end
end
sgtitle(sprintf('%s | %s | Ts = %.2f s', SHAPE, IC, Ts), 'Interpreter', 'none');


%% 5. PLOT: INPUT (u* MPC vs riferimento) + TEMPI DEL SOLVER
inp = {'T [N]','L [N m]','M [N m]','N [N m]'};

figure('Name', ['Input - ' SHAPE ' ' IC], 'Color', 'w', 'Position', [100 100 1300 650]);
for i = 1:4
    subplot(2, 3, i);
    % u*_k agisce su [t_{k+1}, t_{k+2}): si disegna a partire da t_{k+1}
    stairs(t_k(2:N+1), u_ref(i, 2:N+1), 'k--', 'LineWidth', 1.2);
    hold on;
    stairs(t_k(2:N+1), U_mpc(i, :), 'r-', 'LineWidth', 1.3);
    grid on;
    ylabel(inp{i});
    xlabel('t [s]');
    if i == 1, legend('Riferimento', 'MPC u*', 'Location', 'best'); end
end

subplot(2, 3, 5:6);
plot(t_k(1:N), t_solve * 1e3, 'b-', 'LineWidth', 1.2);
hold on;
yline(Ts * 1e3, 'r--', sprintf('T_s = %.0f ms', Ts * 1e3));
if any(~qp_ok)
    plot(t_k(~qp_ok), t_solve(~qp_ok) * 1e3, 'rx', 'MarkerSize', 8);
end
grid on;
xlabel('t [s]');
ylabel('Tempo solver [ms]');
title(sprintf('medio %.1f ms | massimo %.1f ms | QP falliti %d | QP oltre t_{k+1} %d', ...
    mean(t_solve) * 1e3, max(t_solve) * 1e3, sum(~qp_ok), sum(late)));

sgtitle('Input: i comandi reali al drone sono i setpoint di posizione', 'Interpreter', 'none');


%% FUNZIONI LOCALI
function [traj_obj, T_end, periodic] = makeShape(name, tp)
    switch name
        case 'Circle'
            traj_obj = ShapeCircle(tp.Circle);            T_end = tp.T_duration_circle;      periodic = true;
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

function x = measureState(drone)
    % Stato A02 dai getter del drone, yaw in [-pi, pi]
    [pn, pe, pd] = drone.getPosition();
    [vn, ve, vd] = drone.getVelocity();
    [ph, th, ps] = drone.getEuler();
    [p,  q,  r ] = drone.getAngularVelocity();
    x = [pn; pe; pd; vn; ve; vd; ph; th; atan2(sin(ps), cos(ps)); p; q; r];
end

function [xw, uw] = refWindow(x_ref, u_ref, k0, p, x_hover, u_hover)
    % Finestra di riferimento sull'orizzonte, dal campione k0; oltre la fine: hovering
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
