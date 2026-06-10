function [x_ref, u_ref, t_steps] = GeneratePWC_reference(traj_obj, T_sim, Ts, model, flatness, options)
% GENERATEPWC_REFERENCE  Generates a Piece-Wise Constant (PWC) reference
%                        by integrating the nonlinear model forward with
%                        inputs held constant at each sampling instant.
%
% Supports both 12-state (Euler Z-Y-X) and 13-state (Quaternion) representations.
%
% At each step k the input u_k is computed via the flatness
% map evaluated at t = k*Ts, then held constant over [k*Ts, (k+1)*Ts]
%
% INPUTS
%   traj_obj  - trajectory object  (ShapeCircle | ShapeSpiral , ...)
%   T_sim     - total simulation duration  [s]
%   Ts        - sampling time              [s]
%   model     - nonlinear model object
%   flatness  - flatness map object (related to the nonlinear model)
%   options   - ode45 options struct  (odeset)
%
% OUTPUTS
%   x_ref     - nx x N_steps  state   trajectory matrix (nx = 12 or 13)
%   u_ref     -  4 x N_steps  input   trajectory matrix
%   t_steps   -  1 x N_steps  time    vector  [s]

    %% Model parameters
    mp = ModelParameters();

    %% Time grid
    N_steps = round(T_sim / Ts) + 1;
    t_steps = linspace(0, T_sim, N_steps);   

    %% Initial condition via flatness map at t = 0
    [s0, ds0, dds0, ddds0, dddds0] = traj_obj.get_flat_outputs(0);
    [x0, ~] = flatness.map(mp, s0, ds0, dds0, ddds0, dddds0);

    % State dimension (nx = 12 for Euler, 13 for Quaternion)
    nx = length(x0);
    if nx == 13
        % Normalize initial quaternion to prevent drift
        x0(7:10) = x0(7:10) / norm(x0(7:10));
    end

    %% Pre-allocate outputs
    x_ref = zeros(nx, N_steps);
    u_ref = zeros(4,  N_steps);

    x_ref(:, 1) = x0;
    x_curr      = x0;

    %% PWC Inputs integration
    for k = 1 : N_steps - 1

        t_now = t_steps(k);

        % Sample flat outputs at current time step 
        [s0, ds0, dds0, ddds0, dddds0] = traj_obj.get_flat_outputs(t_now);

        % Extract input from flatness map 
        [~, u_k] = flatness.map(mp, s0, ds0, dds0, ddds0, dddds0);

        % Store input at step k 
        u_ref(:, k) = u_k;

        % Integrate nonlinear dynamics with u_k held constant 
        tspan = [t_now, t_now + Ts];
        [~, x_sol] = ode45(@(t, x) model.dynamics(t, x, u_k), ...
                           tspan, x_curr, options);

        x_curr = x_sol(end, :)';
        
        % Normalize quaternion for the 13-state representation
        if nx == 13
            x_curr(7:10) = x_curr(7:10) / norm(x_curr(7:10));
        end
        
        x_ref(:, k+1) = x_curr;

    end

    % Unwrap yaw (12 states)
    if nx == 12
        x_ref(9,:) = unwrap(x_ref(9,:));    
    end

    %% Repeat last input to fill the final column
    u_ref(:, N_steps) = u_ref(:, N_steps - 1);

end