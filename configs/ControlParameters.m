% CONTROLPARAMS Parameters defines the Controllers and system's sampling
% time properties
% Defines the State and Inputs constraints, optimization horizon, sampling rate, and weighting matrices.
%
% Q_diag >= 0 (Positive semi-definite state cost).
% R_diag > 0 (Positive definite input cost for solver stability).

classdef ControlParameters
    
    properties (Constant)

        % Sampling time [s] Configuration
        Ts = 0.05;      

        %% State and Inputs Constraints
        % State Constraints: Bounds for both models
        x_min_12 = [-inf; -inf; -inf; -3; -3; -3; -deg2rad(35); -deg2rad(35); -inf; -deg2rad(600); -deg2rad(600); -deg2rad(600)];
        x_max_12 = [inf; inf; inf; 3; 3; 1; +deg2rad(35); +deg2rad(35); inf; deg2rad(600); deg2rad(600); deg2rad(600)];

        x_min_13 = [-inf; -inf; -inf; -3; -3; -3; -inf; -inf; -inf; -inf; -deg2rad(600); -deg2rad(600); -deg2rad(600)];
        x_max_13 = [inf; inf; inf; 3; 3; 1; +inf; +inf; +inf; +inf; deg2rad(600); deg2rad(600); deg2rad(600)];
        
        % Input Saturation Limits [Thrust, Roll, Pitch, Yaw]
        % u in [u_min, u_max].
        u_min = [0.0; -0.15; -0.15; -0.05]; 
        u_max = [ 2 * 0.270 * 9.81;  0.15;  0.15;  0.05]; 
        

        %% MPC Parameters
        % Prediction horizon
        p  = 18;  
        
        % State Weighting Matrix Q (Diagonal entries)
        % Q_diag_12 -> 12 states models - Euler's angles
        % Q_diag_13 -> 13 states models - Quaternions
        % Vector 12 states: [x_I, y_I, z_I, vx_B, vy_B, vz_B, phi, theta, psi, p, q, r] 
        % Vector 13 states: [x_I, y_I, z_I, vx_B, vy_B, vz_B, q0, q1, q2, q3, p, q, r] 
        Q_diag_12 = [50; 50; 50; 8; 8; 12; 8; 8; 10; 1; 1; 1];
        Qf_diag_12 = [50; 50; 50; 8; 8; 12; 8; 8; 10; 1; 1; 1];

        Q_diag_13  = [50; 50; 50; 8; 8; 12; 0; 32; 32; 40; 1; 1; 1];
        Qf_diag_13 = [50; 50; 50; 8; 8; 12; 0; 32; 32; 40; 1; 1; 1];

        % Input Weighting Matrix R (Diagonal entries)
        % Vector: [T, L, M, N]
        R_diag = [2.0; 2.0; 2.0; 2.0];

    end
end

