classdef ModelParameters
    % MODELPARAMETERS  Physical parameters of the ANT-X quadcopter.
    %
    % Sources:
    %   - Mass, arm length: ANT-X v2 spec sheet.
    %   - Inertia tensor: Cavagnini (2021) thesis.
    %   - C_T, C_Q: preliminary estimate by analogy.
    %
    % ALL VALUES BELOW ARE PRELIMINARY lab identification

    properties (Constant)
        %% PHYSICAL PARAMETERS
        %m   = 0.270;   % [kg]    Total mass v1
        m   = 0.363;   % [kg]     Total mass ANT-X v2 spec sheet
        b   = 0.08;    % [m]      Arm length, CoG to motor
        g   = 9.81;    % [m/s^2]  Gravitational acceleration
        rho = 1.225;   % [kg/m^3] Air density, sea level standard
        R   = 0.0381;  % [m]      Propeller radius
        D   = 0.0762;  % [m]      Propeller diameter

        %% INERTIA TENSOR (Diagonal, X-config symmetry: Jxx = Jyy > Jzz)
        % Cavagnini (2021) 
        Jxx = 0.00307;   % [kg*m^2] Roll inertia
        Jyy = 0.00307;   % [kg*m^2] Pitch inertia
        Jzz = 0.00239;   % [kg*m^2] Yaw inertia

        %% PROPULSION CONSTANTS
        C_T = 0.0343;    % [-] Thrust coefficient (preliminary)
        C_Q = 0.00841;   % [-] Torque coefficient (preliminary)

        % K_T, K_Q, Beta derived from C_T, C_Q, rho, R
        K_T  = 2.7823e-07;   % [N*s^2/rad^2]   Thrust constant
        K_Q  = 2.5994e-09;   % [N*m*s^2/rad^2] Torque constant
        Beta = 0.009340;     % [m] Torque-to-thrust ratio (K_Q/K_T)

        %% ROTATIONAL DAMPING
        Lp = -7.31e-04;   % [Nm*s]
        Mq = -7.31e-04;   % [Nm*s]
        Nr = 0;           % [Nm*s]

        %% TRANSLATIONAL DRAG
        %X_u = ;
        %Y_v = ;
        %Z_w = ;

        %% MOTOR CONSTANTS
        tau_accel = 0.035;   % [s] Accel-phase time constant
        tau_decel = 0.035;   % [s] Decel-phase time constant
    end
end