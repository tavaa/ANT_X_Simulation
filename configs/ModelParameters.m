% MODELPARAMETERS Physical parameters of the ANT-X quadcopter.

classdef ModelParameters
    
    properties (Constant)

        %% KINEMATICS & MASS
        b = 0.08;       % [m]      Arm length (CoG to motor, 160mm frame)
        m = 0.270;      % [kg]     Total mass
        g = 9.81;       % [m/s^2]  Gravitational acceleration

        %% INERTIA TENSOR (Diagonal, X-config symmetry)
        % Due to symmetry, (Jxx ~ Jyy) > Jzz .
        Jxx = 0.00307;   % [kg*m^2] Roll inertia
        Jyy = 0.00307;   % [kg*m^2] Pitch inertia
        Jzz = 0.00239;   % [kg*m^2] Yaw inertia

        %% PROPULSION CONSTANTS
        % C_T = ?;      % [-] Thrust coefficient
        % C_Q = ?;      % [-] Torque coefficient

    end
end