% TRAJECTORYPARAMETERS Config class for reference trajectory generation.
% All spatial coordinates use the NED frame (z < 0 is above ground).
classdef TrajectoryParameters
    
    properties (Constant)

        %% HOVER
        Hover = struct( ...
            'pos', [0.0; 0.0; -1.0], ... % [m] NED pos (1m up)
            'psi', 0.0 ...               % [rad] Yaw angle
        );

        %% SINGLE DOF TESTS (Oscillatory)
        % Pitch test
        Pitch = struct( ...
              'A', 1.0, ...              % [m] Amplitude
              'omega', 1.0, ...          % [rad/s] Freq
              'z_height', -1.0 ...       % [m] NED alt
        );
        
        % Roll test
        Roll = struct( ...
              'A', 1.0, ...              % [m] Amplitude
              'omega', 1.0, ...          % [rad/s] Freq
              'z_height', -1.0 ...       % [m] NED alt
        );
        
        % Yaw test
        Yaw = struct( ...
              'A', +pi/2, ...             % [rad] Amplitude
              'omega', 1.0, ...          % [rad/s] Freq
              'z_height', -1.0 ...       % [m] NED alt
        );
        
        % Thrust test
        Thrust = struct( ...
              'H', 1.0, ...              % [m] Climb height
              'omega', 1.0, ...          % [rad/s] Freq
              'z_ground', -0.50 ...      % [m] NED start alt
        );

        %% 2D Shapes - XY 
        % Circular trajectory
        Circle = struct( ...
            'R', 1.0, ...                % [m] Radius
            'omega', 1.0, ...            % [rad/s] Angular speed
            'z_height', -1.0, ...        % [m] NED alt
            'center', [0.0; 0.0] ...    % [m] XY center
        );
        
        % Lemniscate trajectory
        Lemniscate = struct( ...
            'R', 1.0, ...                % [m] Max amplitude scale
            'omega', 0.5, ...            % [rad/s] Angular speed
            'z_height', -1.0, ...        % [m] NED alt
            'center', [0.0; 0.0] ...     % [m] XY center
        );

        % Lemniscate2 trajectory
        Lemniscate2 = struct( ...
            'R', 1.0, ...                % [m] Max amplitude scale
            'omega', 0.5, ...            % [rad/s] Angular speed
            'z_height', -1.0, ...        % [m] NED alt
            'center', [0.0; 0.0] ...     % [m] XY center
        );

        %% 3D Shapes - XYZ
        Spiral = struct( ...
            'Rx', 2.0, ...               % [m] X-axis radius
            'Ry', 1.0, ...               % [m] Y-axis radius
            'omega', 0.5, ...            % [rad/s] Angular speed
            'vz', 0.2, ...               % [m/s] Climb rate
            'z_start', -0.5 ...         % [m] NED start alt
        );

        %% SIMULATION DURATIONS [s]
        % T = (n_laps * 2*pi) / omega
        T_duration_hover      = 5.0;            % Fixed duration
        T_duration_pitch      = (2*pi / 1.0);     % 1 period
        T_duration_roll       = (2*pi / 1.0);     % 1 period
        T_duration_yaw        = (2*pi / 1.0);     % 1 period
        T_duration_thrust     = (2*pi / 1.0);     % 1 period
        T_duration_circle     = (1 * 2 *pi / 1.0);     % 1 lap
        T_duration_lemniscate = (1 * 2 * pi / 0.5);     % 1 figure-8 lap
        T_duration_lemniscate2 = (1 * 2 * pi / 0.5);     % 1 figure-8 lap
        T_duration_spiral     = (4*pi / 0.5);     % 2 revolutions
        
    end
end