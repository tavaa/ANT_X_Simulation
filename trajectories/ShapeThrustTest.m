classdef ShapeThrustTest < TrajectoryBase
% SHAPETHRUSTTEST  Pure vertical climb from near-ground altitude.
%
% Flat outputs:
%   sigma = [0; 0; z(t); 0]
%
% Smooth sinusoidal altitude profile (C^inf):
%   z(t) = z_ground - H/2 * (1 - cos(omega*t))
%
% Starting at z = z_ground (z_ground = -0.50m = 50cm up)
% the drone smoothly climbs to z = z_ground - H (altitude = H meters)
% and returns, completing one oscillation per period T = 2*pi/omega.
%
% This trajectory isolates vertical dynamics:
%   T = m*(g + z_ddot_compensation) > mg when climbing
%   T < mg when descending
%   L = M = N = 0  (no attitude change, pure altitude control)
%
% Parameters (struct)
%   H          [m]     Climb height (total excursion from start)
%   omega      [rad/s] Oscillation frequency
%   z_ground   [m NED] Starting altitude in NED 
%
% z_ground should be slightly negative (just above ground) to avoid
% singularity at T=0 (assert).

    methods

        function obj = ShapeThrustTest(params)
            obj@TrajectoryBase(params);
        end

        function [sigma, dsigma, ddsigma, dddsigma, ddddsigma] = ...
                get_flat_outputs(obj, t)
            
            %% Parameters
            H   = obj.params.H;
            w   = obj.params.omega;
            z0  = obj.params.z_ground;

            %% POSITION — x and y fixed, z climbs
            %
            % z(t) = z0 - H/2*(1 - cos(omega*t))
            %      = z0 - H/2 + H/2*cos(omega*t)
            %
            % At t=0:     z = z0              (starting altitude)
            % At t=pi/w:  z = z0 - H          (peak altitude, H meters above start)
            % At t=2pi/w: z = z0              (back to start)
            %
            
            % Position
            x    = 0;   y    = 0;
            z    = z0 - (H/2)*(1 - cos(w*t));
            
            % Velocity
            xd   = 0;   yd   = 0;
            zd   = - (H/2) * w * sin(w*t);          
            
            % Acceleration
            xdd  = 0;   ydd  = 0;
            zdd  = - (H/2) * w^2 * cos(w*t);       
            
            % Jerk
            xddd = 0;   yddd = 0;
            zddd = + (H/2) * w^3 * sin(w*t);
            
            % Snap
            xdddd = 0;  ydddd = 0;
            zdddd = + (H/2) * w^4 * cos(w*t);

            %% YAW — fixed 
            psi      = 0;
            psi_d    = 0;
            psi_dd   = 0;
            psi_ddd  = 0;
            psi_dddd = 0;

            %% ASSEMBLY
            sigma     = [x; y; z; psi];
            dsigma    = [xd; yd; zd; psi_d];
            ddsigma   = [xdd; ydd; zdd; psi_dd];
            dddsigma  = [xddd; yddd; zddd; psi_ddd];
            ddddsigma = [xdddd; ydddd; zdddd; psi_dddd];

        end

    end
end