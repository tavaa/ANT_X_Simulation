% SHAPELISSAJOUS3D 3D Lissajous trajectory.
%
% Flat outputs:
%   sigma = [x_I; y_I; z_I; psi]
%
% Parameters:
%   Ax       [m]       X-axis amplitude
%   Ay       [m]       Y-axis amplitude
%   Az       [m]       Z-axis amplitude
%   omega    [rad/s]  Base angular velocity
%   z_height [m NED]  Mean height
%   psi0     [rad]    Initial heading offset
%   center   [m]      XY center
%
% Trajectory:
%   x = cx + Ax*sin(omega*t)
%   y = cy + Ay*sin(2*omega*t)
%   z = z_height - Az*cos(3*omega*t)
%
% The negative sign in z follows the NED convention:
% z < 0 corresponds to increasing altitude.
%
% Yaw:
%   Heading aligned with the horizontal velocity vector.

classdef ShapeLissajous3D < TrajectoryBase

    methods

        function obj = ShapeLissajous3D(params)
            obj@TrajectoryBase(params);
        end

        function [sigma, dsigma, ddsigma, dddsigma, ddddsigma] = ...
                get_flat_outputs(obj, t)

            %% Parameters
            Ax = obj.params.Ax;
            Ay = obj.params.Ay;
            Az = obj.params.Az;
            w  = obj.params.omega;
            z0 = obj.params.z_height;

            cx = obj.params.center(1);
            cy = obj.params.center(2);

            wt = w * t;

            %% Position
            x = cx + Ax * sin(wt);
            y = cy + Ay * sin(2*wt);
            z = z0 - Az * cos(3*wt);

            %% Velocity
            xd =  Ax * w * cos(wt);
            yd =  2 * Ay * w * cos(2*wt);
            zd =  3 * Az * w * sin(3*wt);

            %% Acceleration
            xdd = -Ax * w^2 * sin(wt);
            ydd = -4 * Ay * w^2 * sin(2*wt);
            zdd =  9 * Az * w^2 * cos(3*wt);

            %% Jerk
            xddd = -Ax * w^3 * cos(wt);
            yddd = -8 * Ay * w^3 * cos(2*wt);
            zddd = -27 * Az * w^3 * sin(3*wt);

            %% Snap
            xdddd =  Ax * w^4 * sin(wt);
            ydddd =  16 * Ay * w^4 * sin(2*wt);
            zdddd = -81 * Az * w^4 * cos(3*wt);

            %% YAW
            % Heading aligned with horizontal velocity vector
            v2    = xd^2 + yd^2;
            eps_v = 1e-8;

            psi = atan2(yd, xd);

            if v2 < eps_v
                psi_d    = 0;
                psi_dd   = 0;
                psi_ddd  = 0;
                psi_dddd = 0;

            else
                N1    = xd*ydd - yd*xdd;
                psi_d = N1 / v2;

                v2_d = 2*(xd*xdd + yd*ydd);
                N1_d = xd*yddd - yd*xddd;

                psi_dd = (N1_d*v2 - N1*v2_d) / v2^2;

                psi_ddd  = 0;
                psi_dddd = 0;
                
            end

            %% ASSEMBLY
            sigma = [x; y; z; psi];
            dsigma = [xd; yd; zd; psi_d];
            ddsigma = [xdd; ydd; zdd; psi_dd];
            dddsigma = [xddd; yddd; zddd; psi_ddd];
            ddddsigma = [xdddd; ydddd; zdddd; psi_dddd];

        end
    end
end