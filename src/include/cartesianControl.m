%% Kinematic Model Class - GRAAL Lab
classdef cartesianControl < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        k_a
        k_l
    end

    methods
        % Constructor to initialize the geomModel property
        function self = cartesianControl(gm,angular_gain,linear_gain)
            if nargin > 2
                self.gm = gm;
                self.k_a = angular_gain;
                self.k_l = linear_gain;
            else
                error('Not enough input arguments (cartesianControl)')
            end
        end
        function [x_dot]=getCartesianReference(self,bTg)
            %% getCartesianReference function
            % Inputs :
            % bTg : goal frame
            % Outputs :
            % x_dot : cartesian reference for inverse kinematic control
            x_dot = zeros(6,1);
            
            bTt = self.gm.getToolTransformWrtBase();
            bRt = bTt(1:3,1:3);
            
            tTg = bTt \ bTg;

            t_r = tTg(1:3, 4);
            b_r = bRt * t_r;
            
            tRg = tTg(1:3, 1:3);
            [h, theta] = RotToAngleAxis(tRg);
            t_rho= theta*h;
            b_rho = bRt * t_rho;
               
            % cartesian error
            b_e = [b_rho; b_r];
            % gain matrix
            Lambda = [self.k_a*eye(3) zeros(3); zeros(3) self.k_l*eye(3)];

            x_dot = Lambda * b_e;
        end
    end
end

