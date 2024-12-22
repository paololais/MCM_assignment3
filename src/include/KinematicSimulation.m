function [q] = KinematicSimulation(q, q_dot, dt, q_min, q_max)
%% Kinematic Simulation function
%
% Inputs
% - q current robot configuration
% - q_dot joints velocity
% - ts sample time
% - q_min lower joints bound
% - q_max upper joints bound
%
% Outputs
% - q new joint configuration
    
 q = q + q_dot * dt;
 q = max(q_min, min(q, q_max));
  
end