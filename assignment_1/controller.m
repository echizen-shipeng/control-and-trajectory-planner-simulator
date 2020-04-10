function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

%   pd controller parameters
kp = 200;
ki = 24;

global last_vz
%   compute the current error
error_z = s_des(1) - s(1);
error_vz = s_des(2) - s(2);
za = (s_des(2) - last_vz)/0.01;

u = params.mass * (za + kp * error_z + ki * error_vz + params.gravity)

last_vz = s_des(2);
% FILL IN YOUR CODE HERE


end

