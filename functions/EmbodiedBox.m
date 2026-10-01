function [vertices,expansion] = EmbodiedBox(x,y,theta,s,kappa,vehicle)
%EMBODIEDBOX Paper Eqs. (29) and (32); scalar pose, forward travel s >= 0.
% expansion = [front,left,rear,right], in metres.
turn = kappa*s;
expansion = [s+vehicle.hlb*abs(turn), ...
    max(-vehicle.lr*turn,(vehicle.LF+0.5*s)*turn),0, ...
    max(vehicle.lr*turn,-(vehicle.LF+0.5*s)*turn)];
front = vehicle.LF+expansion(1); rear = vehicle.lr;
left = vehicle.hlb+expansion(2); right = vehicle.hlb+expansion(4);
local = [front,left;front,-right;-rear,-right;-rear,left];
rotation = [cos(theta),-sin(theta);sin(theta),cos(theta)];
vertices = local*rotation.'+[x,y];
end
