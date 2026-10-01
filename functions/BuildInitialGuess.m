function guess = BuildInitialGuess(path,cfg)
%BUILDINITIALGUESS Algorithm 1 / Section IV-B: PMP then greedy Eq. (37).
% Select the last valid dense waypoint before a prerequisite violation.
vehicle = cfg.vehicle;
distance = [0,cumsum(hypot(diff(path.x),diff(path.y)))];
length_total = distance(end);
peak_speed = min(vehicle.v_max,sqrt(length_total*vehicle.a_max));
ramp_time = peak_speed/vehicle.a_max;
cruise_time = max(0,(length_total-peak_speed^2/vehicle.a_max)/peak_speed);
total_time = 2*ramp_time+cruise_time;
time = linspace(0,total_time,ceil(total_time/cfg.sampling.dense_dt)+1);
velocity = min(peak_speed,min(vehicle.a_max*time,vehicle.a_max*(total_time-time)));
position = zeros(size(time));
accel = time<=ramp_time; cruise = time>ramp_time & time<=ramp_time+cruise_time;
decel = time>ramp_time+cruise_time;
position(accel) = 0.5*vehicle.a_max*time(accel).^2;
position(cruise) = 0.5*peak_speed*ramp_time+peak_speed*(time(cruise)-ramp_time);
position(decel) = length_total-0.5*vehicle.a_max*(total_time-time(decel)).^2;
dense_x = interp1(distance,path.x,position);
dense_y = interp1(distance,path.y,position);
dense_theta = interp1(distance,unwrap(path.theta),position);
curvature = gradient(unwrap(path.theta),distance);
dense_phi = atan(vehicle.lw*interp1(distance,curvature,position));
dense_phi = max(-vehicle.phi_max,min(vehicle.phi_max,dense_phi));
dense_phi([1,end]) = 0;
dense_kappa = tan(dense_phi)/vehicle.lw;
selected = 1; anchor = 1; count = numel(time);
while anchor<count
    candidate = anchor+1;
    while candidate<=count
        dt = time(candidate)-time(anchor);
        s = velocity(anchor)*dt; k = abs(dense_kappa(anchor));
        angle = k*s; slack = cfg.sampling.lambda;
        valid = dt<=slack*cfg.sampling.max_dt && angle<=slack*pi/2 ...
            && k*vehicle.LF*tan(angle)<=slack*(1+vehicle.hlb*k) ...
            && (1+vehicle.hlb*k)*tan(angle)<=slack*vehicle.lr*k;
        if ~valid, break; end
        candidate = candidate+1;
    end
    next = min(count,max(anchor+1,candidate-1));
    selected(end+1) = next; %#ok<AGROW>
    anchor = next;
end
guess.x = dense_x(selected); guess.y = dense_y(selected);
guess.theta = dense_theta(selected); guess.v = velocity(selected);
guess.phi = dense_phi(selected); guess.dt = [diff(time(selected)),0];
guess.a = [diff(guess.v)./guess.dt(1:end-1),0];
guess.w = [diff(guess.phi)./guess.dt(1:end-1),0];
guess.a = max(-vehicle.a_max,min(vehicle.a_max,guess.a));
guess.w = max(-vehicle.w_max,min(vehicle.w_max,guess.w));
guess.a([1,end]) = 0; guess.w([1,end]) = 0;
end
