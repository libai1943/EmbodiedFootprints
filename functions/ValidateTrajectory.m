function report = ValidateTrajectory(traj,cfg)
%VALIDATETRAJECTORY Check the model independently of the solver status.
vehicle=cfg.vehicle; dt=traj.dt(1:end-1); n=numel(traj.x);
curvature=tan(traj.phi(1:end-1))/vehicle.lw;
travel=traj.v(1:end-1).*dt;
residual=[diff(traj.x)-travel.*cos(traj.theta(1:end-1)), ...
    diff(traj.y)-travel.*sin(traj.theta(1:end-1)), ...
    diff(traj.theta)-travel.*curvature, ...
    diff(traj.v)-dt.*traj.a(1:end-1),diff(traj.phi)-dt.*traj.w(1:end-1)];
report.dynamics_residual=max(abs(residual));
angle=abs(curvature).*travel;
requirements=[angle-pi/2,abs(curvature)*vehicle.LF.*tan(angle)-(1+vehicle.hlb*abs(curvature)), ...
    (1+vehicle.hlb*abs(curvature)).*tan(angle)-vehicle.lr*abs(curvature)];
report.prerequisite_violation=max([0,requirements]);
report.bound_violation=max([0,-traj.v,traj.v-vehicle.v_max, ...
    abs(traj.a)-vehicle.a_max,abs(traj.phi)-vehicle.phi_max, ...
    abs(traj.w)-vehicle.w_max,-dt,dt-cfg.sampling.max_dt]);
task=cfg.task;
endpoint=[traj.x(1)-task.x0,traj.y(1)-task.y0,traj.theta(1)-task.theta0, ...
    traj.x(end)-task.xf,traj.y(end)-task.yf, ...
    atan2(sin(traj.theta(end)-task.thetaf),cos(traj.theta(end)-task.thetaf)), ...
    traj.v([1,end]),traj.a([1,end]),traj.phi([1,end]),traj.w([1,end])];
report.boundary_residual=max(abs(endpoint));
report.embodied_collisions=0; report.box_containment_violation=0;
for index=1:n
    kappa=tan(traj.phi(index))/vehicle.lw; s=traj.v(index)*traj.dt(index);
    [box,extension]=EmbodiedBox(traj.x(index),traj.y(index),traj.theta(index),s,kappa,vehicle);
    for obstacle=1:cfg.environment.num_obs
        shape=cfg.environment.obs(obstacle);
        if ConvexOverlap(box,[shape.x(1:4).',shape.y(1:4).'])
            report.embodied_collisions=report.embodied_collisions+1;
        end
    end
    % Check the local constant-v/kappa swept footprint against Eq. (29).
    % This numerical check supplements, rather than replaces, the prerequisites.
    for fraction=linspace(0,1,21)
        swept_angle=kappa*s*fraction;
        if abs(kappa)<1e-9, lx=s*fraction; ly=0;
        else, lx=sin(swept_angle)/kappa; ly=(1-cos(swept_angle))/kappa; end
        nominal=EmbodiedBox(lx,ly,swept_angle,0,0,vehicle);
        violation=max([0;nominal(:,1)-(vehicle.LF+extension(1)); ...
            -vehicle.lr-nominal(:,1);nominal(:,2)-(vehicle.hlb+extension(2)); ...
            -(vehicle.hlb+extension(4))-nominal(:,2)]);
        report.box_containment_violation=max(report.box_containment_violation,violation);
    end
end
report.passed = max([report.dynamics_residual,report.prerequisite_violation, ...
    report.bound_violation,report.boundary_residual,report.box_containment_violation])<1e-4 ...
    && report.embodied_collisions==0;
disp(report);
end

function overlap=ConvexOverlap(first,second)
edges=[diff([first;first(1,:)]);diff([second;second(1,:)])];
axes=[-edges(:,2),edges(:,1)];
overlap=true;
for index=1:size(axes,1)
    axis=axes(index,:); norm_axis=norm(axis);
    if norm_axis<1e-12, continue; end
    p=first*(axis/norm_axis).'; q=second*(axis/norm_axis).';
    if max(p)<=min(q)+1e-7 || max(q)<=min(p)+1e-7, overlap=false; return; end
end
end
