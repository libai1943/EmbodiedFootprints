function cfg = ConfigureDemo(project_dir,case_id)
%CONFIGUREDEMO Load the scene and the physical settings in paper Table I.
assert(ismember(case_id,1:3),'case_id must be 1, 2, or 3.');
data = load(fullfile(project_dir,'data','cases_data.mat'),'cases');
scene = data.cases(case_id);
cfg.project_dir = project_dir;
cfg.case_id = case_id;
cfg.output_dir = fullfile(project_dir,'results',sprintf('case_%d',case_id));
if ~isfolder(cfg.output_dir), mkdir(cfg.output_dir); end
cfg.environment.xmin = 0; cfg.environment.xmax = 30;
cfg.environment.ymin = 0; cfg.environment.ymax = 30;
cfg.environment.xhorizon = 30; cfg.environment.yhorizon = 30;
cfg.environment.obs = scene.obs;
cfg.environment.num_obs = numel(scene.obs);
cfg.task = scene.task;
cfg.vehicle.lf = 0.96; cfg.vehicle.lw = 2.8;
cfg.vehicle.LF = cfg.vehicle.lf + cfg.vehicle.lw;
cfg.vehicle.lr = 0.929; cfg.vehicle.lb = 1.942;
cfg.vehicle.hlb = cfg.vehicle.lb/2;
cfg.vehicle.length = cfg.vehicle.LF + cfg.vehicle.lr;
cfg.vehicle.v_max = 5; cfg.vehicle.a_max = 0.75;
cfg.vehicle.phi_max = 0.7; cfg.vehicle.w_max = 0.5;
assert(2*cfg.vehicle.lw > cfg.vehicle.lb*tan(cfg.vehicle.phi_max));
cfg.sampling.lambda = 0.9;
cfg.sampling.dense_dt = 0.001;
cfg.sampling.max_dt = 0.25; % original implementation's time-interval cap
cfg.ha.dx = 0.1; cfg.ha.dy = 0.1; cfg.ha.dtheta = 0.1;
cfg.ha.nx = 301; cfg.ha.ny = 301; cfg.ha.ntheta = ceil(2*pi/0.1)+1;
cfg.ha.num_phy_ha = 5; cfg.ha.penalty_on_phy_change = 0.01;
cfg.ha.penalty_on_biased_from_reference_line = 2;
cfg.ha.simu_unit_duration = 2; cfg.ha.iter_for_HA = 99000;
cfg.ha.multiplier_on_heuristics = 10; cfg.ha.nfe = 200;
cfg.guiding_path.x = []; cfg.guiding_path.y = [];
end
