function trajectory = SolveEmbodiedFootprints(guess,cfg)
%SOLVEEMBODIEDFOOTPRINTS Run the paper's AMPL/IPOPT model using text files.
% Every invocation uses a clean temporary directory; stale results are rejected.
run_dir = tempname(cfg.output_dir); mkdir(run_dir);
copyfile(fullfile(cfg.project_dir,'models','*'),run_dir);
WriteProblemData(guess,cfg,run_dir);
runtime = getenv('EF_SOLVER_DIR');
if isempty(runtime), runtime = fullfile(cfg.project_dir,'runtime'); end
executable = fullfile(runtime,'ampl.exe');
if ~ispc, executable = fullfile(runtime,'ampl'); end
assert(isfile(executable),['AMPL not found. Install AMPL/IPOPT in runtime/ ', ...
    'or set EF_SOLVER_DIR. See README.md.']);
old_dir = pwd; old_path = getenv('PATH');
cleanup = onCleanup(@() RestoreEnvironment(old_dir,old_path)); %#ok<NASGU>
cd(run_dir); setenv('PATH',[runtime,pathsep,old_path]);
[exit_code,solver_log] = system(sprintf('"%s" SolveFootprints.run',executable));
fid = fopen(fullfile(cfg.output_dir,'solver.log'),'w'); fprintf(fid,'%s',solver_log); fclose(fid);
assert(exit_code==0 && isfile('status.txt'),'Solver process failed; see results/solver.log.');
fid = fopen('status.txt','r'); status = fscanf(fid,'%f'); fclose(fid);
assert(status(1)<100,'IPOPT did not report a solved result. See solver.log.');
fid = fopen('trajectory.txt','r'); values = fscanf(fid,'%f',[8,inf]).'; fclose(fid);
assert(isequal(size(values),[numel(guess.x),8]) && all(isfinite(values(:))), ...
    'Incomplete or nonfinite solver output.');
names = {'x','y','theta','v','a','phi','w','dt'};
for column=1:numel(names), trajectory.(names{column}) = values(:,column).'; end
trajectory.dt = max(0,trajectory.dt);
trajectory.time = [0,cumsum(trajectory.dt(1:end-1))];
copyfile('trajectory.txt',fullfile(cfg.output_dir,'trajectory.txt'));
cd(old_dir);
canonical_run=char(java.io.File(run_dir).getCanonicalPath());
canonical_output=char(java.io.File(cfg.output_dir).getCanonicalPath());
assert(startsWith(lower(canonical_run),[lower(canonical_output),filesep]),'Unexpected temporary output path.');
rmdir(run_dir,'s');
end

function RestoreEnvironment(folder,path_value)
cd(folder); setenv('PATH',path_value);
end

function WriteProblemData(guess,cfg,folder)
v = cfg.vehicle; task = cfg.task; count = numel(guess.x);
boundary_theta = guess.theta(end); % same physical heading, unwrapped for continuity
parameters = [task.x0,task.y0,task.theta0,task.xf,task.yf,boundary_theta, ...
    count,v.v_max,v.phi_max,v.a_max,v.w_max,v.lw,v.LF,v.lr,v.hlb, ...
    cfg.environment.num_obs,cfg.sampling.max_dt];
fid = fopen(fullfile(folder,'parameters.dat'),'w');
fprintf(fid,'%d %.17g\n',[(1:numel(parameters));parameters]); fclose(fid);
fid = fopen(fullfile(folder,'obstacles.dat'),'w');
area_file = fopen(fullfile(folder,'obstacle_areas.dat'),'w');
for index=1:cfg.environment.num_obs
    obstacle=cfg.environment.obs(index); ox=obstacle.x(1:4); oy=obstacle.y(1:4);
    for vertex=1:4
        fprintf(fid,'%d %d 1 %.17g\n%d %d 2 %.17g\n',index,vertex,ox(vertex),index,vertex,oy(vertex));
    end
    area = polyarea(ox,oy);
    fprintf(area_file,'%d %.17g\n',index,area+min(0.02,0.02*area));
end
fclose(fid); fclose(area_file);
fid = fopen(fullfile(folder,'initial_guess.dat'),'w');
names={'x','y','theta','v','a','phi','w','dt'};
for index=1:count
    for field=1:numel(names)
        fprintf(fid,'let %s[%d] := %.17g;\n',names{field},index,guess.(names{field})(index));
    end
    kappa=tan(guess.phi(index))/v.lw; s=guess.v(index)*guess.dt(index);
    [box,extension]=EmbodiedBox(guess.x(index),guess.y(index),guess.theta(index),s,kappa,v);
    extra_names={'k','s','up','left','down','right','AX','AY','BX','BY','CX','CY','DX','DY'};
    extra_values=[kappa,s,extension,reshape(box.',1,[])];
    for field=1:numel(extra_names)
        fprintf(fid,'let %s[%d] := %.17g;\n',extra_names{field},index,extra_values(field));
    end
end
fprintf(fid,'let tf := %.17g;\n',sum(guess.dt)); fclose(fid);
end
