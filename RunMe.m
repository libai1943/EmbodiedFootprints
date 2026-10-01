% 第十一章配套代码：《非结构化场景自动驾驶轨迹规划技术》（中文图书）
% English title translation: Trajectory Planning Techniques for Autonomous
% Driving in Unstructured Environments. The book is written in Chinese.
% Chapter 11: Embodied-footprint collision-avoidance constraints.
%
% Citation: B. Li et al., "Embodied Footprints: A Safety-Guaranteed
% Collision-Avoidance Model for Numerical Optimization-Based Trajectory
% Planning," IEEE TITS, 25(2), 2046-2060, 2024.
% DOI: 10.1109/TITS.2023.3316175 (published online in 2023).
%
% Select case_id, then Run. Two static figures are produced. Optional
% playback is display-only; this package contains no video export function.
% See README.md for AMPL/IPOPT installation and protected components.
clear; close all; clc;
case_id = 3;                  % 1, 2, or 3: original research scenarios
play_animation = false;      % optional display-only playback

project_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(project_dir,'functions'),fullfile(project_dir,'protected'));
configuration = ConfigureDemo(project_dir,case_id);
planning_clock = tic;
coarse_path = PlanCoarsePath(configuration);
initial_guess = BuildInitialGuess(coarse_path,configuration);
fprintf('Case %d: %d collocation nodes selected before optimization.\n',case_id,numel(initial_guess.x));
trajectory = SolveEmbodiedFootprints(initial_guess,configuration);
planning_seconds = toc(planning_clock);
validation = ValidateTrajectory(trajectory,configuration);
assert(validation.passed,'Final trajectory validation failed. See validation diagnostics.');
figures = PlotResults(trajectory,initial_guess,coarse_path,configuration);
save(fullfile(configuration.output_dir,'result.mat'),'trajectory','initial_guess', ...
    'coarse_path','configuration','validation','planning_seconds');
fprintf('Validated: T = %.3f s, planning = %.3f s.\n',sum(trajectory.dt),planning_seconds);
if play_animation
    PlayTrajectory(trajectory,configuration);
end
