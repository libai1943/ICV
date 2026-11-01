function result=resume_experiment(count)
% Resume a recorded hard-feasible checkpoint without changing its scenario.
setup_solver();root=fileparts(mfilename('fullpath'));out=fullfile(root,'output');
d=load(fullfile(out,sprintf('checkpoint_%02d.mat',count)));initial=load(fullfile(out,sprintf('reference_%02d.mat',count)));
assert(isfield(d.checkpoint.history,'interval_guards'),'Checkpoint predates whole-interval collision guards.');
result=optimize_trajectories(d.checkpoint.scene,d.checkpoint.trajectory,d.checkpoint);
result.initialization=initial.initialization;reference=initial.reference;
save(fullfile(out,sprintf('case_%02d.mat',count)),'result','reference','-v7');
fprintf('RESUMED %d: accepted=%d, clearance bound %.6f\n',count,result.success,result.validation.interval_clearance_bound);
end
