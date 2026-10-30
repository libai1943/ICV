function reports=run_experiments()
% Actual feasibility demonstrations of the two-stage cooperative planner.
root=fileparts(mfilename('fullpath'));addpath(root,fullfile(root,'common'));
names={'baseline','lower_jerk','second_task'};seeds=[1943,1943,1944];limits=[.2,.1,.2];reports=struct([]);
for k=1:3
 cfg=struct('seed',seeds(k),'vehicle_count',10,'jerk_limit',limits(k), ...
   'output_directory',fullfile(root,'output',names{k}));
 book=run_ch04_readable(cfg);
 reports(k).case_name=names{k};reports(k).seed=seeds(k);reports(k).vehicle_count=10;
 reports(k).jerk_limit=limits(k);reports(k).stage1_seconds=book.stage1_duration;
 reports(k).stage2_seconds=book.time(end)-book.stage1_duration;reports(k).total_seconds=book.time(end);
 reports(k).planning_seconds=book.planning_seconds;reports(k).maximum_translation=max(abs(book.translations));
 reports(k).minimum_interval_clearance=book.interpolation_validation.minimum_interval_clearance_bound;
 reports(k).model=book.model_validation;reports(k).solver=book.solver;
 fid=fopen(fullfile(root,'output','experiment_report.json'),'w');fprintf(fid,'%s',jsonencode(reports));fclose(fid);
end
end
