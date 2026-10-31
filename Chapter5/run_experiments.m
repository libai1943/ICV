function summary=run_experiments(sizes)
if nargin<1,sizes=[9,12,15];end
setup_solver();root=fileparts(mfilename('fullpath'));out=fullfile(root,'output');if ~isfolder(out),mkdir(out);end;summary=struct([]);
for count=sizes
 scene=make_scenario(count);sfm=simulate_sfm(scene);[reference,initialization]=prepare_reference(scene,sfm);save(fullfile(out,sprintf('reference_%02d.mat',count)),'scene','reference','sfm','initialization');
 result=optimize_trajectories(scene,reference);result.initialization=initialization;save(fullfile(out,sprintf('case_%02d.mat',count)),'result','reference','-v7');
 item=struct('vehicles',count,'wall_seconds',result.wall_seconds,'success',result.success,'history',result.history,'validation',rmfield(result.validation,'clearance'));summary=[summary,item];
 file=fopen(fullfile(out,sprintf('case_%02d.json',count)),'w');fprintf(file,'%s',jsonencode(item));fclose(file);
 fprintf('INTERSECTION %d: success=%d, clearance bound=%.4f\n',count,result.success,result.validation.interval_clearance_bound);
end
file=fopen(fullfile(out,'experiment_report.json'),'w');fprintf(file,'%s',jsonencode(summary));fclose(file);
end
